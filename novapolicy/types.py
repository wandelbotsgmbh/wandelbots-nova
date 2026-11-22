"""Data types for the policy package."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
import math
from typing import TYPE_CHECKING, Literal, TypeAlias

import pydantic

from nova.types import Pose

if TYPE_CHECKING:
    from nova.types import RobotState

# Reuse the SDK's IO value type
ValueType = int | str | bool | float | Pose

# Mode literals used across the package
ActionMode = Literal["absolute", "relative"]
JoggingMode = Literal["joint", "cartesian"]


class ActionChunk(pydantic.BaseModel, frozen=True):
    """Action chunk streamed to a waypoint jogging session.

    Single-step (teleoperation at 30 Hz)::

        ActionChunk(joints={"0@ur5e": [[0.1, -1.5, ...]]})

    TCP targets::

        ActionChunk(tcp={"0@ur5e": [[x, y, z, rx, ry, rz]]})

    Multi-step (policy inference, e.g. ACT outputs 16 steps at 33ms)::

        ActionChunk(
            joints={"0@ur5e": [[...], [...], [...], ...]},
            dt_ms=33.0,
        )
    """

    joints: dict[str, list[list[float]]] = pydantic.Field(default_factory=dict)
    """Motion group id → sequence of joint targets (radians)."""

    tcp: dict[str, list[list[float]]] = pydantic.Field(default_factory=dict)
    """Motion group id → sequence of TCP targets [x, y, z, rx, ry, rz]."""

    ios: dict[str, dict[str, bool | int | float | str]] | None = None
    """Motion group id → {io_key: value}, in **hardware** units. Fired once on send().

    Unlike joint and TCP targets, IO values arrive already converted: a policy
    client applies the schema's ``Mapping`` when it decodes its own wire format,
    because only the client knows where in that format the value sits. A chunk
    built by hand therefore carries hardware values directly — ``True``, or
    ``100.0`` for an analogue output — and nothing converts them again.

    Joint and TCP targets are the other way round: they arrive in the policy's
    units and the executor inverts them through
    :meth:`PolicySchema.apply_inverse_ops` before they reach the robot.
    """

    values: dict[str, list[list[float]]] = pydantic.Field(default_factory=dict)
    """Policy outputs for the keys ``Action.computed(..., keys=[...])`` hooks consume, one row
    per step: actuators outside NOVA, such as a gripper on its own bus. Every policy client
    fills it the same way, so a hook reads ``chunk.values[key]`` whatever the policy."""

    dt_ms: float = 0.0
    """Time spacing between steps in milliseconds. 0 = single-step."""

    first_timestamp_ms: int = -1
    """Where this chunk's first waypoint sits, in ms on the session clock
    (``jogger_session_timestamp_ms``, see :data:`novapolicy.jogging.clock.ServerMs`); not
    ``time.monotonic()`` and not wall-clock time.

    * ``>= 0``: the policy's own timeline. Step ``k`` is due ``first_timestamp_ms + k*dt``,
      exactly; steps already past are dropped. The chunk is never bridged.
    * ``-1`` (default): the executor places it (contract T1 in ``docs/contracts.md``): step ``k``
      is due ``max_expected_latency_ms + k*dt`` after the observation (continuous execution) or
      the send (sequential execution)."""

    action_timestep: int = -1
    """Absolute policy-queue timestep of the first action, or ``-1`` if absent.

    Fixed-rate queue clients use this to preserve one action timeline across
    NOVA chunk replacements. The executor maps timestep zero to the initial
    bridge boundary, then places later lookaheads at the corresponding absolute
    session timestamps instead of re-anchoring each replacement to ``now``."""

    seam_backdate_steps: int = 0
    """RTC seam backdate, in steps, for connecting overlapping chunks.

    The executor backdates the chunk anchor by ``seam_backdate_steps * dt_ms`` so
    the step matching the robot's current position lands at "now": the reused head
    sits in the immediate past (matching what's already executing) and the fresh
    prediction extends into the future. The client computes this as
    ``executed_steps - (action_horizon - overlap_steps)`` — the index in the new
    chunk that corresponds to where the robot currently is. (This equals the RTC
    frozen-step count only when overlap is unclamped, hence the field name.)
    0 = no RTC / no backdate."""

    retained_prefix_steps: int = 0
    """Leading steps a queue client already published to the controller: smoothing keeps them
    exactly as they were sent (the queue's predecessor and frozen head). ``0`` for any other
    chunk."""

    resolved: bool = False
    """The targets are already in NOVA units and absolute (the schema's operators and
    relative modes applied), as :class:`~novapolicy.queued_client.QueuedPolicyClient`
    returns them; the executor sends them as they are."""


@dataclass(frozen=True, slots=True)
class SequentialExecution:
    """Execute one complete chunk, reach standstill, then infer again.

    The executor's default pipeline eases each chunk in and out here
    (:class:`~novapolicy.jogging.modules.EndpointRamp`).
    """


@dataclass(frozen=True, slots=True)
class ContinuousExecution:
    """Continuously replace the active lookahead without per-chunk braking."""

    rate_hz: float | None = None
    """Fixed inference rate, or ``None`` to run as fast as inference allows."""

    def __post_init__(self) -> None:
        if self.rate_hz is not None and (not math.isfinite(self.rate_hz) or self.rate_hz <= 0):
            raise ValueError("rate_hz must be a positive finite value or None")


ExecutionMode: TypeAlias = SequentialExecution | ContinuousExecution


class OnStale(StrEnum):
    """What the executor does when data does not arrive in time.

    The trigger is per channel — one camera can go stale on its own — but the
    response is not: one policy drives the whole cell, so holding one arm while
    aborting another is incoherent. This is therefore an executor-wide setting.

    ``zeros``, the third option in ROS-side equivalents, is deliberately absent.
    It suits velocity and effort channels; NOVA streams absolute joint
    positions, where zero commands the arm to its zero pose.
    """

    HOLD = "hold"
    """Skip the tick and retry. The jogging session holds its last target while
    the robot decelerates. Escalates to ``CONTROLLED_STOP`` once the data has
    been stale for longer than the executor's hold budget.

    Honoured for observation staleness only. An inference deadline always ends
    the run: the timed-out call is still in flight on a worker thread, and
    re-entering the client while it may still complete would corrupt the
    policy's timestep sequence.
    """

    CONTROLLED_STOP = "controlled_stop"
    """Run out the waypoints already accepted, then end the run normally with a
    ``stale: ...`` reason."""

    ABORT = "abort"
    """Raise. The default, and what the executor did before this was declared."""


@dataclass(frozen=True, slots=True)
class WaypointConfig:
    """Configuration for NOVA waypoint jogging.

    Sends timestamped joint or TCP pose waypoints directly to the server,
    which handles velocity profiling, interpolation, and IK internally.

    New chunks override previous ones, so the server always tracks the
    freshest prediction.
    """

    state_rate_ms: int = 10
    """State stream update rate."""

    single_step_dt_ms: float = 50.0
    """Spacing used when a live jog target is sent as a single point.

    Which spacing executes most smoothly is a property of the controller rather
    than a universal constant, which is why this is config and not hardcoded.

    Must be positive: it is the divisor that turns the horizon length into a
    waypoint count, and it is the fallback spacing whenever a caller passes
    ``dt_ms=0``. ``0`` here has no "single step" meaning — it just leaves the
    live path with no timeline to lay waypoints on.
    """

    max_expected_latency_ms: float = 100.0
    """How far after its observation a chunk's first waypoint is due, in continuous execution.

    Step ``k`` follows ``k`` policy steps later. One-step chunks are spaced one policy step apart,
    and every request re-sends the earlier waypoints that are not due yet, so the controller
    always has the next few queued. Each one must
    reach the controller at least about 67 ms before it is due, so this covers that margin plus
    the network round trip. Set it for the worst latency of the deployment, as MoveIt Servo's
    ``max_expected_latency`` (default 0.1 s) does: 100 ms fits a policy running on the NOVA
    instance; over a remote network raise it. The arm then follows the policy by this much.
    When acknowledgements show the waypoints arriving too late for it, the session stops with a
    ``MotionError`` instead of letting the motion degrade.
    """

    delay_until_braking_ms: int | None = None
    """How long the robot keeps its velocity after a chunk's last waypoint before braking.

    ``None`` leaves the field out of the request, so the server brakes at the end
    of every chunk as before. With a delay, a chunk that arrives in time continues
    the motion instead of starting it again from standstill, and every chunk
    restarts the countdown. This lets the server do what ``HoldPadding`` does on the
    client: leave that module out of the pipeline for this to apply, because a padded
    chunk ends in a hold that leaves no velocity to keep.

    Meant for short chunks streamed at the policy rate, down to one waypoint.
    About 200 ms rode through 100-150 ms network stalls in teleoperation. At the
    end of a run, :meth:`WaypointJoggingSession.drain` sends the last target once
    more without the delay, so the robot stops on it rather than running past it.

    Needs NOVA 26.8.0-112 or newer; older servers ignore the field.
    """

    def __post_init__(self) -> None:
        if self.max_expected_latency_ms <= 0:
            raise ValueError("max_expected_latency_ms must be positive")
        if self.delay_until_braking_ms is not None and self.delay_until_braking_ms < 0:
            raise ValueError("delay_until_braking_ms must be non-negative")
        if not math.isfinite(self.single_step_dt_ms) or self.single_step_dt_ms <= 0:
            raise ValueError("single_step_dt_ms must be a positive finite value")


@dataclass(slots=True)
class StopContext:
    """State passed to stop-condition callbacks.

    Stop conditions run on every jogging tick. Return ``True`` to stop the
    policy. They must be fast (microseconds) — no network calls or blocking I/O.
    Use ``Observation.computed()`` for async data, then read it here.
    """

    state: RobotState
    prev_state: RobotState | None
    dt: float
    motion_group_id: str
    io_values: dict[str, object] | None = None
    """Latest IO values from the stream cache (if available)."""

    target_joints: list[list[float]] | None = None
    """Intended joint/TCP targets the policy wants to execute.
    Full chunk at inference time, single interpolated step during jogging ticks."""

    target_ios: dict[str, bool | int | float | str] | None = None
    """Intended IO writes (populated at inference time, before IOs fire)."""


StopCondition = Callable[[StopContext], bool]
"""A fast, synchronous check run on every tick. Return ``True`` to stop the policy."""


class EmergencyStopError(Exception):
    """Raised when the robot controller enters a non-operational safety state.

    Covers e-stop, protective stop, cell door, safety violation, fault, etc.
    The Nova SDK does not have an equivalent — it handles e-stop at the
    controller level, not as a Python exception.
    """

    def __init__(self, controller_id: str, safety_state: str = "") -> None:
        self.controller_id = controller_id
        self.safety_state = safety_state
        msg = f"Safety stop on controller '{controller_id}'"
        if safety_state:
            msg += f" (state: {safety_state})"
        super().__init__(msg)


class MotionError(Exception):
    """Raised when jogging detects a motion-blocking condition.

    This happens when the waypoint jogging session detects that the robot is
    paused (joint limit, self-collision, singularity). Distinct from Nova SDK's
    ``RobotMotionError`` which covers trajectory planning/execution failures.
    """

    def __init__(self, motion_group_id: str, message: str) -> None:
        self.motion_group_id = motion_group_id
        super().__init__(f"Motion error on '{motion_group_id}': {message}")


class JoggingNotSupportedError(Exception):
    """Raised when the NOVA instance does not expose action chunk streaming.

    The action chunk streaming websocket endpoint (``executeActionChunks``) only
    exists on api-gateway ``>= 26.6``; on ``26.5`` the same feature was reached
    through ``executeWaypointJogging``, which 26.6 removed. Gateways without the
    endpoint reject the upgrade with HTTP 404; we surface that as this actionable
    error instead of a generic connection failure, so callers know to upgrade
    NOVA rather than chase a network problem.
    """

    def __init__(self, motion_group_id: str) -> None:
        self.motion_group_id = motion_group_id
        super().__init__(
            f"Action chunk streaming is not available for '{motion_group_id}' on this "
            "NOVA instance (the executeActionChunks endpoint returned HTTP 404). "
            "It requires api-gateway >= 26.6."
        )
