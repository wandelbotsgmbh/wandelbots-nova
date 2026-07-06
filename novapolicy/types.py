"""Data types for the policy package."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

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
    """Motion group id → {io_key: value}. Fired once on send()."""

    dt_ms: float = 0.0
    """Time spacing between steps in milliseconds. 0 = single-step."""

    first_timestamp_ms: int = -1
    """Where this chunk's first waypoint sits on the server's session timeline.

    Two placement models, selected by this one field:

    * **Absolute** (``>= 0``): timestamps are pinned to the server timeline as
      ``[first_timestamp_ms, first_timestamp_ms + dt, first_timestamp_ms + 2*dt, ...]`` — the
      first waypoint lands exactly at the anchor (which may be in the *past*).
      The *caller* decides
      the anchor. Required for overlapping chunks (RTC), where a specific
      interior step must land at "now" so consecutive chunks align and the
      server can replace waypoints older than the new anchor. Also used when the
      caller already knows the absolute time (standalone jogging, replay).
    * **Relative** (``-1``, default): the session ignores any caller anchor and
      places waypoints at ``[now + dt, now + 2*dt, ...]`` using the server clock
      read at *send time*. The simplest correct placement for sequential,
      non-overlapping jogging: there is no timeline to align to, and deriving
      "now" at the last instant avoids queue staleness.

    Rule of thumb: overlapping/RTC ⇒ absolute; plain sequential ⇒ relative."""

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


@dataclass(slots=True)
class WaypointConfig:
    """Configuration for NOVA waypoint jogging.

    Sends timestamped joint or TCP pose waypoints directly to the server,
    which handles velocity profiling, interpolation, and IK internally.

    New chunks override previous ones, so the server always tracks the
    freshest prediction.
    """

    state_rate_ms: int = 10
    """State stream update rate."""


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
    """Raised when the NOVA instance does not expose action-chunk streaming.

    The action-chunk-streaming websocket endpoint (``executeActionChunks``) only
    exists on api-gateway ``>= 26.6``. Older gateways reject the upgrade with
    HTTP 404; we surface that as this actionable error instead of a generic
    connection failure, so callers know to upgrade NOVA rather than chase a
    network problem.
    """

    def __init__(self, motion_group_id: str) -> None:
        self.motion_group_id = motion_group_id
        super().__init__(
            f"Waypoint jogging is not available for '{motion_group_id}' on this "
            "NOVA instance (the executeActionChunks endpoint returned HTTP 404). "
            "It requires api-gateway >= 26.6."
        )
