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

    start_time_ms: int = -1
    """Absolute timestamp (ms from session start) for the first waypoint.

    When >=0, the waypoint session uses trajectory-absolute timestamps:
    [start_time_ms + dt, start_time_ms + 2*dt, ...]. This keeps timestamps
    consistent across overlapping chunks and avoids replanning jitter.

    When -1 (default), timestamps are relative to the current time (legacy).
    Use start_time_ms >= 0 for overlapping waypoint jogging."""

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
class GuardState:
    """State passed to safety guard callbacks.

    Guards run on every jogging tick. Return ``False`` to stop immediately.
    Guards must be fast (microseconds) — no network calls or blocking I/O.
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


SafetyGuard = Callable[[GuardState], bool]


class GuardStopError(Exception):
    """Raised when a user-defined safety guard triggers a stop.

    This is specific to the policy executor's safety guard system.
    Not related to Nova SDK's ``RobotMotionError`` which covers trajectory planning.
    """

    def __init__(self, motion_group_id: str, guard_name: str) -> None:
        self.motion_group_id = motion_group_id
        self.guard_name = guard_name
        super().__init__(
            f"Safety guard '{guard_name}' triggered stop for motion group '{motion_group_id}'"
        )


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
