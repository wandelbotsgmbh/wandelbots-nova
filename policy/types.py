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
    """Action chunk sent to the PID runner.

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


@dataclass(slots=True)
class PidConfig:
    """Configuration for the PID velocity controller."""

    velocity_limit: float | list[float] = 1.5
    """Velocity limit in rad/s (joints) or mm/s + rad/s (TCP).
    Scalar applies to all axes; list sets per-axis limits."""

    tolerance: float = 0.01
    p_gain: float = 3.0
    i_gain: float = 0.0
    d_gain: float = 0.1
    ff_gain: float = 0.0
    integral_limit: float = 2.0
    state_rate_ms: int = 10


@dataclass(slots=True)
class GuardState:
    """State passed to the safety guard callback on each jogging tick."""

    state: RobotState
    prev_state: RobotState | None
    dt: float
    motion_group_id: str
    io_values: dict[str, object] | None = None
    """Latest IO values from the stream cache (if available). Keys are IO names."""


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

    This happens when the PID jogging session detects that the robot is paused
    (joint limit, self-collision, singularity). Distinct from Nova SDK's
    ``RobotMotionError`` which covers trajectory planning/execution failures.
    """

    def __init__(self, motion_group_id: str, message: str) -> None:
        self.motion_group_id = motion_group_id
        super().__init__(f"Motion error on '{motion_group_id}': {message}")
