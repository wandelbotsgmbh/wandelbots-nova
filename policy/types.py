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
class MotionConfig:
    """Configuration for robot motion execution via NOVA Jogging API.

    Uses a trapezoidal velocity profile: computes velocities from position
    differences between chunk steps, applies a ramp envelope, and advances
    based on the robot's actual position. Guarantees zero overshoot.
    """

    n_action_steps: int = 0
    """Number of steps from each action chunk to actually execute.
    0 = execute all steps. When set (e.g. 8), only the first N steps
    are sent to the controller — later steps have higher prediction
    uncertainty and are discarded (receding horizon)."""

    execute_and_wait: bool = True
    """Whether to wait for the current chunk to finish before querying
    new inference (receding horizon).

    When True (default): execute n_action_steps, wait until done, then
    query fresh inference. Standard approach for GR00T/LeRobot.

    When False: query inference continuously at inference_hz. Each new
    chunk replaces the old one, starting from the step closest to the
    robot's current position."""

    velocity_limit: float | list[float] = 2.0
    """Maximum joint velocity in rad/s. Scalar or per-axis list."""

    ramp_steps: int = 3
    """Number of steps for the trapezoidal ramp-up/ramp-down envelope.
    Higher = smoother starts/stops, lower = more responsive."""

    p_gain: float = 3.0
    """P-gain for single-position targets (teleop mode).
    Only used when chunk has 1 step (dt_ms=0)."""

    state_rate_ms: int = 10
    """State stream update rate."""




@dataclass(slots=True)
class GuardState:
    """State passed to safety guard callbacks.

    Guards run at ~100Hz during PID jogging. Return ``False`` to stop immediately.
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
    Full chunk at inference time, single interpolated step during PID ticks."""

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

    This happens when the PID jogging session detects that the robot is paused
    (joint limit, self-collision, singularity). Distinct from Nova SDK's
    ``RobotMotionError`` which covers trajectory planning/execution failures.
    """

    def __init__(self, motion_group_id: str, message: str) -> None:
        self.motion_group_id = motion_group_id
        super().__init__(f"Motion error on '{motion_group_id}': {message}")
