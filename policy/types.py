"""Data types for the policy package."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

import pydantic

from nova.types import Pose

if TYPE_CHECKING:
    from nova.types import RobotState

# Reuse the SDK's IO value type
ValueType = int | str | bool | float | Pose


class PolicyResponse(pydantic.BaseModel):
    """Serializable response from a policy server (wire format).

    This is what a policy service returns over ZMQ.
    The policy always returns actions — it never signals "done".

    Examples:
        # Action targets:
        PolicyResponse(joints={"0@ur10e": [[0.1, -1.5, ...]]})

        # Action targets with IO:
        PolicyResponse(
            joints={"0@ur10e": [[0.1, -1.5, ...]]},
            ios={"0@ur10e": {"digital_out[0]": True}},
        )

        # Multi-step chunk (ACT/Diffusion Policy output):
        PolicyResponse(
            joints={"0@ur10e": [[step0], [step1], ..., [step15]]},
            dt_ms=33.0,
        )
    """

    joints: dict[str, list[list[float]]] | None = None
    """Motion group id → sequence of joint targets (radians)."""

    ios: dict[str, dict[str, bool | int | float | str]] | None = None
    """Motion group id → {io_key: value}."""

    dt_ms: float = 0.0
    """Time spacing between steps in milliseconds."""


class ActionChunk(pydantic.BaseModel):
    """Action chunk sent to the PID runner.

    Single-step (teleoperation at 30 Hz):
        ActionChunk(joints={"0@ur5e": [[0.1, -1.5, ...]]})

    Multi-step (policy inference, e.g. ACT outputs 16 steps at 33ms):
        ActionChunk(
            joints={"0@ur5e": [[...], [...], [...], ...]},
            dt_ms=33.0,
        )
    """

    joints: dict[str, list[list[float]]]
    """Motion group id → sequence of joint targets (radians)."""

    ios: dict[str, dict[str, bool | int | float | str]] | None = None
    """Motion group id → {io_key: value}. Fired once on send()."""

    dt_ms: float = 0.0
    """Time spacing between steps in milliseconds. 0 = single-step."""

    timestamp: float | None = None
    """When this chunk was produced (seconds since epoch). Auto-filled if None."""

    def model_post_init(self, _context: object) -> None:
        if self.timestamp is None:
            self.timestamp = time.time()

    @classmethod
    def from_response(cls, resp: PolicyResponse) -> ActionChunk:
        """Create from a PolicyResponse (assumes it has joints)."""
        if resp.joints is None:
            msg = "PolicyResponse has no joints"
            raise ValueError(msg)
        return cls(joints=resp.joints, ios=resp.ios, dt_ms=resp.dt_ms)

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> ActionChunk:
        """Construct from a plain dictionary (e.g. from JSON message)."""
        resp = PolicyResponse.model_validate(data)
        if resp.joints is None:
            msg = "Response has no joints"
            raise ValueError(msg)
        return cls(joints=resp.joints, ios=resp.ios, dt_ms=resp.dt_ms)


@dataclass
class PolicyRunnerConfig:
    """Configuration for the PolicyRunner PID velocity controller."""

    velocity_limit: float | list[float] = 1.5
    """Joint velocity limit in rad/s. Scalar (all joints) or per-joint list.
    Only used for joint jogging. Matches ``joint_velocity_limits`` in NOVA API."""

    tcp_velocity_limit: float = 250.0
    """TCP translation velocity limit in mm/s. Only used for TCP jogging."""

    tcp_orientation_velocity_limit: float = 1.5
    """TCP rotation velocity limit in rad/s. Only used for TCP jogging."""
    tolerance: float = 0.01
    p_gain: float = 3.0
    i_gain: float = 0.0
    d_gain: float = 0.1
    ff_gain: float = 0.0
    integral_limit: float = 2.0
    state_rate_ms: int = 10


@dataclass
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
    """Raised when a safety guard returns False."""

    def __init__(self, motion_group_id: str, guard_name: str) -> None:
        self.motion_group_id = motion_group_id
        self.guard_name = guard_name
        super().__init__(
            f"Safety guard '{guard_name}' triggered stop for motion group '{motion_group_id}'"
        )


class EmergencyStopError(Exception):
    """Raised when the robot controller enters a non-operational safety state.

    This covers all safety stops: emergency stop, protective stop (cell door opened),
    safety violation, fault, etc. — any state where the robot cannot be moved.
    """

    def __init__(self, controller_id: str, safety_state: str = "") -> None:
        self.controller_id = controller_id
        self.safety_state = safety_state
        msg = f"Safety stop on controller '{controller_id}'"
        if safety_state:
            msg += f" (state: {safety_state})"
        super().__init__(msg)


class MotionError(Exception):
    """Raised when the jogging API reports a motion error.

    This happens when the commanded velocity would move the robot into
    a joint limit or cause a self-collision.
    """

    def __init__(self, motion_group_id: str, message: str) -> None:
        self.motion_group_id = motion_group_id
        super().__init__(f"Motion error on '{motion_group_id}': {message}")
