"""Data types for the policy package."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
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
    """Action chunk streamed to a action chunk streaming session, one or more steps per motion group."""

    joints: dict[str, list[list[float]]] = pydantic.Field(default_factory=dict)
    """Motion group id → sequence of joint targets (radians)."""

    tcp: dict[str, list[list[float]]] = pydantic.Field(default_factory=dict)
    """Motion group id → sequence of TCP targets [x, y, z, rx, ry, rz] (mm, rotation vector)."""

    ios: dict[str, dict[str, bool | int | float | str]] | None = None
    """Motion group id → {io_key: value}, in **hardware** units, written once (contract M8).

    Already mapped by the policy client; joint and TCP targets are in the policy's units.
    """

    values: dict[str, list[list[float]]] = pydantic.Field(default_factory=dict)
    """Policy outputs for the keys ``Action.computed(..., keys=[...])`` hooks consume, one row
    per step."""

    dt_ms: float = 0.0
    """Time spacing between steps in milliseconds. 0 = single-step."""

    first_timestamp_ms: int = -1
    """Where this chunk's first waypoint sits, in ms on the session clock (``ServerMs``).

    ``>= 0``: the policy's own timeline, never bridged (contract T2); ``-1`` (default): placed
    by the executor (contract T1)."""

    action_timestep: int = -1
    """Absolute policy-queue timestep of the first action, or ``-1`` if absent; a
    ``TimestepTimeline`` keeps a queue's chunks on one timeline by it."""

    retained_prefix_steps: int = 0
    """Leading steps a queue client already sent; ``Smoothing`` keeps them exact."""

    resolved: bool = False
    """The targets are already absolute and in NOVA units; the executor sends them as they are."""


@dataclass(frozen=True, slots=True)
class SequentialExecution:
    """Execute one complete chunk, reach standstill, then infer again."""


@dataclass(frozen=True, slots=True)
class ContinuousExecution:
    """Continuously replace the active lookahead without per-chunk braking."""

    rate_hz: float | None = None
    """Fixed inference rate, or ``None`` to run as fast as inference allows."""

    def __post_init__(self) -> None:
        if self.rate_hz is not None and (not math.isfinite(self.rate_hz) or self.rate_hz <= 0):
            raise ValueError("rate_hz must be a positive finite value or None")


ExecutionMode: TypeAlias = SequentialExecution | ContinuousExecution


@dataclass(frozen=True, slots=True)
class ActionChunkConfig:
    """Configuration for NOVA's action chunk streaming; a new chunk replaces the queued ones."""

    state_rate_ms: int = 10
    """State stream update rate in ms."""

    single_step_dt_ms: float = 50.0
    """Spacing in ms for a live jog target sent as a single point, and the fallback for
    ``dt_ms=0``. Must be positive."""

    max_expected_latency_ms: float = 100.0
    """How far after its reference a chunk's first waypoint is due, in ms (contract T1).

    Covers the controller's ~67 ms margin plus the network round trip: set it for the
    deployment's worst latency (ADR 0002). The arm follows the policy by this much.
    """

    delay_until_braking_ms: int | None = None
    """How long, in ms, the robot keeps its velocity after a chunk's last waypoint before braking.

    ``None`` brakes at the end of every chunk. Leave ``HoldPadding`` out for it to apply. Needs
    NOVA 26.8.0-112 or newer (docs/jogging.md).
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
    """State passed to stop-condition callbacks, which run on every tick and must not block.

    Use ``Observation.computed()`` for async data, then read it here.
    """

    state: RobotState
    prev_state: RobotState | None
    dt: float
    motion_group_id: str
    io_values: dict[str, object] | None = None
    """Latest IO values from the stream cache (if available)."""

    target_joints: list[list[float]] | None = None
    """Intended joint/TCP targets: the full chunk at inference, one step during ticks."""

    target_ios: dict[str, bool | int | float | str] | None = None
    """Intended IO writes (populated at inference time, before IOs fire)."""


StopCondition = Callable[[StopContext], bool]
"""A fast, synchronous check run on every tick. Return ``True`` to stop the policy."""


class EmergencyStopError(Exception):
    """Raised when the robot controller enters a non-operational safety state.

    E-stop, protective stop, cell door, safety violation, fault and the like.
    """

    def __init__(self, controller_id: str, safety_state: str = "") -> None:
        self.controller_id = controller_id
        self.safety_state = safety_state
        msg = f"Safety stop on controller '{controller_id}'"
        if safety_state:
            msg += f" (state: {safety_state})"
        super().__init__(msg)


class MotionError(Exception):
    """Raised when the session sees the robot paused (joint limit, self-collision, singularity).

    Not the SDK's ``RobotMotionError`` (planning and execution failures).
    """

    def __init__(self, motion_group_id: str, message: str) -> None:
        self.motion_group_id = motion_group_id
        super().__init__(f"Motion error on '{motion_group_id}': {message}")


class JoggingNotSupportedError(Exception):
    """Raised when the NOVA instance does not expose action chunk streaming.

    ``executeActionChunks`` needs api-gateway ``>= 26.6``; older gateways answer HTTP 404.
    """

    def __init__(self, motion_group_id: str) -> None:
        self.motion_group_id = motion_group_id
        super().__init__(
            f"Action chunk streaming is not available for '{motion_group_id}' on this "
            "NOVA instance (the executeActionChunks endpoint returned HTTP 404). "
            "It requires api-gateway >= 26.6."
        )
