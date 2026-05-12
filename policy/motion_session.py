"""MotionSession protocol — shared interface for PID jogging and trajectory sessions."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from policy.types import ValueType


@runtime_checkable
class MotionSession(Protocol):
    """Protocol for motion sessions used by the PolicyExecutor.

    Two implementations:
    - ``PidJoggingSession``: PID velocity control via the NOVA Jogging API
    - ``TrajectorySession``: Planned joint_ptp trajectories via the NOVA Trajectory API
    """

    @property
    def motion_group(self) -> MotionGroup: ...

    @property
    def motion_group_id(self) -> str: ...

    @property
    def current_state(self) -> RobotState | None: ...

    @property
    def is_running(self) -> bool: ...

    @property
    def has_failed(self) -> bool: ...

    @property
    def failure_reason(self) -> str: ...

    @property
    def failure_exception(self) -> BaseException | None: ...

    def set_io_values_ref(self, values: dict[str, object]) -> None: ...

    def update_chunk(self, steps: list[list[float]], dt_ms: float) -> None: ...

    async def write_ios(self, ios: dict[str, ValueType]) -> None: ...

    async def start(self) -> None: ...

    async def stop(self) -> None: ...
