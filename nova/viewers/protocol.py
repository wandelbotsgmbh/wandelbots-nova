"""Protocol definitions for Nova viewer implementations."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Protocol, Union, runtime_checkable

if TYPE_CHECKING:
    from nova import api
    from nova.actions import Action
    from nova.cell.motion_group import MotionGroup
    from nova.core.nova import Nova


@runtime_checkable
class NovaRerunBridgeProtocol(Protocol):
    """Protocol defining the interface for NovaRerunBridge."""

    nova: Nova
    show_safety_link_chain: bool

    async def __aenter__(self) -> NovaRerunBridgeProtocol:
        """Async context manager entry."""
        ...

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> Optional[bool]:
        """Async context manager exit."""
        ...

    async def setup_blueprint(self) -> None:
        """Setup the blueprint."""
        ...

    async def log_safety_zones(self, motion_group: MotionGroup) -> None:
        """Log safety zones for a motion group."""
        ...

    async def log_actions(
        self,
        actions: Union[list[Action], Action],
        show_connection: bool = False,
        show_labels: bool = False,
        motion_group: Optional[MotionGroup] = None,
        tcp: Optional[str] = None,
    ) -> None:
        """Log actions to the viewer."""
        ...

    async def log_trajectory(
        self,
        trajectory: api.models.JointTrajectory,
        tcp: str,
        motion_group: MotionGroup,
        collision_setups: dict[str, api.models.CollisionSetup],
        time_offset: float = 0,
        tool_asset: Optional[str] = None,
    ) -> None:
        """Log trajectory to the viewer."""
        ...

    def log_collision_setups(self, collision_setups: dict[str, api.models.CollisionSetup]) -> None:
        """Log collision scenes to the viewer."""
        ...

    async def visualize_collision_scene(
        self,
        motion_group_description: api.models.MotionGroupDescription,
        collision_setup: api.models.CollisionSetup | None = None,
        joint_positions: list[float] | tuple[float, ...] | None = None,
        tcp_name: str = "Flange",
        base_entity_path: str = "collision_scenes/static",
        show_safety_geometry: bool = True,
        show_collision_geometry: bool = True,
        show_environment: bool = True,
        highlight_collisions: bool = True,
    ) -> None:
        """Visualize a static collision scene."""
        ...

    async def visualize_plan(
        self,
        trajectory: api.models.JointTrajectory,
        motion_group: MotionGroup,
        tcp: str,
        collision_setup: api.models.CollisionSetup | None = None,
        tool_asset: Optional[str] = None,
    ) -> None:
        """Visualize a planned trajectory without executing it."""
        ...

    def log_coordinate_system(self) -> None:
        """Log the coordinate system."""
        ...

    async def log_error_feedback(
        self, error_feedback: api.models.PlanTrajectoryFailedResponse
    ) -> None:
        """Log error feedback to the viewer."""
        ...
