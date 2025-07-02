"""
Viewer implementations for Nova programs.

This module provides different viewer backends that can be used
to visualize and monitor Nova programs during execution.
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional, Sequence

if TYPE_CHECKING:
    from nova.actions import Action
    from nova.api import models
    from nova.core.motion_group import MotionGroup
    from nova.core.nova import Nova

# Global registry of active viewers
_active_viewers: list["Viewer"] = []


def _register_viewer(viewer: "Viewer") -> None:
    """Register a viewer as active."""
    global _active_viewers
    if viewer not in _active_viewers:
        _active_viewers.append(viewer)


def _configure_active_viewers(nova: "Nova") -> None:
    """Configure all active viewers with the Nova instance."""
    global _active_viewers
    for viewer in _active_viewers:
        viewer.configure(nova)


async def _setup_active_viewers_after_preconditions() -> None:
    """Setup all active viewers after preconditions are satisfied."""
    global _active_viewers
    for viewer in _active_viewers:
        await viewer.setup_after_preconditions()


def _cleanup_active_viewers() -> None:
    """Clean up all active viewers."""
    global _active_viewers
    for viewer in _active_viewers:
        viewer.cleanup()
    _active_viewers.clear()


def _extract_collision_scenes_from_actions(
    actions: Sequence["Action"],
) -> dict[str, "models.CollisionScene"]:
    """Extract unique collision scenes from a list of actions.

    Args:
        actions: List of actions to extract collision scenes from

    Returns:
        Dictionary mapping collision scene IDs to CollisionScene objects
    """
    from nova.actions.motions import CollisionFreeMotion, Motion

    collision_scenes: dict[str, "models.CollisionScene"] = {}

    for action in actions:
        # Check if action is a motion with collision_scene attribute
        if isinstance(action, (Motion, CollisionFreeMotion)) and action.collision_scene is not None:
            # Generate a unique ID for the collision scene
            # Using hash of the collision scene content for uniqueness
            scene_id = f"action_scene_{hash(str(action.collision_scene))}"
            collision_scenes[scene_id] = action.collision_scene

    return collision_scenes


async def _log_planning_results_to_viewers(
    actions: Sequence["Action"],
    trajectory: "models.JointTrajectory",
    tcp: str,
    motion_group: "MotionGroup",
) -> None:
    """Log successful planning results to all active viewers.

    Args:
        actions: List of actions that were planned
        trajectory: The resulting trajectory
        tcp: TCP used for planning
        motion_group: The motion group used for planning
    """
    global _active_viewers

    if not _active_viewers:
        return

    for viewer in _active_viewers:
        try:
            await viewer.log_planning_success(actions, trajectory, tcp, motion_group)
        except Exception as e:
            # Don't fail planning if logging fails
            print(f"Warning: Failed to log planning results to viewer: {e}")


async def _log_planning_error_to_viewers(
    actions: Sequence["Action"], error: Exception, tcp: str, motion_group: "MotionGroup"
) -> None:
    """Log planning failure to all active viewers.

    Args:
        actions: List of actions that failed to plan
        error: The planning error that occurred
        tcp: TCP used for planning
        motion_group: The motion group used for planning
    """
    global _active_viewers

    if not _active_viewers:
        return

    for viewer in _active_viewers:
        try:
            await viewer.log_planning_failure(actions, error, tcp, motion_group)
        except Exception as e:
            # Don't fail planning if logging fails
            print(f"Warning: Failed to log planning error to viewer: {e}")


class Viewer(ABC):
    """Abstract base class for Nova program viewers."""

    @abstractmethod
    def configure(self, nova: "Nova") -> None:
        """Configure the viewer for program execution."""
        pass

    async def setup_after_preconditions(self) -> None:
        """Setup viewer components after preconditions are satisfied.

        Override this method in subclasses that need to wait for preconditions
        like active controllers before setting up visualization components.
        """
        pass

    @abstractmethod
    def cleanup(self) -> None:
        """Clean up the viewer after program execution."""
        pass

    async def log_planning_success(
        self,
        actions: Sequence["Action"],
        trajectory: "models.JointTrajectory",
        tcp: str,
        motion_group: "MotionGroup",
    ) -> None:
        """Log successful planning results.

        Args:
            actions: List of actions that were planned
            trajectory: The resulting trajectory
            tcp: TCP used for planning
            motion_group: The motion group used for planning
        """
        pass

    async def log_planning_failure(
        self, actions: Sequence["Action"], error: Exception, tcp: str, motion_group: "MotionGroup"
    ) -> None:
        """Log planning failure results.

        Args:
            actions: List of actions that failed to plan
            error: The planning error that occurred
            tcp: TCP used for planning
            motion_group: The motion group used for planning
        """
        pass


class Rerun(Viewer):
    """
    Rerun viewer for 3D visualization of robot motion and program execution.

    This viewer automatically captures and visualizes:
    - Robot trajectories and motion paths
    - TCP poses and transformations
    - Motion group states
    - Planning requests and responses
    - Collision scenes and safety zones (optional)

    Example usage:
        @nova.program(
            name="My Program",
            viewer=nova.viewers.Rerun(
                show_safety_zones=True,
                show_collision_scenes=True,
            ),
            preconditions=...
        )
        async def my_program():
            # Your program code here
            pass
    """

    def __init__(
        self,
        application_id: Optional[str] = None,
        spawn: bool = True,
        show_safety_zones: bool = True,
        show_collision_scenes: bool = True,
    ):
        """
        Initialize the Rerun viewer.

        Args:
            application_id: Optional application ID for the rerun recording
            spawn: Whether to spawn a rerun viewer process automatically
            show_safety_zones: Whether to visualize safety zones for motion groups
            show_collision_scenes: Whether to show collision scenes
            enable_streaming: Whether to enable real-time robot state streaming
            collision_scene_ids: Specific collision scene IDs to show (if None, shows all)
        """
        self.application_id = application_id
        self.spawn = spawn
        self.show_safety_zones = show_safety_zones
        self.show_collision_scenes = show_collision_scenes
        self._bridge = None
        self._configured = False

        # Register this viewer as active
        _register_viewer(self)

    def configure(self, nova: "Nova") -> None:
        """Configure rerun integration for program execution."""
        if self._configured:
            return  # Already configured

        try:
            from nova_rerun_bridge import NovaRerunBridge

            self._bridge = NovaRerunBridge(
                nova=nova, spawn=self.spawn, recording_id=self.application_id
            )
            self._configured = True
            # Don't setup async components immediately - wait for controllers to be ready
        except ImportError:
            # nova_rerun_bridge not available, skip rerun integration
            pass

    async def setup_after_preconditions(self) -> None:
        """Setup async components after preconditions (like controllers) are satisfied."""
        if self._bridge and not getattr(self, "_async_setup_done", False):
            await self._setup_async_components()
            self._async_setup_done = True

    async def _setup_async_components(self) -> None:
        """Setup async components like blueprint."""
        if self._bridge:
            # Initialize the bridge's own Nova client before using it
            await self._bridge.__aenter__()

            await self._bridge.setup_blueprint()
            await self._setup_collision_scenes()
            await self._setup_safety_zones()

    async def _setup_collision_scenes(self) -> None:
        """Setup collision scene visualization."""
        if not self.show_collision_scenes or not self._bridge:
            return

    async def _setup_safety_zones(self) -> None:
        """Setup safety zone visualization."""
        if not self.show_safety_zones or not self._bridge:
            return

        try:
            # Get all motion groups and show safety zones for each
            cell = self._bridge.nova.cell()
            controllers = await cell.controllers()

            for controller in controllers:
                motion_groups = await controller.activated_motion_groups()
                for motion_group in motion_groups:
                    await self._bridge.log_saftey_zones(motion_group)
        except Exception as e:
            # Don't fail the entire setup if safety zones can't be loaded
            print(f"Warning: Could not load safety zones: {e}")

    async def _log_planning_results(
        self,
        actions: Sequence["Action"],
        trajectory: "models.JointTrajectory",
        tcp: str,
        motion_group: "MotionGroup",
    ) -> None:
        """Log planning results including actions, trajectory, and collision scenes.

        Args:
            actions: List of actions that were planned
            trajectory: The resulting trajectory
            tcp: TCP used for planning
            motion_group: The motion group used for planning
        """
        if not self._bridge:
            return

        try:
            # Log actions
            await self._bridge.log_actions(list(actions), motion_group=motion_group)

            # Log trajectory
            await self._bridge.log_trajectory(trajectory, tcp, motion_group)

            # Log collision scenes from actions if configured
            if self.show_collision_scenes:
                collision_scenes = _extract_collision_scenes_from_actions(actions)
                if collision_scenes:
                    # Log collision scenes using the sync method
                    self._bridge._log_collision_scene(collision_scenes)

        except Exception as e:
            print(f"Warning: Failed to log planning results in Rerun viewer: {e}")

    def get_bridge(self) -> Optional[object]:
        """Get the underlying NovaRerunBridge instance.

        This allows advanced users to access the full bridge functionality.

        Returns:
            The NovaRerunBridge instance if configured, None otherwise.
        """
        return self._bridge

    async def log_planning_success(
        self,
        actions: Sequence["Action"],
        trajectory: "models.JointTrajectory",
        tcp: str,
        motion_group: "MotionGroup",
    ) -> None:
        """Log successful planning results to Rerun viewer.

        Args:
            actions: List of actions that were planned
            trajectory: The resulting trajectory
            tcp: TCP used for planning
            motion_group: The motion group used for planning
        """
        await self._log_planning_results(actions, trajectory, tcp, motion_group)

    async def log_planning_failure(
        self, actions: Sequence["Action"], error: Exception, tcp: str, motion_group: "MotionGroup"
    ) -> None:
        """Log planning failure to Rerun viewer.

        Args:
            actions: List of actions that failed to plan
            error: The planning error that occurred
            tcp: TCP used for planning
            motion_group: The motion group used for planning
        """
        if not self._bridge:
            return

        try:
            # Log the failed actions
            await self._bridge.log_actions(list(actions), motion_group=motion_group)

            # Log error information as text
            import rerun as rr

            error_message = f"Planning failed: {type(error).__name__}: {str(error)}"
            rr.log("planning/errors", rr.TextLog(error_message, level=rr.TextLogLevel.ERROR))

            # Log collision scenes from actions if configured (they might be relevant to the failure)
            if self.show_collision_scenes:
                collision_scenes = _extract_collision_scenes_from_actions(actions)
                if collision_scenes:
                    # Log collision scenes using the sync method
                    self._bridge._log_collision_scene(collision_scenes)

        except Exception as e:
            print(f"Warning: Failed to log planning failure in Rerun viewer: {e}")

    def cleanup(self) -> None:
        """Clean up rerun integration after program execution."""
        self._bridge = None
        self._configured = False


# For convenience, allow direct import of Rerun
__all__ = [
    "Viewer",
    "Rerun",
    "_configure_active_viewers",
    "_setup_active_viewers_after_preconditions",
    "_cleanup_active_viewers",
    "_log_planning_results_to_viewers",
    "_log_planning_error_to_viewers",
    "_extract_collision_scenes_from_actions",
]
