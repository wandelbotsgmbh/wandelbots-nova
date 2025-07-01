"""
Viewer implementations for Nova programs.

This module provides different viewer backends that can be used
to visualize and monitor Nova programs during execution.
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
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


class Rerun(Viewer):
    """
    Rerun viewer for 3D visualization of robot motion and program execution.

    This viewer automatically captures and visualizes:
    - Robot trajectories and motion paths
    - TCP poses and transformations
    - Motion group states
    - Planning requests and responses
    - Collision scenes and safety zones (optional)
    - Real-time robot state streaming (optional)

    Example usage:
        @nova.program(
            name="My Program",
            viewer=nova.viewers.Rerun(
                show_safety_zones=True,
                show_collision_scenes=True,
                enable_streaming=True
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
        enable_streaming: bool = False,
        collision_scene_ids: Optional[list[str]] = None,
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
        self.enable_streaming = enable_streaming
        self.collision_scene_ids = collision_scene_ids
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
            await self._bridge.setup_blueprint()
            await self._setup_collision_scenes()
            await self._setup_safety_zones()
            await self._setup_streaming()

    async def _setup_collision_scenes(self) -> None:
        """Setup collision scene visualization."""
        if not self.show_collision_scenes or not self._bridge:
            return

        try:
            if self.collision_scene_ids:
                # Log specific collision scenes
                for scene_id in self.collision_scene_ids:
                    await self._bridge.log_collision_scene(scene_id)
            else:
                # Log all collision scenes
                await self._bridge.log_collision_scenes()
        except Exception as e:
            # Don't fail the entire setup if collision scenes can't be loaded
            print(f"Warning: Could not load collision scenes: {e}")

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

    async def _setup_streaming(self) -> None:
        """Setup real-time robot state streaming."""
        if not self.enable_streaming or not self._bridge:
            return

        try:
            # Start streaming for all motion groups
            cell = self._bridge.nova.cell()
            controllers = await cell.controllers()

            for controller in controllers:
                motion_groups = await controller.activated_motion_groups()
                for motion_group in motion_groups:
                    await self._bridge.start_streaming(motion_group)
        except Exception as e:
            # Don't fail the entire setup if streaming can't be started
            print(f"Warning: Could not start streaming: {e}")

    def get_bridge(self) -> Optional[object]:
        """Get the underlying NovaRerunBridge instance.

        This allows advanced users to access the full bridge functionality.

        Returns:
            The NovaRerunBridge instance if configured, None otherwise.
        """
        return self._bridge

    def cleanup(self) -> None:
        """Clean up rerun integration after program execution."""
        if self._bridge:
            # Stop any streaming tasks
            import asyncio

            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    loop.create_task(self._bridge.stop_streaming())
            except RuntimeError:
                # Event loop not running, can't stop streaming
                pass

        # Cleanup is handled by the NovaRerunBridge context manager
        # when the Nova instance is closed
        self._bridge = None
        self._configured = False


# For convenience, allow direct import of Rerun
__all__ = [
    "Viewer",
    "Rerun",
    "_configure_active_viewers",
    "_setup_active_viewers_after_preconditions",
    "_cleanup_active_viewers",
]
