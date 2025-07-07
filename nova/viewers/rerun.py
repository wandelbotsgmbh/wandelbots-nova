"""Rerun viewer implementation for 3D visualization."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional, Sequence, cast

if TYPE_CHECKING:
    from nova.actions import Action
    from nova.api import models
    from nova.core.motion_group import MotionGroup
    from nova.core.nova import Nova

from .base import Viewer
from .manager import register_viewer
from .protocol import NovaRerunBridgeProtocol
from .utils import extract_collision_scenes_from_actions

logger = logging.getLogger(__name__)


class Rerun(Viewer):
    """
    Rerun viewer for 3D visualization of robot motion and program execution.

    This viewer automatically captures and visualizes:
    - Robot trajectories and motion paths
    - TCP poses and transformations
    - Motion group states
    - Planning requests and responses
    - Collision scenes and safety zones (optional)
    - Tool geometries attached to specific TCPs

    Example usage:
        # 3D view only (default)
        @nova.program(
            viewer=nova.viewers.Rerun(
                tcp_tools={"vacuum": "assets/vacuum_cup.stl"}
            )
        )

        # Full interface with detailed analysis panels
        @nova.program(
            viewer=nova.viewers.Rerun(
                show_details=True,
                show_safety_zones=True,
                show_collision_link_chain=True,
                show_safety_link_chain=True,
                tcp_tools={
                    "vacuum": "assets/vacuum_cup.stl",
                    "gripper": "assets/parallel_gripper.stl"
                }
            )
        )
    """

    def __init__(
        self,
        application_id: Optional[str] = None,
        spawn: bool = True,
        show_safety_zones: bool = True,
        show_collision_scenes: bool = True,
        show_collision_link_chain: bool = False,
        show_safety_link_chain: bool = True,
        tcp_tools: Optional[dict[str, str]] = None,
        show_details: bool = False,
    ) -> None:
        """
        Initialize the Rerun viewer.

        Args:
            application_id: Optional application ID for the rerun recording
            spawn: Whether to spawn a rerun viewer process automatically
            show_safety_zones: Whether to visualize safety zones for motion groups
            show_collision_scenes: Whether to show collision scenes
            show_collision_link_chain: Whether to show robot collision mesh geometry
            show_safety_link_chain: Whether to show robot safety geometry (from controller)
            tcp_tools: Optional mapping of TCP IDs to tool asset file paths
            show_details: Whether to show detailed analysis panels with charts and logs (False = 3D view only)
        """
        self.application_id: Optional[str] = application_id
        self.spawn: bool = spawn
        self.show_safety_zones: bool = show_safety_zones
        self.show_collision_scenes: bool = show_collision_scenes
        self.show_collision_link_chain: bool = show_collision_link_chain
        self.show_safety_link_chain: bool = show_safety_link_chain
        self.tcp_tools: dict[str, str] = tcp_tools or {}
        self.show_details: bool = show_details
        self._bridge: Optional[NovaRerunBridgeProtocol] = None
        self._logged_safety_zones: set[str] = (
            set()
        )  # Track motion groups that already have safety zones logged

        # Register this viewer as active
        register_viewer(self)

    def configure(self, nova: Nova) -> None:
        """Configure rerun integration for program execution."""
        if self._bridge is not None:
            return  # Already configured

        try:
            from nova_rerun_bridge import NovaRerunBridge

            bridge = NovaRerunBridge(
                nova=nova,
                spawn=self.spawn,
                recording_id=self.application_id,
                show_details=self.show_details,
                show_collision_link_chain=self.show_collision_link_chain,
                show_safety_link_chain=self.show_safety_link_chain,
            )
            self._bridge = cast(NovaRerunBridgeProtocol, bridge)
            # Don't setup async components immediately - wait for controllers to be ready
        except ImportError:
            # nova_rerun_bridge not available, skip rerun integration
            logger.warning(
                "Rerun viewer configured but nova_rerun_bridge not available. "
                "Install with: uv add wandelbots-nova --extra nova-rerun-bridge"
            )

    async def setup_after_preconditions(self) -> None:
        """Setup async components after preconditions (like controllers) are satisfied."""
        if self._bridge and not hasattr(self, "_async_setup_done"):
            await self._setup_async_components()
            self._async_setup_done = True

    async def _setup_async_components(self) -> None:
        """Setup async components like blueprint."""
        if self._bridge:
            # Initialize the bridge's own Nova client before using it
            await self._bridge.__aenter__()

            # Setup blueprint (show_details is already configured in bridge)
            await self._bridge.setup_blueprint()

    async def _ensure_safety_zones_logged(self, motion_group: MotionGroup) -> None:
        """Ensure safety zones are logged for the given motion group.

        This method is called during planning to ensure safety zones are shown
        only for motion groups that are actually being used.

        Args:
            motion_group: The motion group to log safety zones for
        """
        if not self.show_safety_zones or not self._bridge:
            return

        # Use the motion group ID as unique identifier
        motion_group_id = motion_group.motion_group_id

        if motion_group_id not in self._logged_safety_zones:
            try:
                await self._bridge.log_safety_zones(motion_group)
                self._logged_safety_zones.add(motion_group_id)
            except Exception as e:
                logger.warning(
                    "Could not log safety zones for motion group %s: %s", motion_group_id, e
                )

    async def _log_planning_results(
        self,
        actions: Sequence[Action],
        trajectory: models.JointTrajectory,
        tcp: str,
        motion_group: MotionGroup,
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

            # Log trajectory with tool asset if configured for this TCP
            tool_asset = self._resolve_tool_asset(tcp)
            await self._bridge.log_trajectory(trajectory, tcp, motion_group, tool_asset=tool_asset)

            # Log collision scenes from actions if configured
            if self.show_collision_scenes:
                collision_scenes = extract_collision_scenes_from_actions(actions)
                if collision_scenes:
                    # Log collision scenes using the sync method
                    self._bridge._log_collision_scene(collision_scenes)

        except Exception as e:
            logger.warning("Failed to log planning results in Rerun viewer: %s", e)

    async def log_planning_success(
        self,
        actions: Sequence[Action],
        trajectory: models.JointTrajectory,
        tcp: str,
        motion_group: MotionGroup,
    ) -> None:
        """Log successful planning results to Rerun viewer.

        Args:
            actions: List of actions that were planned
            trajectory: The resulting trajectory
            tcp: TCP used for planning
            motion_group: The motion group used for planning
        """
        # Ensure safety zones are logged for this motion group (only on first use)
        await self._ensure_safety_zones_logged(motion_group)

        # Log the planning results
        await self._log_planning_results(actions, trajectory, tcp, motion_group)

    async def log_planning_failure(
        self, actions: Sequence[Action], error: Exception, tcp: str, motion_group: MotionGroup
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

        # Ensure safety zones are logged for this motion group (only on first use)
        await self._ensure_safety_zones_logged(motion_group)

        try:
            # Log the failed actions
            await self._bridge.log_actions(list(actions), motion_group=motion_group)

            # Handle specific PlanTrajectoryFailed errors which have additional data
            from nova.core.exceptions import PlanTrajectoryFailed

            if isinstance(error, PlanTrajectoryFailed):
                # Log the trajectory from the failed plan
                if hasattr(error.error, "joint_trajectory") and error.error.joint_trajectory:
                    await self._bridge.log_trajectory(
                        error.error.joint_trajectory, tcp, motion_group
                    )

                # Log error feedback if available
                if hasattr(error.error, "error_feedback") and error.error.error_feedback:
                    await self._bridge.log_error_feedback(error.error.error_feedback)

            # Log error information as text
            import rerun as rr

            error_message = f"Planning failed: {type(error).__name__}: {str(error)}"
            rr.log("planning/errors", rr.TextLog(error_message, level=rr.TextLogLevel.ERROR))

            # Log collision scenes from actions if configured (they might be relevant to the failure)
            if self.show_collision_scenes:
                collision_scenes = extract_collision_scenes_from_actions(actions)
                if collision_scenes:
                    # Log collision scenes using the sync method
                    self._bridge._log_collision_scene(collision_scenes)

        except Exception as e:
            logger.warning("Failed to log planning failure in Rerun viewer: %s", e)

    def get_bridge(self) -> Optional[NovaRerunBridgeProtocol]:
        """Get the underlying NovaRerunBridge instance.

        This allows advanced users to access the full bridge functionality.

        Returns:
            The NovaRerunBridge instance if configured, None otherwise.
        """
        return self._bridge

    def cleanup(self) -> None:
        """Clean up rerun integration after program execution."""
        self._bridge = None
        self._logged_safety_zones.clear()  # Reset safety zone tracking

    def _resolve_tool_asset(self, tcp: str) -> Optional[str]:
        """Resolve the tool asset file path for a given TCP.

        Args:
            tcp: The TCP ID to resolve tool asset for

        Returns:
            Path to tool asset file if configured, None otherwise
        """
        return self.tcp_tools.get(tcp)
