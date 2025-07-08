import asyncio
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rerun as rr
from loguru import logger
from wandelbots_api_client.models import (
    FeedbackCollision,
    FeedbackOutOfWorkspace,
    PlanTrajectoryFailedResponseErrorFeedback,
)

from nova import MotionGroup
from nova.actions import Action
from nova.actions.io import WriteAction
from nova.actions.motions import CollisionFreeMotion, Motion
from nova.api import models
from nova.core.nova import Nova
from nova.types.pose import Pose
from nova_rerun_bridge.blueprint import send_blueprint
from nova_rerun_bridge.collision_scene import log_collision_scenes
from nova_rerun_bridge.consts import RECORDING_INTERVAL, TIME_INTERVAL_NAME
from nova_rerun_bridge.helper_scripts.code_server_helpers import get_rerun_address
from nova_rerun_bridge.helper_scripts.download_models import get_project_root
from nova_rerun_bridge.safety_zones import log_safety_zones
from nova_rerun_bridge.stream_state import stream_motion_group
from nova_rerun_bridge.trajectory import TimingMode, log_motion


class NovaRerunBridge:
    """Bridge between Nova and Rerun for visualization.

    This class provides functionality to visualize Nova data in Rerun.
    It handles trajectoy, collision scenes, blueprints and proper cleanup of resources.

    Example:
        ```python
        from nova.core.nova import Nova
        from nova_rerun_bridge import NovaRerunBridge

        async def main():
            nova = Nova()
            async with NovaRerunBridge(nova) as bridge:
                await bridge.setup_blueprint()
                await bridge.log_collision_scenes()

        ```

    Args:
        nova (Nova): Instance of Nova client
        spawn (bool, optional): Whether to spawn Rerun viewer. Defaults to True.
    """

    def __init__(
        self,
        nova: Nova,
        spawn: bool = True,
        recording_id=None,
        show_details: bool = True,
        show_collision_link_chain: bool = False,
        show_safety_link_chain: bool = True,
    ) -> None:
        self._ensure_models_exist()
        # Store the original nova instance for initial setup and to copy connection parameters
        self.nova = nova
        # Create a separate Nova instance for the bridge to use for API calls
        self._bridge_nova = None
        self._streaming_tasks: dict[MotionGroup, asyncio.Task] = {}
        # Track timing per motion group - each motion group has its own timeline
        self._motion_group_timers: dict[str, float] = {}
        self.show_details = show_details
        self.show_collision_link_chain = show_collision_link_chain
        self.show_safety_link_chain = show_safety_link_chain

        recording_id = recording_id or f"nova_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        if "VSCODE_PROXY_URI" in os.environ:
            rr.init(application_id="nova", recording_id=recording_id, spawn=False)
            rr.save("nova.rrd")
            logger.info(f"Install rerun app and open the visual log on {get_rerun_address()}")
        elif spawn:
            rr.init(application_id="nova", recording_id=recording_id, spawn=True)

        logger.add(sink=rr.LoggingHandler("logs/handler"))

    def _ensure_models_exist(self):
        """Ensure robot models are downloaded"""
        models_dir = Path(get_project_root()) / "models"
        if not models_dir.exists() or not list(models_dir.glob("*.glb")):
            print("Models not found, run update_robot_models() or uv run download-models")

    async def setup_blueprint(self) -> None:
        """Configure and send blueprint configuration to Rerun.

        Fetches motion groups from Nova and configures visualization layout.
        """
        nova = self._bridge_nova or self.nova
        cell = nova.cell()

        controllers = await cell.controllers()
        motion_groups = []

        if not controllers:
            logger.warning("No controllers found")
            return

        for controller in controllers:
            for motion_group in await controller.activated_motion_groups():
                motion_groups.append(motion_group.motion_group_id)

        rr.reset_time()
        rr.set_time(TIME_INTERVAL_NAME, duration=0)

        send_blueprint(motion_groups, self.show_details)
        self.log_coordinate_system()

    def log_coordinate_system(self) -> None:
        """Log the coordinate system of the cell."""

        coordinate_origins = np.zeros((3, 3))  # Origin points for x, y, z arrows
        coordinate_vectors = (
            np.array(
                [
                    [1.0, 0.0, 0.0],  # X direction
                    [0.0, 1.0, 0.0],  # Y direction
                    [0.0, 0.0, 1.0],  # Z direction
                ]
            )
            * 200.0
        )  # Scale factor of 200.0 for better visibility

        coordinate_colors = np.array(
            [
                [1.0, 0.125, 0.376, 1.0],  # #ff2060 - Red/Pink for X
                [0.125, 0.875, 0.502, 1.0],  # #20df80 - Green for Y
                [0.125, 0.502, 1.0, 1.0],  # #2080ff - Blue for Z
            ]
        )

        rr.log(
            "coordinate_system_world",
            rr.Arrows3D(
                origins=coordinate_origins,
                vectors=coordinate_vectors,
                colors=coordinate_colors,
                radii=rr.Radius.ui_points([5.0]),
            ),
            static=True,
        )

    async def log_collision_scenes(self) -> dict[str, models.CollisionScene]:
        """Fetch and log all collision scenes from Nova to Rerun."""
        bridge_nova = self._bridge_nova or self.nova
        collision_scenes = (
            await bridge_nova._api_client.store_collision_scenes_api.list_stored_collision_scenes(
                cell=bridge_nova.cell()._cell_id
            )
        )
        log_collision_scenes(collision_scenes)
        return collision_scenes

    async def log_collision_scene(self, scene_id: str) -> dict[str, models.CollisionScene]:
        """Log a specific collision scene by its ID.

        Args:
            scene_id (str): The ID of the collision scene to log

        Raises:
            ValueError: If scene_id is not found in stored collision scenes
        """
        bridge_nova = self._bridge_nova or self.nova
        collision_scenes = (
            await bridge_nova._api_client.store_collision_scenes_api.list_stored_collision_scenes(
                cell=bridge_nova.cell()._cell_id
            )
        )

        if scene_id not in collision_scenes:
            raise ValueError(f"Collision scene with ID {scene_id} not found")

        log_collision_scenes({scene_id: collision_scenes[scene_id]})
        return {scene_id: collision_scenes[scene_id]}

    def _log_collision_scene(self, collision_scenes: dict[str, models.CollisionScene]) -> None:
        log_collision_scenes(collision_scenes=collision_scenes)

    async def log_safety_zones(self, motion_group: MotionGroup) -> None:
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        rr.reset_time()
        rr.set_time(TIME_INTERVAL_NAME, duration=0)

        log_safety_zones(
            motion_group.motion_group_id, await motion_group._get_optimizer_setup(tcp=tcp)
        )

    # Backward compatibility method (deprecated)
    async def log_saftey_zones(self, motion_group: MotionGroup) -> None:
        """Deprecated: Use log_safety_zones instead."""
        await self.log_safety_zones(motion_group)

    def log_safety_zones_(
        self, motion_group_id: str, optimizer_setup: models.OptimizerSetup
    ) -> None:
        log_safety_zones(motion_group_id, optimizer_setup)

    async def log_motion(
        self,
        motion_id: str,
        timing_mode=TimingMode.CONTINUE,
        time_offset: float = 0,
        tool_asset: Optional[str] = None,
    ) -> None:
        """Log motion trajectory to Rerun viewer.

        Args:
            motion_id: The motion identifier
            timing_mode: DEPRECATED - Timing mode (ignored in new implementation)
            time_offset: Time offset for visualization
            tool_asset: Optional tool asset file path
        """
        if timing_mode != TimingMode.CONTINUE:
            import warnings

            warnings.warn(
                "timing_mode parameter is deprecated and will be removed in a future version. "
                "Timing is now handled automatically per motion group.",
                DeprecationWarning,
                stacklevel=2,
            )
        logger.debug(f"log_motion called with motion_id: {motion_id}")
        try:
            # Use the bridge's own Nova client for API calls
            bridge_nova = self._bridge_nova or self.nova
            logger.debug(
                f"Using bridge_nova: {bridge_nova is not None}, bridge_nova._api_client: {hasattr(bridge_nova, '_api_client') if bridge_nova else False}"
            )

            # Fetch motion details from api
            logger.debug("Fetching motion details...")
            motion = await bridge_nova._api_client.motion_api.get_planned_motion(
                bridge_nova.cell()._cell_id, motion_id
            )
            logger.debug("Fetching optimizer config...")
            optimizer_config = (
                await bridge_nova._api_client.motion_group_infos_api.get_optimizer_configuration(
                    bridge_nova.cell()._cell_id, motion.motion_group
                )
            )
            logger.debug("Fetching trajectory...")
            trajectory = await bridge_nova._api_client.motion_api.get_motion_trajectory(
                bridge_nova.cell()._cell_id, motion_id, int(RECORDING_INTERVAL * 1000)
            )

            logger.debug("Fetching motion groups...")
            motion_groups = await bridge_nova._api_client.motion_group_api.list_motion_groups(
                bridge_nova.cell()._cell_id
            )
            motion_motion_group = next(
                (mg for mg in motion_groups.instances if mg.motion_group == motion.motion_group),
                None,
            )

            logger.debug("Fetching collision scenes...")
            collision_scenes = await bridge_nova._api_client.store_collision_scenes_api.list_stored_collision_scenes(
                cell=bridge_nova.cell()._cell_id
            )

            if motion_motion_group is None:
                raise ValueError(f"Motion group {motion.motion_group} not found")

            # Get or initialize the timer for this motion group
            motion_group_id = motion.motion_group
            current_time = self._motion_group_timers.get(motion_group_id, 0.0)

            logger.debug(
                f"Calling log_motion function with trajectory points: {len(trajectory.trajectory or [])}"
            )
            log_motion(
                motion_id=motion_id,
                model_from_controller=motion_motion_group.model_from_controller,
                motion_group=motion.motion_group,
                optimizer_config=optimizer_config,
                trajectory=trajectory.trajectory or [],
                collision_scenes=collision_scenes,
                time_offset=current_time + time_offset,
                tool_asset=tool_asset,
                show_collision_link_chain=self.show_collision_link_chain,
                show_safety_link_chain=self.show_safety_link_chain,
            )
            # Update the timer for this motion group based on trajectory duration
            if trajectory.trajectory:
                last_trajectory_point = trajectory.trajectory[-1]
                if last_trajectory_point.time is not None:
                    self._motion_group_timers[motion_group_id] = (
                        current_time + time_offset + last_trajectory_point.time
                    )
            logger.debug("log_motion completed successfully")
        except RuntimeError as e:
            if "Session is closed" in str(e):
                # Session is closed, skip trajectory logging
                logger.debug(f"Skipping trajectory logging due to closed session: {e}")
                return
            else:
                raise
        except Exception as e:
            # Log other errors but don't fail
            logger.error(f"Failed to log motion trajectory: {e}")
            raise

    async def log_trajectory(
        self,
        joint_trajectory: models.JointTrajectory,
        tcp: str,
        motion_group: MotionGroup,
        timing_mode=TimingMode.CONTINUE,
        time_offset: float = 0,
        tool_asset: Optional[str] = None,
    ) -> None:
        """Log joint trajectory to Rerun viewer.

        Args:
            joint_trajectory: The joint trajectory to log
            tcp: TCP identifier
            motion_group: Motion group for planning
            timing_mode: DEPRECATED - Timing mode (ignored in new implementation)
            time_offset: Time offset for visualization
            tool_asset: Optional tool asset file path
        """
        if timing_mode != TimingMode.CONTINUE:
            import warnings

            warnings.warn(
                "timing_mode parameter is deprecated and will be removed in a future version. "
                "Timing is now handled automatically per motion group.",
                DeprecationWarning,
                stacklevel=2,
            )

        if len(joint_trajectory.joint_positions) == 0:
            raise ValueError("No joint trajectory provided")

        load_plan_response = await motion_group._load_planned_motion(joint_trajectory, tcp)

        await self.log_motion(
            load_plan_response.motion, time_offset=time_offset, tool_asset=tool_asset
        )

    def continue_after_sync(self) -> None:
        """No longer needed with per-motion-group timing.

        This method is now a no-op since timing is automatically managed
        per motion group. Each motion group maintains its own independent timeline.

        .. deprecated::
            continue_after_sync() is deprecated and will be removed in a future version.
            Timing is now handled automatically per motion group.
        """
        import warnings

        warnings.warn(
            "continue_after_sync() is deprecated and will be removed in a future version. "
            "Timing is now handled automatically per motion group.",
            DeprecationWarning,
            stacklevel=2,
        )

    async def log_error_feedback(
        self, error_feedback: PlanTrajectoryFailedResponseErrorFeedback
    ) -> None:
        if isinstance(error_feedback.actual_instance, FeedbackOutOfWorkspace):
            if (
                error_feedback.actual_instance.invalid_tcp_pose
                and error_feedback.actual_instance.invalid_tcp_pose.position
            ):
                position = error_feedback.actual_instance.invalid_tcp_pose.position
                rr.log(
                    "motion/errors/FeedbackOutOfWorkspace",
                    rr.Points3D(
                        [[position[0], position[1], position[2]]],
                        radii=rr.Radius.ui_points([5.0]),
                        colors=[(255, 0, 0, 255)],
                        labels=["Out of Workspace"],
                    ),
                    static=True,
                )

        if isinstance(error_feedback.actual_instance, FeedbackCollision):
            collisions = error_feedback.actual_instance.collisions
            if not collisions:
                return

            for i, collision in enumerate(collisions):
                if collision.position_on_a is None or collision.position_on_b is None:
                    continue
                if collision.position_on_a.world is None or collision.position_on_b.world is None:
                    continue
                if collision.normal_world_on_b is None:
                    continue

                # Extract positions
                pos_a = collision.position_on_a.world
                pos_b = collision.position_on_b.world
                normal = collision.normal_world_on_b

                # Scale normal for visibility
                arrow_length = 50

                # Log collision points
                rr.log(
                    f"motion/errors/FeedbackCollision/collisions/point_{i}/a",
                    rr.Points3D(
                        [pos_a], radii=rr.Radius.ui_points([5.0]), colors=[(255, 0, 0, 255)]
                    ),
                    static=True,
                )

                rr.log(
                    f"motion/errors/FeedbackCollision/collisions/point_{i}/b",
                    rr.Points3D(
                        [pos_b], radii=rr.Radius.ui_points([5.0]), colors=[(0, 0, 255, 255)]
                    ),
                    static=True,
                )

                # Log normal vector as arrow
                rr.log(
                    f"motion/errors/FeedbackCollision/collisions/normal_{i}",
                    rr.Arrows3D(
                        origins=[pos_b],
                        vectors=[
                            [
                                normal[0] * arrow_length,
                                normal[1] * arrow_length,
                                normal[2] * arrow_length,
                            ]
                        ],
                        colors=[(255, 255, 0, 255)],
                    ),
                    static=True,
                )

    async def start_streaming(self, motion_group: MotionGroup) -> None:
        """Start streaming real-time robot state to Rerun viewer."""
        if motion_group in self._streaming_tasks:
            return

        task = asyncio.create_task(stream_motion_group(self, motion_group=motion_group))
        self._streaming_tasks[motion_group] = task

    async def stop_streaming(self) -> None:
        """Stop all streaming tasks."""
        for task in self._streaming_tasks.values():
            task.cancel()
        self._streaming_tasks.clear()

    async def log_actions(
        self,
        actions: list[Action | CollisionFreeMotion] | Action,
        show_connection: bool = False,
        show_labels: bool = False,
        motion_group: Optional[MotionGroup] = None,
        tcp: Optional[str] = None,
    ) -> None:
        """Log robot actions as points in the Rerun viewer.

        This method visualizes robot actions by determining their TCP poses and displaying
        them as colored points in 3D space. Joint motions are converted to poses using
        forward kinematics with the specified TCP.

        Args:
            actions: Single action or list of actions to visualize
            show_connection: Whether to draw lines connecting consecutive action points
            show_labels: Whether to display action type labels on points
            motion_group: Motion group for forward kinematics (required for joint motions)
            tcp: TCP identifier to use for forward kinematics. If None and motion_group
                 is provided, uses the first available TCP. Should match the TCP used
                 for trajectory planning to ensure consistency.

        Raises:
            ValueError: If no actions are provided
        """
        rr.reset_time()

        # Use motion group specific timing if available
        if motion_group is not None:
            motion_group_time = self._motion_group_timers.get(motion_group.motion_group_id, 0.0)
            rr.set_time(TIME_INTERVAL_NAME, duration=motion_group_time)
        else:
            # Fallback to time 0 if no motion group provided
            rr.set_time(TIME_INTERVAL_NAME, duration=0.0)

        if not isinstance(actions, list):
            actions = [actions]

        if len(actions) == 0:
            raise ValueError("No actions provided")

        # Determine the TCP to use - if not provided, get the first available TCP
        if tcp is None and motion_group is not None:
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0] if tcp_names else "Flange"

        positions = []
        point_colors = []
        labels = []

        # Keep track of the last pose for write actions
        last_pose = None
        last_joints = None

        # Process each action to get its pose
        for i, action in enumerate(actions):
            pose = None
            action_type = getattr(action, "type", type(action).__name__)

            if isinstance(action, WriteAction):
                # Write actions use the last pose or joint config
                if last_pose is not None:
                    pose = last_pose
                elif last_joints is not None and motion_group is not None and tcp is not None:
                    # Use forward kinematics to convert joint config to pose
                    pose = await self._joint_to_pose(last_joints, motion_group, tcp)
                else:
                    # Skip write actions without a previous pose/joint config
                    continue

            elif isinstance(action, CollisionFreeMotion) and isinstance(action.target, Pose):
                pose = action.target
                last_pose = pose

            elif isinstance(action, Motion):
                if hasattr(action, "target"):
                    if isinstance(action.target, Pose):
                        # Cartesian motion
                        pose = action.target
                        last_pose = pose
                    elif (
                        isinstance(action.target, tuple)
                        and motion_group is not None
                        and tcp is not None
                    ):
                        # Joint motion - use forward kinematics
                        pose = await self._joint_to_pose(action.target, motion_group, tcp)
                        last_joints = action.target
                        last_pose = pose
                    else:
                        # Skip actions without a usable target
                        continue
                else:
                    # Skip actions without a target
                    continue
            else:
                # Skip other action types that we don't know how to handle
                continue

            # If we reach here, we should have a valid pose or we need to skip
            if pose is None:
                continue

            logger.debug(f"Action {i}: {action_type}, Pose: {pose}")
            positions.append([pose.position.x, pose.position.y, pose.position.z])

            # Determine action type and color using better color palette
            from nova_rerun_bridge.colors import colors

            if isinstance(action, WriteAction):
                point_colors.append(tuple(colors[0]))  # Light purple for IO actions
            elif action_type == "joint_ptp":
                point_colors.append(tuple(colors[2]))  # Medium purple for joint motions
            elif action_type == "cartesian_ptp":
                point_colors.append(tuple(colors[4]))  # Deeper purple for cartesian motions
            elif action_type == "linear":
                point_colors.append(tuple(colors[6]))  # Dark purple for linear motions
            elif action_type == "circular":
                point_colors.append(tuple(colors[8]))  # Very dark purple for circular motions
            else:
                point_colors.append(tuple(colors[10]))  # Darkest color for other actions

            # Create descriptive label with ID and action type (only if needed)
            labels.append(f"{len(positions) - 1}: {action_type}")

        entity_path = (
            f"motion/{motion_group.motion_group_id}/actions" if motion_group else "motion/actions"
        )

        # Log all positions with labels and colors
        if positions:
            # Prepare labels
            point_labels = labels

            rr.log(
                entity_path,
                rr.Points3D(
                    positions,
                    colors=point_colors,
                    labels=point_labels,
                    show_labels=show_labels,
                    radii=rr.Radius.ui_points([8.0]),
                ),
                static=True,
            )

            # Log connections between consecutive actions if show_connection is True
            if show_connection and len(positions) > 1:
                connection_lines = []
                for i in range(len(positions) - 1):
                    connection_lines.append([positions[i], positions[i + 1]])

                rr.log(
                    f"{entity_path}/connections",
                    rr.LineStrips3D(
                        connection_lines,
                        colors=[(128, 128, 128)],  # Gray connections
                        radii=rr.Radius.ui_points([2.0]),
                    ),
                    static=True,
                )

    async def _joint_to_pose(
        self, joint_config: tuple[float, ...], motion_group: MotionGroup, tcp: str
    ) -> Pose:
        """Convert joint configuration to pose using forward kinematics.

        Args:
            joint_config: The joint configuration to convert
            motion_group: The motion group for forward kinematics
            tcp: The TCP identifier used for planning (should match the TCP used in planning)
        """
        try:
            # Use Nova's forward kinematics API to get TCP pose from joint configuration
            from nova.api import models

            # Create a joint position object
            joint_position = models.Joints(joints=list(joint_config))

            # Create TcpPoseRequest for forward kinematics calculation
            tcp_pose_request = models.TcpPoseRequest(
                motion_group=motion_group.motion_group_id, joint_position=joint_position, tcp=tcp
            )

            # Use Nova's API to calculate the TCP pose from joint configuration
            api_pose = await motion_group._api_gateway.motion_group_kinematic_api.calculate_forward_kinematic(
                cell=motion_group._cell,
                motion_group=motion_group.motion_group_id,
                tcp_pose_request=tcp_pose_request,
            )

            # Convert API pose to Nova Pose
            orientation = api_pose.orientation
            return Pose(
                (
                    api_pose.position.x,
                    api_pose.position.y,
                    api_pose.position.z,
                    orientation.x if orientation else 0.0,
                    orientation.y if orientation else 0.0,
                    orientation.z if orientation else 0.0,
                )
            )

        except Exception as e:
            logger.warning(f"Failed to convert joints to pose using forward kinematics: {e}")
            # Fallback: return a pose at origin
            return Pose((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def get_motion_group_time(self, motion_group_id: str) -> float:
        """Get the current timeline position for a motion group.

        Args:
            motion_group_id: The motion group identifier

        Returns:
            Current time position for the motion group (0.0 if not seen before)
        """
        return self._motion_group_timers.get(motion_group_id, 0.0)

    def reset_motion_group_time(self, motion_group_id: str) -> None:
        """Reset the timeline for a specific motion group back to 0.

        Args:
            motion_group_id: The motion group identifier to reset
        """
        self._motion_group_timers[motion_group_id] = 0.0

    async def __aenter__(self) -> "NovaRerunBridge":
        """Context manager entry point.

        Creates a separate Nova client instance for the bridge to use.

        Returns:
            NovaRerunBridge: Self reference for context manager usage.
        """
        # Create a separate Nova instance for the bridge using the same connection parameters
        # This ensures the bridge has its own session that won't be closed by the main program
        api_client = self.nova._api_client

        # Extract the host without the /api/v1 suffix
        host = api_client._host
        if host.endswith("/api/v1"):
            host = host[:-7]  # Remove '/api/v1'
        elif host.endswith("/api/"):
            host = host[:-5]  # Remove '/api/'

        self._bridge_nova = Nova(
            host=host,
            access_token=api_client._access_token,
            username=api_client._username,
            password=api_client._password,
            version=api_client._version,
            verify_ssl=api_client._verify_ssl,
        )
        await self._bridge_nova.__aenter__()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit point, ensures cleanup."""
        if "VSCODE_PROXY_URI" in os.environ:
            logger.info(f"Install rerun app and open the visual log on {get_rerun_address()}")

        await self.cleanup()

    async def cleanup(self) -> None:
        """Cleanup resources and close Nova API client connection."""
        # Clean up the bridge's own Nova instance
        if self._bridge_nova is not None:
            await self._bridge_nova.__aexit__(None, None, None)
            self._bridge_nova = None

        # Note: Don't clean up self.nova as it belongs to the main program
