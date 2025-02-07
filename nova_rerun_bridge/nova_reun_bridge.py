import asyncio
from datetime import datetime
from pathlib import Path
from typing import Dict

import numpy as np
import rerun as rr
from loguru import logger
from nova import MotionGroup
from nova.actions import Action, CombinedActions, WriteAction
from nova.api import models
from nova.core.nova import Nova
from wandelbots_api_client.models import (
    FeedbackOutOfWorkspace,
    PlanTrajectoryFailedResponseErrorFeedback,
)

from nova_rerun_bridge import colors
from nova_rerun_bridge.blueprint import send_blueprint
from nova_rerun_bridge.collision_scene import log_collision_scenes
from nova_rerun_bridge.consts import RECORDING_INTERVAL, TIME_INTERVAL_NAME
from nova_rerun_bridge.helper_scripts.download_models import get_project_root
from nova_rerun_bridge.stream_state import stream_motion_group
from nova_rerun_bridge.trajectory import TimingMode, continue_after_sync, log_motion


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

    def __init__(self, nova: Nova, spawn: bool = True, recording_id=None) -> None:
        self._ensure_models_exist()
        self.nova = nova
        self._streaming_tasks = {}
        if spawn:
            recording_id = recording_id or f"nova_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            rr.init(application_id="nova", recording_id=recording_id, spawn=True)
        logger.add(sink=rr.LoggingHandler("logs/handler"))

    def _ensure_models_exist(self):
        """Ensure robot models are downloaded"""
        models_dir = Path(get_project_root()) / "models"
        if not models_dir.exists() or not list(models_dir.glob("*.glb")):
            print("Models not found, run update_robot_models() or poetry run download-models")

    async def setup_blueprint(self) -> None:
        """Configure and send blueprint configuration to Rerun.

        Fetches motion groups from Nova and configures visualization layout.
        """
        cell = self.nova.cell()

        controllers = await cell.controllers()
        motion_groups = []

        if not controllers:
            logger.warning("No controllers found")
            return

        for controller in controllers:
            for motion_group in await controller.activated_motion_groups():
                motion_groups.append(motion_group.motion_group_id)

        send_blueprint(motion_groups)
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
            timeless=True,
            static=True,
        )

    async def log_collision_scenes(self) -> Dict[str, models.CollisionScene]:
        """Fetch and log all collision scenes from Nova to Rerun."""
        collision_scenes = (
            await self.nova._api_client.store_collision_scenes_api.list_stored_collision_scenes(
                cell=self.nova.cell()._cell_id
            )
        )
        log_collision_scenes(collision_scenes)
        return collision_scenes

    async def log_collision_scene(self, scene_id: str) -> Dict[str, models.CollisionScene]:
        """Log a specific collision scene by its ID.

        Args:
            scene_id (str): The ID of the collision scene to log

        Raises:
            ValueError: If scene_id is not found in stored collision scenes
        """
        collision_scenes = (
            await self.nova._api_client.store_collision_scenes_api.list_stored_collision_scenes(
                cell=self.nova.cell()._cell_id
            )
        )

        if scene_id not in collision_scenes:
            raise ValueError(f"Collision scene with ID {scene_id} not found")

        log_collision_scenes({scene_id: collision_scenes[scene_id]})
        return {scene_id: collision_scenes[scene_id]}

    def _log_collision_scene(self, collision_scenes: Dict[str, models.CollisionScene]) -> None:
        log_collision_scenes(collision_scenes=collision_scenes)

    async def log_motion(
        self, motion_id: str, timing_mode=TimingMode.CONTINUE, time_offset: float = 0
    ) -> None:
        # Fetch motion details from api
        motion = await self.nova._api_client.motion_api.get_planned_motion(
            self.nova.cell()._cell_id, motion_id
        )
        optimizer_config = (
            await self.nova._api_client.motion_group_infos_api.get_optimizer_configuration(
                self.nova.cell()._cell_id, motion.motion_group
            )
        )
        trajectory = await self.nova._api_client.motion_api.get_motion_trajectory(
            self.nova.cell()._cell_id, motion_id, int(RECORDING_INTERVAL * 1000)
        )

        motion_groups = await self.nova._api_client.motion_group_api.list_motion_groups(
            self.nova.cell()._cell_id
        )
        motion_motion_group = next(
            (mg for mg in motion_groups.instances if mg.motion_group == motion.motion_group), None
        )

        collision_scenes = (
            await self.nova._api_client.store_collision_scenes_api.list_stored_collision_scenes(
                cell=self.nova.cell()._cell_id
            )
        )

        log_motion(
            motion_id=motion_id,
            model_from_controller=motion_motion_group.model_from_controller,
            motion_group=motion.motion_group,
            optimizer_config=optimizer_config,
            trajectory=trajectory.trajectory,
            collision_scenes=collision_scenes,
            time_offset=time_offset,
            timing_mode=timing_mode,
        )

    async def log_trajectory(
        self,
        joint_trajectory: models.JointTrajectory,
        tcp: str,
        motion_group: MotionGroup,
        timing_mode=TimingMode.CONTINUE,
        time_offset: float = 0,
    ) -> None:
        if len(joint_trajectory.joint_positions) == 0:
            raise ValueError("No joint trajectory provided")
        load_plan_response = await motion_group._load_planned_motion(joint_trajectory, tcp)
        await self.log_motion(
            load_plan_response.motion, timing_mode=timing_mode, time_offset=time_offset
        )

    def continue_after_sync(self) -> None:
        continue_after_sync()

    async def log_error_feedback(
        self, error_feedback: PlanTrajectoryFailedResponseErrorFeedback
    ) -> None:
        if isinstance(error_feedback.actual_instance, FeedbackOutOfWorkspace):
            rr.log(
                "motion/errors/FeedbackOutOfWorkspace",
                rr.Points3D(
                    [
                        error_feedback.actual_instance.invalid_tcp_pose.position[0],
                        error_feedback.actual_instance.invalid_tcp_pose.position[1],
                        error_feedback.actual_instance.invalid_tcp_pose.position[2],
                    ],
                    radii=rr.Radius.ui_points([5.0]),
                    colors=[(255, 0, 0, 255)],
                    labels=["Out of Workspace"],
                ),
                timeless=True,
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
        self, actions: list[Action] | Action, show_connection: bool = False
    ) -> None:
        from nova_rerun_bridge import trajectory

        rr.set_time_seconds(TIME_INTERVAL_NAME, trajectory._last_end_time)

        if not isinstance(actions, list):
            actions = [actions]

        if len(actions) == 0:
            raise ValueError("No actions provided")

        poses = CombinedActions(items=tuple(actions)).poses()
        positions = []
        point_colors = []
        use_red = False

        # Collect all positions and determine colors
        for i, action in enumerate(actions):
            if isinstance(action, WriteAction):
                use_red = True
            if i < len(poses):  # Only process if there's a corresponding pose
                pose = poses[i]
                logger.debug(f"Pose: {pose}")
                positions.append([pose.position.x, pose.position.y, pose.position.z])
                point_colors.append(colors.colors[1] if use_red else colors.colors[9])

        # Log all positions at once
        rr.log(
            "motion/actions",
            rr.Points3D(positions, colors=point_colors, radii=rr.Radius.ui_points([5.0])),
            timeless=True,
            static=True,
        )

        if show_connection:
            rr.log(
                "motion/actions/connection",
                rr.LineStrips3D([positions], colors=[155, 155, 155, 50]),
            )

    async def __aenter__(self) -> "NovaRerunBridge":
        """Context manager entry point.

        Returns:
            NovaRerunBridge: Self reference for context manager usage.
        """
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit point, ensures cleanup."""
        await self.cleanup()

    async def cleanup(self) -> None:
        """Cleanup resources and close Nova API client connection."""
        if hasattr(self.nova, "_api_client"):
            await self.nova._api_client.close()
