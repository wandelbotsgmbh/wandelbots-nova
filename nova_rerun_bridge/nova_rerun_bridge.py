import asyncio
import math
import warnings
import os
from datetime import datetime
from typing import Optional

import numpy as np
import rerun as rr
from loguru import logger

from nova import MotionGroup, api
from nova.actions import Action
from nova.actions.io import WriteAction
from nova.actions.motions import CollisionFreeMotion, Motion
from nova.core.nova import Nova
from nova.types.pose import Pose
from nova_rerun_bridge.blueprint import send_blueprint
from nova_rerun_bridge.collision_scene import log_collision_setups
from nova_rerun_bridge.consts import TIME_INTERVAL_NAME
from nova_rerun_bridge.helper_scripts.code_server_helpers import get_rerun_address
from nova_rerun_bridge.safety_zones import log_safety_zones
from nova_rerun_bridge.stream_state import stream_motion_group
from nova_rerun_bridge.trajectory import MotionGroupTimeline, TimingMode, log_motion


def _resolve_show_collision(
    show_collision: Optional[bool], link_chain: Optional[bool], tool: Optional[bool]
) -> Optional[bool]:
    """Settle the collision setting, honouring its two deprecated aliases.

    ``show_collision_link_chain`` and ``show_collision_tool`` split one choice
    in two; both kinds of volume come out of the URDF's ``<collision>``, so
    ``show_collision`` covers them together and wins over the aliases.
    """
    if link_chain is None and tool is None:
        return show_collision
    warnings.warn(
        "show_collision_link_chain and show_collision_tool are deprecated; "
        "use show_collision, which covers both.",
        DeprecationWarning,
        stacklevel=3,
    )
    if show_collision is not False:
        # The caller set the new one; it wins over what it replaced.
        return show_collision
    return bool(link_chain or tool)


class NovaRerunBridge:
    """Bridge between Nova and Rerun for visualization.

    This class provides functionality to visualize Nova data in Rerun.
    It handles trajectoy, collision scenes, blueprints and proper cleanup of resources.

    Robot models are downloaded on-demand when needed during trajectory visualization
    or state streaming via the NOVA API.

    Args:
        nova (Nova): Instance of Nova client
        spawn (bool, optional): Whether to spawn Rerun viewer. Defaults to True.
        state_sample_interval_ms: Target interval between live robot-state samples.
    """

    def __init__(
        self,
        nova: Nova,
        spawn: bool = True,
        recording_id=None,
        show_safety_link_chain: bool = True,
        show_collision: bool | None = False,
        show_collision_link_chain: bool | None = None,
        show_collision_tool: bool | None = None,
        state_sample_interval_ms: float = 1000.0 / 30.0,
        tcp_tools: dict[str, str] | None = None,
    ) -> None:
        # Store the Nova instance for API calls
        self.nova = nova
        self.tcp_tools: dict[str, str] = tcp_tools or {}
        """Tool mesh per TCP id, exported onto that TCP's frame in the URDF so
        the tool rides the robot instead of being drawn beside it."""
        self._streaming_tasks: dict[MotionGroup, asyncio.Task] = {}
        # Timing per motion group: one clock each, so plans logged one after
        # another line up instead of landing on top of each other.
        self._timeline = MotionGroupTimeline()
        self.show_safety_link_chain = show_safety_link_chain
        self.show_collision = _resolve_show_collision(
            show_collision, show_collision_link_chain, show_collision_tool
        )
        """Whether the URDF's collision geometry is drawn, see-through. Off by
        default; ``None`` draws it only where the model carries no visual
        mesh."""
        if not math.isfinite(state_sample_interval_ms) or state_sample_interval_ms <= 0:
            raise ValueError("state_sample_interval_ms must be a positive finite value")
        self.state_sample_interval_ms = state_sample_interval_ms

        recording_id = recording_id or f"nova_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # In CI/GitHub Actions we should never attempt to spawn/connect the Rerun viewer.
        # It's optional and can introduce long timeouts/failures (no local proxy running).
        if (os.environ.get("CI") or os.environ.get("GITHUB_ACTIONS")) and os.environ.get(
            "NOVA_RERUN_SPAWN", ""
        ).lower() not in {"1", "true", "yes", "on"}:
            spawn = False

        try:
            if "VSCODE_PROXY_URI" in os.environ:
                rr.init(application_id="nova", recording_id=recording_id, spawn=False)
                rr.save("nova.rrd")
                logger.info(f"Install rerun app and open the visual log on {get_rerun_address()}")
            elif spawn:
                rr.init(application_id="nova", recording_id=recording_id, spawn=True)

            # Attach loguru to rerun (best-effort).
            logger.add(sink=rr.LoggingHandler("logs/handler"))
        except Exception as e:
            # Rerun is a best-effort visualization backend. Never fail Nova execution if
            # the viewer cannot be spawned/connected.
            logger.warning(f"Rerun initialization failed; continuing without Rerun viewer: {e}")

    async def _controllers(self, cell) -> list:
        """The cell's controllers, by name if their configurations do not parse.

        Listing them reads each controller's configuration, and one this client
        cannot parse -- a manufacturer whose name its enum does not carry, say
        -- fails the whole call. A blueprint needs the names and their motion
        groups, so it falls back to those rather than going without.
        """
        try:
            return await cell.controllers()
        except Exception as exc:  # noqa: BLE001  # a blueprint is not worth a run
            logger.warning("Could not list controllers ({}); falling back to names", exc)
            names = await self.nova._api_client.controller_api.list_robot_controllers(
                cell=cell.cell_id
            )
            return [await cell.controller(name) for name in names]

    async def setup_blueprint(self) -> None:
        """Configure and send blueprint configuration to Rerun.

        Fetches motion groups from Nova and configures visualization layout.
        """
        cell = self.nova.cell()

        controllers = await self._controllers(cell)
        if not controllers:
            logger.warning("No controllers found")
            return

        motion_groups = []
        for controller in controllers:
            for motion_group in await controller.motion_groups():
                motion_groups.append(motion_group.id)

        rr.reset_time()
        rr.set_time(TIME_INTERVAL_NAME, duration=0)

        send_blueprint(motion_groups, True)
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

    def log_collision_setups(
        self, collision_setups: dict[str, api.models.CollisionSetup]
    ) -> dict[str, api.models.CollisionSetup]:
        """Fetch and log all collision setups from Nova to Rerun."""
        # collision_setups = (
        #     await self.nova.api.store_collision_setups_api.list_stored_collision_setups(
        #         cell=self.nova.cell()._cell_id
        #     )
        # )
        log_collision_setups(collision_setups)
        return collision_setups

    async def log_collision_setup(self, setup_id: str) -> dict[str, api.models.CollisionSetup]:
        """Log a specific collision scene by its ID.

        Args:
            setup_id (str): The ID of the collision setup to log

        Raises:
            ValueError: If scene_id is not found in stored collision scenes
        """
        collision_setups = (
            await self.nova._api_client.store_collision_setups_api.list_stored_collision_setups(
                cell=self.nova.cell()._cell_id
            )
        )

        if setup_id not in collision_setups:
            raise ValueError(f"Collision setup with ID {setup_id} not found")

        log_collision_setups({setup_id: collision_setups[setup_id]})
        return {setup_id: collision_setups[setup_id]}

    async def log_safety_zones(self, motion_group: MotionGroup) -> None:
        rr.reset_time()
        rr.set_time(TIME_INTERVAL_NAME, duration=0)

        motion_group_description = await motion_group.get_description()
        log_safety_zones(
            motion_group_id=motion_group.id, motion_group_description=motion_group_description
        )

    async def log_motion(
        self,
        trajectory: api.models.JointTrajectory,
        tcp: str,
        collision_setups: dict[str, api.models.CollisionSetup],
        motion_group: MotionGroup,
        timing_mode=TimingMode.CONTINUE,
        time_offset: float = 0,
        tool_asset: Optional[str] = None,
    ) -> None:
        """Log motion trajectory to Rerun viewer.

        Args:
            trajectory: The trajectory to log
            tcp: TCP identifier
            collision_setups: The plan's collision setups
            motion_group: The motion group that will drive it
            timing_mode: Where it starts against what was logged before; see
                :class:`~nova_rerun_bridge.trajectory.TimingMode`
            time_offset: Read as *timing_mode* says
            tool_asset: Optional tool asset file path
        """
        try:
            logger.debug(
                f"Calling log_motion function with trajectory points: {len(trajectory.joint_positions or [])}"
            )
            await log_motion(
                trajectory=trajectory,
                tcp=tcp,
                motion_group=motion_group,
                collision_setups=collision_setups,
                timing_mode=timing_mode,
                time_offset=time_offset,
                tool_asset=tool_asset,
                tool_assets=self.tcp_tools or None,
                timeline=self._timeline,
                show_safety_link_chain=self.show_safety_link_chain,
                show_collision=self.show_collision,
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
        trajectory: api.models.JointTrajectory,
        tcp: str,
        motion_group: MotionGroup,
        collision_setups: dict[str, api.models.CollisionSetup],
        timing_mode=TimingMode.CONTINUE,
        time_offset: float = 0,
        tool_asset: Optional[str] = None,
    ) -> None:
        """Log joint trajectory to Rerun viewer.

        Args:
            joint_trajectory: The joint trajectory to log
            tcp: TCP identifier
            motion_group: Motion group for planning
            timing_mode: Where it starts against what was logged before; see
                :class:`~nova_rerun_bridge.trajectory.TimingMode`
            time_offset: Read as *timing_mode* says
            tool_asset: Optional tool asset file path
        """
        if len(trajectory.joint_positions) == 0:
            raise ValueError("No joint trajectory provided")

        await self.log_motion(
            trajectory=trajectory,
            tcp=tcp,
            motion_group=motion_group,
            collision_setups=collision_setups,
            timing_mode=timing_mode,
            time_offset=time_offset,
            tool_asset=tool_asset,
        )

    def continue_after_sync(self) -> None:
        """No longer needed with per-motion-group timing.

        A no-op: every motion group carries its own clock, so trajectories
        already follow each other without being told to. Each motion group maintains its own independent timeline.

        .. deprecated::
            continue_after_sync() is deprecated and will be removed in a future version.
            Every motion group carries its own clock; see
            :class:`~nova_rerun_bridge.trajectory.TimingMode`.
        """
        import warnings

        warnings.warn(
            "continue_after_sync() is deprecated and will be removed in a future "
            "version: each motion group carries its own clock.",
            DeprecationWarning,
            stacklevel=2,
        )

    async def log_error_feedback(
        self, error_feedback: api.models.PlanTrajectoryFailedResponse
    ) -> None:
        if isinstance(error_feedback.error_feedback, api.models.FeedbackOutOfWorkspace):
            if (
                error_feedback.error_feedback.invalid_tcp_pose
                and error_feedback.error_feedback.invalid_tcp_pose.position
            ):
                position = error_feedback.error_feedback.invalid_tcp_pose.position
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

        if isinstance(error_feedback.error_feedback, api.models.FeedbackCollision):
            collisions = error_feedback.error_feedback.collisions
            if not collisions:
                return

            for i, collision in enumerate(collisions):
                if collision.position_on_a is None or collision.position_on_b is None:
                    continue
                if collision.position_on_a.root is None or collision.position_on_b.root is None:
                    continue
                if collision.normal_root_on_b is None:
                    continue

                # Extract positions
                # TODO: is it local or root? (world)
                pos_a = collision.position_on_a.root
                pos_b = collision.position_on_b.root
                normal = collision.normal_root_on_b

                # Scale normal for visibility
                arrow_length = 50

                # Log collision points
                rr.log(
                    f"motion/errors/FeedbackCollision/collisions/point_{i}/a",
                    rr.Points3D(
                        positions=[pos_a],
                        radii=rr.Radius.ui_points([5.0]),
                        colors=[(255, 0, 0, 255)],
                    ),
                    static=True,
                )

                rr.log(
                    f"motion/errors/FeedbackCollision/collisions/point_{i}/b",
                    rr.Points3D(
                        positions=[pos_b],
                        radii=rr.Radius.ui_points([5.0]),
                        colors=[(0, 0, 255, 255)],
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

        task = asyncio.create_task(
            stream_motion_group(
                self,
                nova=self.nova,
                motion_group=motion_group,
                tcp_name=None,
                target_frequency=1000.0 / self.state_sample_interval_ms,
                show_collision=self.show_collision,
                tool_assets=self.tcp_tools or None,
            )
        )
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
            motion_group_time = self._timeline.times.get(motion_group.id, 0.0)
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

        entity_path = f"motion/{motion_group.id}/actions" if motion_group else "motion/actions"

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
            # Create a joint position object
            poses = await motion_group.forward_kinematics([joint_config], tcp)
            return poses[0]

        except Exception as e:
            logger.warning(f"Failed to convert joints to pose using forward kinematics: {e}")
            # Fallback: return a pose at origin
            return Pose((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    @property
    def _motion_group_timers(self) -> dict[str, float]:
        """The motion group clocks, under the name they had before."""
        return self._timeline.times

    def get_motion_group_time(self, motion_group_id: str) -> float:
        """Get the current timeline position for a motion group.

        Args:
            motion_group_id: The motion group identifier

        Returns:
            Current time position for the motion group (0.0 if not seen before)
        """
        return self._timeline.times.get(motion_group_id, 0.0)

    def reset_motion_group_time(self, motion_group_id: str) -> None:
        """Reset the timeline for a specific motion group back to 0.

        Args:
            motion_group_id: The motion group identifier to reset
        """
        self._timeline.times[motion_group_id] = 0.0

    async def __aenter__(self) -> "NovaRerunBridge":
        """Context manager entry point.

        Note: This is primarily for standalone usage of NovaRerunBridge.
        When used via the viewer integration, the Nova instance is already
        connected and ready to use.
        """
        # For standalone usage, ensure the Nova instance is connected
        if not self.nova.is_connected():
            await self.nova.open()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit point, ensures cleanup."""
        if "VSCODE_PROXY_URI" in os.environ:
            logger.info(f"Install rerun app and open the visual log on {get_rerun_address()}")

        await self.cleanup()

    async def cleanup(self) -> None:
        """
        Cleanup resources.

        This method is intentionally left empty because the Nova instance (`self.nova`)
        belongs to the caller and should not be cleaned up here.
        """
