import asyncio
import time

import rerun as rr
from loguru import logger
from scipy.spatial.transform import Rotation as R

from nova import MotionGroup
from nova_rerun_bridge import colors
from nova_rerun_bridge.consts import TIME_REALTIME_NAME
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.robot_visualizer import RobotVisualizer


def log_joint_positions_once(motion_group: str, robot: DHRobot, joint_position):
    """Compute and log joint positions for a robot."""
    joint_positions = robot.calculate_joint_positions(joint_position)
    line_segments = [
        [joint_positions[i], joint_positions[i + 1]] for i in range(len(joint_positions) - 1)
    ]
    segment_colors = [colors.colors[i % len(colors.colors)] for i in range(len(line_segments))]

    rr.log(f"{motion_group}/dh_parameters", rr.LineStrips3D(line_segments, colors=segment_colors))


class MotionGroupProcessor:
    def __init__(self):
        self.last_tcp_pose = {}

    def tcp_pose_changed(self, motion_group: str, tcp_pose) -> bool:
        """Check if the TCP pose has changed compared to the last logged value."""
        last_pose = self.last_tcp_pose.get(motion_group)
        current_position = [tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z]
        current_rotation = [tcp_pose.orientation.x, tcp_pose.orientation.y, tcp_pose.orientation.z]

        if last_pose is None:
            # No previous pose, consider it as changed
            self.last_tcp_pose[motion_group] = (current_position, current_rotation)
            return True

        last_position, last_rotation = last_pose
        if current_position != last_position or current_rotation != last_rotation:
            # Update the cache and return True if either position or rotation has changed
            self.last_tcp_pose[motion_group] = (current_position, current_rotation)
            return True

        return False

    def log_tcp_orientation(self, motion_group: str, tcp_pose):
        """Log TCP orientation and position."""
        rotation_vector = [tcp_pose.orientation.x, tcp_pose.orientation.y, tcp_pose.orientation.z]
        rotation = R.from_rotvec(rotation_vector)
        angle = rotation.magnitude() if rotation.magnitude() != 0 else 0.0
        axis_angle = rotation.as_rotvec() / angle if angle != 0 else [0, 0, 0]

        rr.log(
            f"{motion_group}/tcp_position",
            rr.Transform3D(
                translation=[tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z],
                rotation=rr.RotationAxisAngle(axis=axis_angle, angle=angle),
            ),
            static=True,
        )


async def stream_motion_group(self, motion_group: MotionGroup) -> None:
    """Stream individual motion group state to Rerun."""
    processor = MotionGroupProcessor()

    motion_groups = await self.nova._api_client.motion_group_api.list_motion_groups(
        self.nova.cell()._cell_id
    )
    motion_motion_group = next(
        (mg for mg in motion_groups.instances if mg.motion_group == motion_group.motion_group_id),
        None,
    )

    if motion_motion_group is None:
        logger.error(f"Motion group {motion_group} not found")
        return

    try:
        optimizer_config = (
            await self.nova._api_client.motion_group_infos_api.get_optimizer_configuration(
                self.nova.cell()._cell_id, motion_group.motion_group_id
            )
        )

        robot = DHRobot(optimizer_config.dh_parameters, optimizer_config.mounting)
        rr.reset_time()
        rr.set_time(TIME_REALTIME_NAME, timestamp=time.time())
        visualizer = RobotVisualizer(
            robot=robot,
            robot_model_geometries=optimizer_config.safety_setup.robot_model_geometries,
            tcp_geometries=optimizer_config.safety_setup.tcp_geometries,
            static_transform=False,
            base_entity_path=motion_group.motion_group_id,
            albedo_factor=[0, 255, 100],
            model_from_controller=motion_motion_group.model_from_controller,
        )

        logger.info(f"Started streaming motion group {motion_group}")
        async for state in self.nova._api_client.motion_group_infos_api.stream_motion_group_state(
            self.nova.cell()._cell_id, motion_group.motion_group_id
        ):
            if processor.tcp_pose_changed(motion_group.motion_group_id, state.state.tcp_pose):
                rr.reset_time()
                rr.set_time(TIME_REALTIME_NAME, timestamp=time.time())

                # Log joint positions
                log_joint_positions_once(
                    motion_group.motion_group_id, robot, state.state.joint_position
                )

                # Log robot geometries
                visualizer.log_robot_geometry(state.state.joint_position)

                processor.log_tcp_orientation(motion_group.motion_group_id, state.state.tcp_pose)

        await asyncio.sleep(0.01)  # Prevents CPU overuse
    except asyncio.CancelledError:
        logger.info(f"Stopped streaming motion group {motion_group}")
    except Exception as e:
        logger.error(f"Error streaming motion group {motion_group}: {e}")
