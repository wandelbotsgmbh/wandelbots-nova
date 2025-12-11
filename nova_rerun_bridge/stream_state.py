import asyncio
import time

import rerun as rr
from loguru import logger
from scipy.spatial.transform import Rotation as R

from nova import MotionGroup, Nova, api
from nova.types import Pose
from nova.utils.downsample import downsample_stream
from nova_rerun_bridge import colors
from nova_rerun_bridge.consts import TIME_REALTIME_NAME
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.robot_visualizer import RobotVisualizer


def log_joint_positions_once(motion_group_id: str, robot: DHRobot, joint_position: list[float]):
    """Compute and log joint positions for a robot."""
    joint_positions = robot.calculate_joint_positions(joint_position)
    line_segments = [
        [joint_positions[i], joint_positions[i + 1]] for i in range(len(joint_positions) - 1)
    ]
    segment_colors = [colors.colors[i % len(colors.colors)] for i in range(len(line_segments))]

    rr.log(
        f"{motion_group_id}/dh_parameters", rr.LineStrips3D(line_segments, colors=segment_colors)
    )


class MotionGroupProcessor:
    def __init__(self):
        self.last_tcp_pose = {}

    def tcp_pose_changed(self, motion_group_id: str, tcp_pose: Pose) -> bool:
        """Check if the TCP pose has changed compared to the last logged value."""
        last_pose = self.last_tcp_pose.get(motion_group_id)

        if last_pose is None:
            # No previous pose, consider it as changed
            self.last_tcp_pose[motion_group_id] = tcp_pose
            return True

        if tcp_pose != last_pose:
            # Update the cache and return True if either position or rotation has changed
            self.last_tcp_pose[motion_group_id] = tcp_pose
            return True

        return False

    def log_tcp_orientation(self, motion_group_id: str, tcp_pose: Pose):
        """Log TCP orientation and position."""
        rotation_vector = tcp_pose.orientation.to_tuple()
        rotation = R.from_rotvec(rotation_vector)
        angle = rotation.magnitude() if rotation.magnitude() != 0 else 0.0
        axis_angle = rotation.as_rotvec() / angle if angle != 0 else [0, 0, 0]

        rr.log(
            f"{motion_group_id}/tcp_position",
            rr.Transform3D(
                translation=tcp_pose.position.to_tuple(),
                rotation=rr.RotationAxisAngle(axis=axis_angle, angle=angle),
            ),
            static=True,
        )


async def stream_motion_group(
    self,
    nova: Nova,
    motion_group: MotionGroup,
    tcp_name: str | None,
    target_frequency: float | None = 1.0 / 0.033,
) -> None:
    """Stream individual motion group state to Rerun.

    Args:
        self: Nova instance (unused but kept for compatibility)
        nova: Nova instance
        motion_group: Motion group to stream
        tcp_name: Optional TCP name
        target_frequency: Target frequency in Hz for downsampling. Default is ~30.3 Hz (33ms interval).
    """
    processor = MotionGroupProcessor()

    motion_group_description = await motion_group.get_description()
    motion_group_model = await motion_group.get_model()

    tcp_geometries: list[api.models.Collider] = []
    if motion_group_description.safety_tool_colliders is not None and tcp_name is not None:
        tool_colliders = motion_group_description.safety_tool_colliders.get(tcp_name)
        if tool_colliders is not None:
            tcp_geometries = [tool_collider for tool_collider in list(tool_colliders.root.values())]

    robot_model_geometries: list[api.models.LinkChain] = []
    if motion_group_description.safety_link_colliders is not None:
        robot_model_geometries = [
            api.models.LinkChain(
                [
                    api.models.Link(link.root)
                    for link in motion_group_description.safety_link_colliders
                ]
            )
        ]

    try:
        mounting = motion_group_description.mounting or api.models.Pose(
            position=api.models.Vector3d([0, 0, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        )
        robot = DHRobot(
            dh_parameters=motion_group_description.dh_parameters or [], mounting=mounting
        )
        rr.reset_time()
        rr.set_time(TIME_REALTIME_NAME, timestamp=time.time())
        visualizer = RobotVisualizer(
            robot=robot,
            robot_model_geometries=robot_model_geometries,
            tcp_geometries=tcp_geometries,
            static_transform=False,
            base_entity_path=motion_group.id,
            albedo_factor=[0, 255, 100],
            motion_group_model=motion_group_model,
        )

        logger.info(f"Started streaming motion group {motion_group.id}")

        async for state in downsample_stream(motion_group.stream_state(), target_frequency):
            current_joint_position = state.joint_position.root
            tcp_pose = Pose(state.tcp_pose)
            if processor.tcp_pose_changed(motion_group_id=motion_group.id, tcp_pose=tcp_pose):
                rr.reset_time()
                rr.set_time(TIME_REALTIME_NAME, timestamp=time.time())
                log_joint_positions_once(
                    motion_group_id=motion_group.id,
                    robot=robot,
                    joint_position=current_joint_position,
                )
                visualizer.log_robot_geometry(joint_position=current_joint_position)
                processor.log_tcp_orientation(motion_group_id=motion_group.id, tcp_pose=tcp_pose)

        await asyncio.sleep(0.01)  # Prevents CPU overuse
    except asyncio.CancelledError:
        logger.info(f"Stopped streaming motion group {motion_group}")
    except Exception as e:
        logger.error(f"Error streaming motion group {motion_group}: {e}")
