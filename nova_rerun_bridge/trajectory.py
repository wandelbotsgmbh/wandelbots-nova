from enum import Enum, auto
import uuid
from typing import Optional

import numpy as np
import rerun as rr
import rerun.archetypes as ra
from scipy.spatial.transform import Rotation

from nova import MotionGroup, api
from nova_rerun_bridge.collision_scene import extract_link_chain_and_tcp
from nova_rerun_bridge.consts import TIME_INTERVAL_NAME
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.robot_visualizer import RobotVisualizer


class TimingMode(Enum):
    """Controls how trajectories are timed relative to each other.

    .. deprecated::
        TimingMode is deprecated and will be removed in a future version.
        The new viewer system handles timing automatically per motion group.
    """

    RESET = auto()  # Start at time_offset
    CONTINUE = auto()  # Start after last trajectory
    SYNC = auto()  # Use exact time_offset, don't update last time
    OVERRIDE = auto()  # Use exact time_offset and reset last time


# Deprecated global timing variables - kept for backward compatibility
_last_end_time = 0.0
_last_offset = 0.0

_visualizer_cache: dict[str, RobotVisualizer] = {}


async def log_motion(
    trajectory: api.models.JointTrajectory,
    tcp: str,
    motion_group: MotionGroup,
    collision_setups: dict[str, api.models.CollisionSetup],
    time_offset: float = 0,
    timing_mode: TimingMode = TimingMode.CONTINUE,  # Deprecated parameter kept for compatibility
    tool_asset: str | None = None,
    show_collision_link_chain: bool = False,
    show_collision_tool: bool = True,
    show_safety_link_chain: bool = True,
):
    """
    Fetch and process a single motion for visualization.

    Args:
        trajectory: Joint trajectory to log
        tcp: TCP to log
        motion_group: Motion group to log
        collision_setups: Dictionary of collision setups
        time_offset: Time offset for visualization
        timing_mode: DEPRECATED - Timing mode control (ignored in new implementation)
        tool_asset: Optional tool asset file path
        show_collision_link_chain: Whether to show collision geometry
        show_safety_link_chain: Whether to show safety geometry
    """
    # Issue deprecation warning if timing_mode is explicitly used
    if timing_mode != TimingMode.CONTINUE:
        import warnings

        warnings.warn(
            "TimingMode parameter is deprecated and will be removed in a future version. "
            "Timing is now handled automatically per motion group.",
            DeprecationWarning,
            stacklevel=2,
        )

    motion_group_setup = await motion_group.get_setup(tcp)
    motion_group_model = (await motion_group.get_model()).root
    motion_group_id = motion_group.motion_group_id
    motion_id = str(uuid.uuid4())

    # Initialize DHRobot and Visualizer
    if motion_group_model == "Yaskawa_TURN2":
        if motion_group_setup.dh_parameters is not None:
            motion_group_setup.dh_parameters[0].a = 0
            motion_group_setup.dh_parameters[0].d = 360
            motion_group_setup.dh_parameters[0].alpha = np.pi / 2
            motion_group_setup.dh_parameters[0].theta = 0

            motion_group_setup.dh_parameters[1].a = 0
            motion_group_setup.dh_parameters[1].d = 0
            motion_group_setup.dh_parameters[1].alpha = 0
            motion_group_setup.dh_parameters[1].theta = np.pi / 2

    if motion_group_setup.dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    robot = DHRobot(motion_group_setup.dh_parameters, motion_group_setup.mounting)

    collision_link_chain, collision_tcp = extract_link_chain_and_tcp(
        collision_setups, motion_group_setup.motion_group_type
    )

    rr.reset_time()
    rr.set_time(TIME_INTERVAL_NAME, duration=time_offset)

    # Get or create visualizer from cache
    if motion_group not in _visualizer_cache:
        collision_link_chain, collision_tcp = extract_link_chain_and_tcp(
            collision_setups, motion_group_setup.motion_group_type
        )

        _visualizer_cache[motion_group] = RobotVisualizer(
            robot=robot,
            robot_model_geometries=motion_group_setup.safety_setup.robot_model_geometries or [],
            tcp_geometries=motion_group_setup.safety_setup.tcp_geometries or [],
            static_transform=False,
            base_entity_path=f"motion/{motion_group_id}",
            model_from_controller=motion_group_model,
            collision_link_chain=collision_link_chain,
            collision_tcp=collision_tcp,
            show_collision_link_chain=show_collision_link_chain,
            show_collision_tool=show_collision_tool,
            show_safety_link_chain=show_safety_link_chain,
        )

    visualizer = _visualizer_cache[motion_group]

    # Process trajectory points
    log_trajectory(
        motion_id=motion_id,
        trajectory=trajectory,
        motion_group=motion_group,
        robot=robot,
        visualizer=visualizer,
        trajectory=trajectory,
        motion_group_setup=motion_group_setup,
        timer_offset=time_offset,
        tool_asset=tool_asset,
    )

    del trajectory
    del robot
    del visualizer


def get_times_column(
    trajectory: list[api.models.TrajectorySample], timer_offset: float = 0
) -> rr.TimeColumn:
    times = np.array([timer_offset + point.time for point in trajectory])
    times_column = rr.TimeColumn(TIME_INTERVAL_NAME, duration=times)
    return times_column


async def log_trajectory(
    motion_id: str,
    trajectory: api.models.JointTrajectory,
    motion_group: MotionGroup,
    robot: DHRobot,
    visualizer: RobotVisualizer,
    motion_group_setup: api.models.MotionGroupSetup,
    timer_offset: float,
    tool_asset: Optional[str] = None,
):
    """
    Process a single trajectory point and log relevant data.
    """
    rr.reset_time()
    rr.set_time(TIME_INTERVAL_NAME, duration=timer_offset)

    times_column = get_times_column(trajectory, timer_offset)
    motion_group_id = motion_group.motion_group_id

    # TODO: calculate tcp pose from joint positions
    joint_positions = [tuple(p.root) for p in trajectory.joint_positions]
    tcp_poses = await motion_group.forward_kinematics(joints=joint_positions, tcp=trajectory.tcp)
    points = [[p.position.x, p.position.y, p.position.z] for p in tcp_poses]

    rr.log(
        f"motion/{motion_group_id}/trajectory",
        rr.LineStrips3D([points], colors=[[1.0, 1.0, 1.0, 1.0]]),
    )

    rr.log("logs/motion", rr.TextLog(f"{motion_group_id}/{motion_id}", level=rr.TextLogLevel.INFO))

    # Calculate and log joint positions
    line_segments_batch = []
    for point in trajectory:
        joint_positions = robot.calculate_joint_positions(point.joint_position)
        line_segments_batch.append([joint_positions])  # Wrap each as a line strip

    rr.send_columns(
        f"motion/{motion_group_id}/dh_parameters",
        indexes=[times_column],
        columns=rr.LineStrips3D.columns(strips=line_segments_batch),
    )
    rr.log(
        f"motion/{motion_group_id}/dh_parameters",
        rr.LineStrips3D.from_fields(clear_unset=True, colors=[0.5, 0.5, 0.5, 1.0]),
    )

    # Log the robot geometries
    visualizer.log_robot_geometries(trajectory, times_column)

    # Log TCP pose/orientation
    log_tcp_pose(tcp_poses=[], motion_group_id=motion_group_id, times_column=times_column, tool_asset=tool_asset)

    # Log joint data
    log_joint_data(trajectory=trajectory, motion_group=motion_group, times_column=times_column, motion_group_setup=motion_group_setup)


def log_tcp_pose(
    tcp_poses: list[api.models.Pose],
    motion_group_id: str,
    times_column,
    tool_asset: str | None = None,
):
    """
    Log TCP pose (position + orientation) data.
    """
    # TODO: correct parsing here.
    # Extract positions and orientations from the trajectory
    positions = [[p.position.x, p.position.y, p.position.z] for p in tcp_poses]
    orientations = Rotation.from_rotvec(
        [[p.orientation.x, p.orientation.y, p.orientation.z] for p in tcp_poses]
    ).as_quat()

    # Log TCP and tool asset
    tcp_entity_path = f"/motion/{motion_group_id}/tcp_position"
    rr.log(tcp_entity_path, rr.Transform3D(clear=False, axis_length=100))
    if tool_asset:
        rr.log(tcp_entity_path, rr.Asset3D(path=tool_asset), static=True)

    rr.send_columns(
        tcp_entity_path,
        indexes=[times_column],
        columns=rr.Transform3D.columns(translation=positions, quaternion=orientations),
    )


def log_joint_data(
    trajectory: list[api.models.TrajectorySample],
    motion_group,
    times_column,
    motion_group_setup: api.models.MotionGroupSetup,
) -> None:
    """
    Log joint-related data (position, velocity, acceleration, torques) from a trajectory as columns.
    """
    # Initialize lists for each joint and each data type
    if motion_group_setup.dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    num_joints = len(motion_group_setup.dh_parameters)
    joint_data: dict[str, list[list[float]]] = {
        "velocity": [[] for _ in range(num_joints)],
        "acceleration": [[] for _ in range(num_joints)],
        "position": [[] for _ in range(num_joints)],
        "torque": [[] for _ in range(num_joints)],
        "velocity_lower_limit": [[] for _ in range(num_joints)],
        "velocity_upper_limit": [[] for _ in range(num_joints)],
        "acceleration_lower_limit": [[] for _ in range(num_joints)],
        "acceleration_upper_limit": [[] for _ in range(num_joints)],
        "position_lower_limit": [[] for _ in range(num_joints)],
        "position_upper_limit": [[] for _ in range(num_joints)],
        "torque_limit": [[] for _ in range(num_joints)],
    }

    # Collect data from the trajectory
    for point in trajectory:
        for i in range(num_joints):
            joint_data["velocity"][i].append(point.joint_velocity.joints[i])
            joint_data["acceleration"][i].append(point.joint_acceleration.joints[i])
            joint_data["position"][i].append(point.joint_position.joints[i])
            if point.joint_torques and len(point.joint_torques.joints) > i:
                joint_data["torque"][i].append(point.joint_torques.joints[i])

            # Collect joint limits
            joint_data["velocity_lower_limit"][i].append(
                -motion_group_setup.global_limits.joint_velocity_limits[i]
            )
            joint_data["velocity_upper_limit"][i].append(
                motion_group_setup.global_limits.joint_velocity_limits[i]
            )
            joint_data["acceleration_lower_limit"][i].append(
                -motion_group_setup.global_limits.joint_acceleration_limits[i]
            )
            joint_data["acceleration_upper_limit"][i].append(
                motion_group_setup.global_limits.joint_acceleration_limits[i]
            )
            joint_data["position_lower_limit"][i].append(
                motion_group_setup.global_limits.joint_position_limits[i].lower_limit
            )
            joint_data["position_upper_limit"][i].append(
                motion_group_setup.global_limits.joint_position_limits[i].upper_limit
            )
            if point.joint_torques and len(point.joint_torques.joints) > i:
                joint_data["torque_limit"][i].append(
                    motion_group_setup.global_limits.joint_torque_limits[i]
                )

    # Send columns if data is not empty
    for data_type, data in joint_data.items():
        for i in range(num_joints):
            if data[i]:
                rr.send_columns(
                    f"motion/{motion_group}/joint_{data_type}_{i + 1}",
                    indexes=[times_column],
                    columns=[*ra.Scalars.columns(scalars=data[i])],
                )


def to_trajectory_samples(self) -> list[api.models.TrajectorySample]:
    """Convert JointTrajectory to list of TrajectorySample objects."""
    samples = []
    for joint_pos, time, location in zip(self.joint_positions, self.times, self.locations):
        sample = api.models.TrajectorySample(
            joint_position=joint_pos, time=time, location_on_trajectory=location
        )
        samples.append(sample)
    return samples


def continue_after_sync():
    """Continue timing after a sync operation.

    .. deprecated::
        continue_after_sync() is deprecated and will be removed in a future version.
        The new viewer system handles timing automatically per motion group.
    """
    import warnings

    warnings.warn(
        "continue_after_sync() is deprecated and will be removed in a future version. "
        "Timing is now handled automatically per motion group.",
        DeprecationWarning,
        stacklevel=2,
    )

    global _last_end_time, _last_offset
    effective_offset = _last_end_time + _last_offset
    _last_offset = 0
    _last_end_time = effective_offset
