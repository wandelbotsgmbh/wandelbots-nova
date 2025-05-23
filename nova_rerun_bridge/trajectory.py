from enum import Enum, auto
from typing import Optional

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation

from nova.api import models
from nova_rerun_bridge.collision_scene import extract_link_chain_and_tcp
from nova_rerun_bridge.consts import TIME_INTERVAL_NAME
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.robot_visualizer import RobotVisualizer


class TimingMode(Enum):
    """Controls how trajectories are timed relative to each other."""

    RESET = auto()  # Start at time_offset
    CONTINUE = auto()  # Start after last trajectory
    SYNC = auto()  # Use exact time_offset, don't update last time
    OVERRIDE = auto()  # Use exact time_offset and reset last time


# Track both last end time and last offset separately
_last_end_time = 0.0
_last_offset = 0.0

_visualizer_cache: dict[str, RobotVisualizer] = {}


def log_motion(
    motion_id: str,
    model_from_controller: str,
    motion_group: str,
    optimizer_config: models.OptimizerSetup,
    trajectory: list[models.TrajectorySample],
    collision_scenes: dict[str, models.CollisionScene],
    time_offset: float = 0,
    timing_mode: TimingMode = TimingMode.CONTINUE,
    tool_asset: Optional[str] = None,
):
    """
    Fetch and process a single motion with timing control.

    Args:
        ...existing args...
        timing_mode: Controls how trajectory timing is handled
            RESET: Start at time_offset (default)
            CONTINUE: Start after last trajectory
            SYNC: Use exact time_offset provided
    """
    global _visualizer_cache, _last_end_time, _last_offset

    # Calculate start time based on timing mode
    if timing_mode == TimingMode.CONTINUE:
        effective_offset = _last_end_time + _last_offset
    elif timing_mode == TimingMode.SYNC:
        effective_offset = _last_end_time
    elif timing_mode == TimingMode.OVERRIDE:
        effective_offset = time_offset
        _last_end_time = time_offset
    else:  # TimingMode.RESET
        effective_offset = time_offset
        _last_end_time = time_offset

    # Initialize DHRobot and Visualizer
    if model_from_controller == "Yaskawa_TURN2":
        if optimizer_config.dh_parameters is not None:
            optimizer_config.dh_parameters[0].a = 0
            optimizer_config.dh_parameters[0].d = 360
            optimizer_config.dh_parameters[0].alpha = np.pi / 2
            optimizer_config.dh_parameters[0].theta = 0

            optimizer_config.dh_parameters[1].a = 0
            optimizer_config.dh_parameters[1].d = 0
            optimizer_config.dh_parameters[1].alpha = 0
            optimizer_config.dh_parameters[1].theta = np.pi / 2

    if optimizer_config.dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    robot = DHRobot(optimizer_config.dh_parameters, optimizer_config.mounting)

    collision_link_chain, collision_tcp = extract_link_chain_and_tcp(
        collision_scenes, optimizer_config.motion_group_type
    )

    rr.reset_time()
    rr.set_time_seconds(TIME_INTERVAL_NAME, effective_offset)

    # Get or create visualizer from cache
    if motion_group not in _visualizer_cache:
        collision_link_chain, collision_tcp = extract_link_chain_and_tcp(
            collision_scenes, optimizer_config.motion_group_type
        )

        _visualizer_cache[motion_group] = RobotVisualizer(
            robot=robot,
            robot_model_geometries=optimizer_config.safety_setup.robot_model_geometries or [],
            tcp_geometries=optimizer_config.safety_setup.tcp_geometries or [],
            static_transform=False,
            base_entity_path=f"motion/{motion_group}",
            model_from_controller=model_from_controller,
            collision_link_chain=collision_link_chain,
            collision_tcp=collision_tcp,
        )

    visualizer = _visualizer_cache[motion_group]

    # Process trajectory points
    log_trajectory(
        motion_id=motion_id,
        motion_group=motion_group,
        robot=robot,
        visualizer=visualizer,
        trajectory=trajectory,
        optimizer_config=optimizer_config,
        timer_offset=effective_offset,
        tool_asset=tool_asset,
    )

    # Update last times based on timing mode
    if trajectory:
        if trajectory[-1].time is None:
            raise ValueError("Last trajectory point has no time")

        if timing_mode == TimingMode.SYNC:
            _last_offset = trajectory[-1].time
        else:
            _last_offset = 0
            _last_end_time = effective_offset + trajectory[-1].time

    del trajectory
    del robot
    del visualizer


def continue_after_sync():
    global _last_end_time, _last_offset

    effective_offset = _last_end_time + _last_offset

    _last_offset = 0
    _last_end_time = effective_offset


def log_trajectory_path(
    motion_id: str, trajectory: list[models.TrajectorySample], motion_group: str
):
    if not all(p.tcp_pose is not None for p in trajectory):
        raise ValueError("All trajectory points must have a tcp_pose")

    points = [
        [p.tcp_pose.position.x, p.tcp_pose.position.y, p.tcp_pose.position.z]
        for p in trajectory
        if p.tcp_pose and p.tcp_pose.position
    ]
    rr.log(
        f"motion/{motion_group}/trajectory",
        rr.LineStrips3D([points], colors=[[1.0, 1.0, 1.0, 1.0]]),
    )

    rr.log("logs/motion", rr.TextLog(f"{motion_group}/{motion_id}", level=rr.TextLogLevel.INFO))


def get_times_column(
    trajectory: list[models.TrajectorySample], timer_offset: float = 0
) -> rr.TimeSecondsColumn:
    times = np.array([timer_offset + point.time for point in trajectory])
    times_column = rr.TimeSecondsColumn(TIME_INTERVAL_NAME, times)
    return times_column


def log_trajectory(
    motion_id: str,
    motion_group: str,
    robot: DHRobot,
    visualizer: RobotVisualizer,
    trajectory: list[models.TrajectorySample],
    optimizer_config: models.OptimizerSetup,
    timer_offset: float,
    tool_asset: Optional[str] = None,
):
    """
    Process a single trajectory point and log relevant data.
    """
    rr.reset_time()
    rr.set_time_seconds(TIME_INTERVAL_NAME, timer_offset)

    times_column = get_times_column(trajectory, timer_offset)

    log_trajectory_path(motion_id, trajectory, motion_group)

    # Calculate and log joint positions
    line_segments_batch = []
    for point in trajectory:
        joint_positions = robot.calculate_joint_positions(point.joint_position)
        line_segments_batch.append(joint_positions)

    rr.send_columns(
        f"motion/{motion_group}/dh_parameters",
        indexes=[times_column],
        columns=[
            *rr.LineStrips3D.columns(
                strips=line_segments_batch, colors=[0.5, 0.5, 0.5, 1.0] * len(line_segments_batch)
            )
        ],
    )

    # Log the robot geometries
    visualizer.log_robot_geometries(trajectory, times_column)

    # Log TCP pose/orientation
    log_tcp_pose(trajectory, motion_group, times_column, tool_asset)

    # Log joint data
    log_joint_data(trajectory, motion_group, times_column, optimizer_config)

    # Log scalar data
    log_scalar_values(trajectory, motion_group, times_column, optimizer_config)


def log_tcp_pose(
    trajectory: list[models.TrajectorySample],
    motion_group,
    times_column,
    tool_asset: Optional[str] = None,
):
    """
    Log TCP pose (position + orientation) data.
    """

    # Extract positions and orientations from the trajectory
    poses = [t.tcp_pose for t in trajectory]
    positions = [[p.position.x, p.position.y, p.position.z] for p in poses]
    orientations = Rotation.from_rotvec(
        [[p.orientation.x, p.orientation.y, p.orientation.z] for p in poses]
    ).as_quat()

    # Log TCP and tool asset
    tcp_entity_path = f"/motion/{motion_group}/tcp_position"
    rr.log(tcp_entity_path, rr.Transform3D(clear=False, axis_length=100))
    if tool_asset:
        rr.log(tcp_entity_path, rr.Asset3D(path=tool_asset), static=True)
    rr.send_columns(
        tcp_entity_path,
        indexes=[times_column],
        columns=rr.Transform3D.columns(translation=positions, quaternion=orientations),
    )


def log_joint_data(
    trajectory: list[models.TrajectorySample],
    motion_group,
    times_column,
    optimizer_config: models.OptimizerSetup,
) -> None:
    """
    Log joint-related data (position, velocity, acceleration, torques) from a trajectory as columns.
    """
    # Initialize lists for each joint and each data type
    if optimizer_config.dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    num_joints = len(optimizer_config.dh_parameters)
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
                -optimizer_config.safety_setup.global_limits.joint_velocity_limits[i]
            )
            joint_data["velocity_upper_limit"][i].append(
                optimizer_config.safety_setup.global_limits.joint_velocity_limits[i]
            )
            joint_data["acceleration_lower_limit"][i].append(
                -optimizer_config.safety_setup.global_limits.joint_acceleration_limits[i]
            )
            joint_data["acceleration_upper_limit"][i].append(
                optimizer_config.safety_setup.global_limits.joint_acceleration_limits[i]
            )
            joint_data["position_lower_limit"][i].append(
                optimizer_config.safety_setup.global_limits.joint_position_limits[i].lower_limit
            )
            joint_data["position_upper_limit"][i].append(
                optimizer_config.safety_setup.global_limits.joint_position_limits[i].upper_limit
            )
            if point.joint_torques and len(point.joint_torques.joints) > i:
                joint_data["torque_limit"][i].append(
                    optimizer_config.safety_setup.global_limits.joint_torque_limits[i]
                )

    # Send columns if data is not empty
    for data_type, data in joint_data.items():
        for i in range(num_joints):
            if data[i]:
                rr.send_columns(
                    f"motion/{motion_group}/joint_{data_type}_{i + 1}",
                    indexes=[times_column],
                    columns=[*rr.Scalar.columns(scalar=data[i])],
                )


def log_scalar_values(
    trajectory: list[models.TrajectorySample],
    motion_group,
    times_column,
    optimizer_config: models.OptimizerSetup,
):
    """
    Log scalar values such as TCP velocity, acceleration, orientation velocity/acceleration, time, and location.
    """
    scalar_data: dict[str, list[float]] = {
        "tcp_velocity": [],
        "tcp_acceleration": [],
        "tcp_orientation_velocity": [],
        "tcp_orientation_acceleration": [],
        "time": [],
        "location_on_trajectory": [],
        "tcp_velocity_limit": [],
        "tcp_orientation_velocity_lower_limit": [],
        "tcp_orientation_velocity_upper_limit": [],
        "tcp_acceleration_lower_limit": [],
        "tcp_acceleration_upper_limit": [],
        "tcp_orientation_acceleration_lower_limit": [],
        "tcp_orientation_acceleration_upper_limit": [],
    }

    # Collect data from the trajectory
    for point in trajectory:
        if point.tcp_velocity is not None:
            scalar_data["tcp_velocity"].append(point.tcp_velocity)
        if point.tcp_acceleration is not None:
            scalar_data["tcp_acceleration"].append(point.tcp_acceleration)
        if point.tcp_orientation_velocity is not None:
            scalar_data["tcp_orientation_velocity"].append(point.tcp_orientation_velocity)
        if point.tcp_orientation_acceleration is not None:
            scalar_data["tcp_orientation_acceleration"].append(point.tcp_orientation_acceleration)
        if point.time is not None:
            scalar_data["time"].append(point.time)
        if point.location_on_trajectory is not None:
            scalar_data["location_on_trajectory"].append(point.location_on_trajectory)
        if optimizer_config.safety_setup.global_limits.tcp_velocity_limit is not None:
            scalar_data["tcp_velocity_limit"].append(
                optimizer_config.safety_setup.global_limits.tcp_velocity_limit
            )
        if optimizer_config.safety_setup.global_limits.tcp_orientation_velocity_limit is not None:
            scalar_data["tcp_orientation_velocity_lower_limit"].append(
                -optimizer_config.safety_setup.global_limits.tcp_orientation_velocity_limit
            )
            scalar_data["tcp_orientation_velocity_upper_limit"].append(
                optimizer_config.safety_setup.global_limits.tcp_orientation_velocity_limit
            )
        if optimizer_config.safety_setup.global_limits.tcp_acceleration_limit is not None:
            scalar_data["tcp_acceleration_lower_limit"].append(
                -optimizer_config.safety_setup.global_limits.tcp_acceleration_limit
            )
            scalar_data["tcp_acceleration_upper_limit"].append(
                optimizer_config.safety_setup.global_limits.tcp_acceleration_limit
            )
        if (
            optimizer_config.safety_setup.global_limits.tcp_orientation_acceleration_limit
            is not None
        ):
            scalar_data["tcp_orientation_acceleration_lower_limit"].append(
                -optimizer_config.safety_setup.global_limits.tcp_orientation_acceleration_limit
            )
            scalar_data["tcp_orientation_acceleration_upper_limit"].append(
                optimizer_config.safety_setup.global_limits.tcp_orientation_acceleration_limit
            )

    # Send columns if data is not empty
    for key, values in scalar_data.items():
        if values:
            rr.send_columns(
                f"motion/{motion_group}/{key}",
                indexes=[times_column],
                columns=[*rr.Scalar.columns(scalar=values)],
            )


def to_trajectory_samples(self) -> list[models.TrajectorySample]:
    """Convert JointTrajectory to list of TrajectorySample objects."""
    samples = []
    for joint_pos, time, location in zip(self.joint_positions, self.times, self.locations):
        sample = models.TrajectorySample(
            joint_position=joint_pos, time=time, location_on_trajectory=location
        )
        samples.append(sample)
    return samples
