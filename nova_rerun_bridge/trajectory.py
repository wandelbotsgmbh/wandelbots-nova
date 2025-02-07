from enum import Enum, auto
from typing import Dict, List

import numpy as np
import rerun as rr
from nova.api import models
from scipy.spatial.transform import Rotation

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


def log_motion(
    motion_id: str,
    model_from_controller: str,
    motion_group: str,
    optimizer_config: models.OptimizerSetup,
    trajectory: List[models.TrajectorySample],
    collision_scenes: Dict[str, models.CollisionScene],
    time_offset: float = 0,
    timing_mode: TimingMode = TimingMode.CONTINUE,
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
    global _last_end_time, _last_offset

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
        optimizer_config.dh_parameters[0].a = 0
        optimizer_config.dh_parameters[0].d = 360
        optimizer_config.dh_parameters[0].alpha = np.pi / 2
        optimizer_config.dh_parameters[0].theta = 0

        optimizer_config.dh_parameters[1].a = 0
        optimizer_config.dh_parameters[1].d = 0
        optimizer_config.dh_parameters[1].alpha = 0
        optimizer_config.dh_parameters[1].theta = np.pi / 2

    robot = DHRobot(optimizer_config.dh_parameters, optimizer_config.mounting)

    collision_link_chain, collision_tcp = extract_link_chain_and_tcp(collision_scenes)

    visualizer = RobotVisualizer(
        robot=robot,
        robot_model_geometries=optimizer_config.safety_setup.robot_model_geometries,
        tcp_geometries=optimizer_config.safety_setup.tcp_geometries,
        static_transform=False,
        base_entity_path=f"motion/{motion_group}",
        model_from_controller=model_from_controller,
        collision_link_chain=collision_link_chain,
        collision_tcp=collision_tcp,
    )

    rr.set_time_seconds(TIME_INTERVAL_NAME, effective_offset)

    # Process trajectory points
    log_trajectory(
        motion_id=motion_id,
        motion_group=motion_group,
        robot=robot,
        visualizer=visualizer,
        trajectory=trajectory,
        optimizer_config=optimizer_config,
        timer_offset=effective_offset,
    )

    # Update last times based on timing mode
    if trajectory:
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
    motion_id: str, trajectory: List[models.TrajectorySample], motion_group: str
):
    points = [
        [p.tcp_pose.position.x, p.tcp_pose.position.y, p.tcp_pose.position.z] for p in trajectory
    ]
    rr.log(
        f"motion/{motion_group}/trajectory",
        rr.LineStrips3D([points], colors=[[1.0, 1.0, 1.0, 1.0]]),
    )

    rr.log("logs/motion", rr.TextLog(f"{motion_group}/{motion_id}", level=rr.TextLogLevel.INFO))


def get_times_column(
    trajectory: List[models.TrajectorySample], timer_offset: float = 0
) -> rr.TimeSecondsColumn:
    times = np.array([timer_offset + point.time for point in trajectory])
    times_column = rr.TimeSecondsColumn(TIME_INTERVAL_NAME, times)
    return times_column


def log_trajectory(
    motion_id: str,
    motion_group: str,
    robot: DHRobot,
    visualizer: RobotVisualizer,
    trajectory: List[models.TrajectorySample],
    optimizer_config: models.OptimizerSetup,
    timer_offset: float,
):
    """
    Process a single trajectory point and log relevant data.
    """
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
        times=[times_column],
        components=[
            rr.LineStrips3D.indicator(),
            rr.components.LineStrip3DBatch(line_segments_batch),
            rr.components.ColorBatch([0.5, 0.5, 0.5, 1.0] * len(line_segments_batch)),
        ],
    )

    # Log the robot geometries
    visualizer.log_robot_geometries(trajectory, times_column)

    # Log TCP pose/orientation
    log_tcp_pose(trajectory, motion_group, times_column)

    # Log joint data
    log_joint_data(trajectory, motion_group, times_column, optimizer_config)

    # Log scalar data
    log_scalar_values(trajectory, motion_group, times_column, optimizer_config)


def log_tcp_pose(trajectory: List[models.TrajectorySample], motion_group, times_column):
    """
    Log TCP pose (position + orientation) data.
    """
    tcp_positions = []
    tcp_rotations = []

    # Collect data from the trajectory
    for point in trajectory:
        # Collect TCP position
        tcp_positions.append(
            [point.tcp_pose.position.x, point.tcp_pose.position.y, point.tcp_pose.position.z]
        )

        # Convert and collect TCP orientation as axis-angle
        rotation_vector = [
            point.tcp_pose.orientation.x,
            point.tcp_pose.orientation.y,
            point.tcp_pose.orientation.z,
        ]
        rotation = Rotation.from_rotvec(rotation_vector)
        angle = rotation.magnitude()
        axis_angle = rotation.as_rotvec() / angle if angle != 0 else [0, 0, 0]
        tcp_rotations.append(rr.RotationAxisAngle(axis=axis_angle, angle=angle))

    rr.send_columns(
        f"motion/{motion_group}/tcp_position",
        times=[times_column],
        components=[
            rr.Transform3D.indicator(),
            rr.components.Translation3DBatch(tcp_positions),
            rr.components.RotationAxisAngleBatch(tcp_rotations),
        ],
    )


def log_joint_data(
    trajectory: List[models.TrajectorySample],
    motion_group,
    times_column,
    optimizer_config: models.OptimizerSetup,
) -> None:
    """
    Log joint-related data (position, velocity, acceleration, torques) from a trajectory as columns.
    """
    # Initialize lists for each joint and each data type (assuming 6 joints)
    num_joints = len(optimizer_config.dh_parameters)
    joint_data = {
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
                    times=[times_column],
                    components=[rr.components.ScalarBatch(data[i])],
                )


def log_scalar_values(
    trajectory: List[models.TrajectorySample],
    motion_group,
    times_column,
    optimizer_config: models.OptimizerSetup,
):
    """
    Log scalar values such as TCP velocity, acceleration, orientation velocity/acceleration, time, and location.
    """
    scalar_data = {
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
                times=[times_column],
                components=[rr.components.ScalarBatch(values)],
            )


def to_trajectory_samples(self) -> List[models.TrajectorySample]:
    """Convert JointTrajectory to list of TrajectorySample objects."""
    samples = []
    for joint_pos, time, location in zip(self.joint_positions, self.times, self.locations):
        sample = models.TrajectorySample(
            joint_position=joint_pos, time=time, location_on_trajectory=location
        )
        samples.append(sample)
    return samples
