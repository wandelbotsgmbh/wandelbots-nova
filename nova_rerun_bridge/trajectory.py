import uuid
from enum import Enum, auto
from typing import Optional

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation as R

from nova import MotionGroup, api
from nova.types import Pose
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
        collision_setups: Collision setups to log [setup_name: collision_setup]
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
    motion_group_model = await motion_group.get_model()
    motion_group_description = await motion_group.get_description()
    motion_group_id = motion_group.id
    motion_id = str(uuid.uuid4())

    if motion_group_description.dh_parameters is not None:
        motion_group_description.dh_parameters[0].a = (
            motion_group_description.dh_parameters[0].a or 0
        )
        motion_group_description.dh_parameters[0].d = (
            motion_group_description.dh_parameters[0].d or 0
        )
        motion_group_description.dh_parameters[0].alpha = (
            motion_group_description.dh_parameters[0].alpha or 0
        )
        motion_group_description.dh_parameters[0].theta = (
            motion_group_description.dh_parameters[0].theta or 0
        )

        motion_group_description.dh_parameters[1].a = (
            motion_group_description.dh_parameters[1].a or 0
        )
        motion_group_description.dh_parameters[1].d = (
            motion_group_description.dh_parameters[1].d or 0
        )
        motion_group_description.dh_parameters[1].alpha = (
            motion_group_description.dh_parameters[1].alpha or 0
        )
        motion_group_description.dh_parameters[1].theta = (
            motion_group_description.dh_parameters[1].theta or 0
        )

    if motion_group_description.dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    mounting = motion_group_setup.mounting or api.models.Pose(
        position=api.models.Vector3d([0, 0, 0]), orientation=api.models.RotationVector([0, 0, 0])
    )
    robot = DHRobot(dh_parameters=motion_group_description.dh_parameters, mounting=mounting)

    # TODO: merge collision_setups
    collision_link_chain, collision_tcp = extract_link_chain_and_tcp(
        collision_setups=collision_setups
    )

    rr.reset_time()
    rr.set_time(TIME_INTERVAL_NAME, duration=time_offset)

    # Get or create visualizer from cache
    if motion_group.id not in _visualizer_cache:
        collision_link_chain, collision_tcp = extract_link_chain_and_tcp(
            collision_setups=collision_setups
        )

        # Build tcp geometries
        tcp_geometries: list[api.models.Collider] = []
        if motion_group_description.safety_tool_colliders is not None:
            tool_colliders = motion_group_description.safety_tool_colliders.get(tcp)
            if tool_colliders is not None:
                tcp_geometries = [
                    tool_collider for tool_collider in list(tool_colliders.root.values())
                ]

        # Build safety link chain
        safety_link_chain: list[api.models.LinkChain] = []
        if motion_group_description.safety_link_colliders is not None:
            safety_link_chain = [
                api.models.LinkChain(
                    [
                        api.models.Link(link.root)
                        for link in motion_group_description.safety_link_colliders
                    ]
                )
            ]

        _visualizer_cache[motion_group.id] = RobotVisualizer(
            robot=robot,
            robot_model_geometries=safety_link_chain,
            tcp_geometries=tcp_geometries,
            static_transform=False,
            base_entity_path=f"motion/{motion_group_id}",
            motion_group_model=motion_group_model,
            collision_link_chain=collision_link_chain,
            collision_tcp=collision_tcp,
            show_collision_link_chain=show_collision_link_chain,
            show_collision_tool=show_collision_tool,
            show_safety_link_chain=show_safety_link_chain,
        )

    visualizer = _visualizer_cache[motion_group.id]

    # Process trajectory points
    await log_trajectory(
        motion_id=motion_id,
        trajectory=trajectory,
        tcp=tcp,
        motion_group=motion_group,
        robot=robot,
        visualizer=visualizer,
        timer_offset=time_offset,
        tool_asset=tool_asset,
    )

    del trajectory
    del robot
    del visualizer


def get_times_column(
    trajectory: api.models.JointTrajectory, timer_offset: float = 0
) -> rr.TimeColumn:
    times = np.array([timer_offset + time for time in trajectory.times])
    times_column = rr.TimeColumn(TIME_INTERVAL_NAME, duration=times)
    return times_column


async def log_trajectory(
    motion_id: str,
    trajectory: api.models.JointTrajectory,
    tcp: str,
    motion_group: MotionGroup,
    robot: DHRobot,
    visualizer: RobotVisualizer,
    timer_offset: float,
    tool_asset: Optional[str] = None,
):
    """
    Process a single trajectory point and log relevant data.
    """
    rr.reset_time()
    rr.set_time(TIME_INTERVAL_NAME, duration=timer_offset)

    times_column = get_times_column(trajectory, timer_offset)
    motion_group_id = motion_group.id

    # TODO: calculate tcp pose from joint positions
    joint_positions = [tuple(p.root) for p in trajectory.joint_positions]
    tcp_poses = await motion_group.forward_kinematics(joints=joint_positions, tcp=tcp)
    positions = [[p.position.x, p.position.y, p.position.z] for p in tcp_poses]

    rr.log(
        f"motion/{motion_group_id}/trajectory",
        rr.LineStrips3D([positions], colors=[[1.0, 1.0, 1.0, 1.0]]),
    )

    rr.log("logs/motion", rr.TextLog(f"{motion_group_id}/{motion_id}", level=rr.TextLogLevel.INFO))

    # Calculate and log joint positions
    line_segments_batch = []
    for joint_position in trajectory.joint_positions:
        robot_joint_positions = robot.calculate_joint_positions(joint_positions=joint_position.root)
        line_segments_batch.append([robot_joint_positions])  # Wrap each as a line strip

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
    visualizer.log_robot_geometries(trajectory=trajectory, times_column=times_column)

    # Log TCP pose/orientation
    log_tcp_pose(
        tcp_poses=tcp_poses,
        motion_group_id=motion_group_id,
        times_column=times_column,
        tool_asset=tool_asset,
    )


def log_tcp_pose(
    tcp_poses: list[Pose], motion_group_id: str, times_column, tool_asset: str | None = None
):
    """
    Log TCP pose (position + orientation) data.
    """
    # Handle empty trajectory
    if not tcp_poses:
        return

    # Extract positions and orientations from the trajectory
    positions = [p.position.to_tuple() for p in tcp_poses]
    orientations = R.from_rotvec([p.orientation.to_tuple() for p in tcp_poses]).as_quat()

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
