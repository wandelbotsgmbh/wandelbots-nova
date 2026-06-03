import uuid
from enum import Enum, auto
from typing import Optional

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation as R

from nova import Nova, MotionGroup, api
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
    safety_collision_setup = await motion_group.get_safety_collision_setup(tcp)

    motion_group_description = await motion_group.get_description()

    dh_parameters = motion_group_description.dh_parameters
    if dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    tcp_offset = motion_group_setup.tcp_offset or api.models.Pose(
        position=api.models.Vector3d([0, 0, 0]),
        orientation=api.models.RotationVector([0, 0, 0]),
    )

    await _log_motion(
        trajectory=trajectory,
        tcp=tcp_offset,
        motion_group_id=motion_group.id,
        motion_group_setup=motion_group_setup,
        dh_parameters=dh_parameters,
        collision_setups=collision_setups,
        safety_collision_setup=safety_collision_setup,
        time_offset=time_offset,
        tool_asset=tool_asset,
        show_collision_link_chain=show_collision_link_chain,
        show_collision_tool=show_collision_tool,
        show_safety_link_chain=show_safety_link_chain,
    )


async def _log_motion(
    trajectory: api.models.JointTrajectory,
    tcp: api.models.Pose,
    motion_group_id: str,
    motion_group_setup: api.models.MotionGroupSetup,
    dh_parameters:  list[api.models.DHParameter],
    collision_setups: dict[str, api.models.CollisionSetup],
    safety_collision_setup: api.models.CollisionSetup,
    time_offset: float = 0,
    tool_asset: str | None = None,
    show_collision_link_chain: bool = False,
    show_collision_tool: bool = True,
    show_safety_link_chain: bool = True,
):
    """Process a single motion for visualization from already-resolved motion group data.

    Args:
        trajectory: Joint trajectory to log.
        tcp: TCP offset pose (flange -> TCP).
        motion_group_id: Motion group identifier used for entity paths/caching.
        motion_group_setup: Resolved motion group setup (mounting, model name, etc.).
        dh_parameters: DH parameters for the motion group's kinematic model.
        collision_setups: Collision setups to visualize (layer name -> setup).
        safety_collision_setup: Collision setup providing safety link-chain/tool geometry.
        time_offset: Time offset for visualization.
        tool_asset: Optional tool asset file path.
        show_collision_link_chain: Whether to show collision geometry.
        show_collision_tool: Whether to show collision tool geometry.
        show_safety_link_chain: Whether to show safety geometry.
    """
    motion_id = str(uuid.uuid4())

    if dh_parameters is not None:
        dh_parameters[0].a = (
            dh_parameters[0].a or 0
        )
        dh_parameters[0].d = (
            dh_parameters[0].d or 0
        )
        dh_parameters[0].alpha = (
            dh_parameters[0].alpha or 0
        )
        dh_parameters[0].theta = (
            dh_parameters[0].theta or 0
        )

        dh_parameters[1].a = (
            dh_parameters[1].a or 0
        )
        dh_parameters[1].d = (
            dh_parameters[1].d or 0
        )
        dh_parameters[1].alpha = (
            dh_parameters[1].alpha or 0
        )
        dh_parameters[1].theta = (
            dh_parameters[1].theta or 0
        )

    if dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    mounting = motion_group_setup.mounting or api.models.Pose(
        position=api.models.Vector3d([0, 0, 0]), orientation=api.models.RotationVector([0, 0, 0])
    )
    robot = DHRobot(dh_parameters=dh_parameters, mounting=mounting)

    rr.reset_time()
    rr.set_time(TIME_INTERVAL_NAME, duration=time_offset)

    # Get or create visualizer from cache
    if motion_group_id not in _visualizer_cache:
        # TODO: merge collision_setups
        collision_link_chain, collision_tcp = extract_link_chain_and_tcp(
            collision_setups=collision_setups
        )

        safety_link_chain, safety_tcp_geometry = extract_link_chain_and_tcp(
            collision_setups={"safety":safety_collision_setup}
        )

        _visualizer_cache[motion_group_id] = RobotVisualizer(
            robot=robot,
            robot_model_geometries=[safety_link_chain] if safety_link_chain else [],
            tcp_geometries=safety_tcp_geometry.root if safety_tcp_geometry else {},
            static_transform=False,
            base_entity_path=f"motion/{motion_group_id}",
            motion_group_model=motion_group_setup.motion_group_model.root,
            collision_link_chain=collision_link_chain,
            collision_tcp=collision_tcp,
            show_collision_link_chain=show_collision_link_chain,
            show_collision_tool=show_collision_tool,
            show_safety_link_chain=show_safety_link_chain,
        )

    visualizer = _visualizer_cache[motion_group_id]

    # Process trajectory points
    await log_trajectory(
        motion_id=motion_id,
        trajectory=trajectory,
        tcp=Pose(tcp),
        motion_group_id=motion_group_id,
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
    tcp: Pose,
    motion_group_id: str,
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

    # TODO: calculate tcp pose from joint positions
    tcp_poses = [visualizer.compute_flange_pose(joints.root) @ tcp for joints in trajectory.joint_positions]
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


async def log_multi_motion_group_trajectory(
    nova: Nova,
    trajectory: api.models.MultiJointTrajectory,
    motion_group_setups: api.models.MotionGroupSetupDictionary,
    collision_setups: api.models.MultiCollisionSetupDictionary | None = None,
):
    """Visualize a multi motion group collision-free trajectory.

    Iterates over motion groups in ``motion_group_setups`` and logs each group's
    joint trajectory (from ``trajectory``) using resolved kinematic data.

    Args:
        nova: Nova client used to resolve kinematic models.
        trajectory: Multi motion-group joint trajectory with shared timestamps.
        motion_group_setups: Motion group setups keyed by motion group id.
        collision_setups: Optional collision setups used during planning.
    """

    joint_positions_by_key = trajectory.joint_positions_by_motion_group_key.root

    # Cache resolved kinematic models per motion group model to avoid duplicate calls.
    dh_parameters_cache: dict[str, list[api.models.DHParameter]] = {}

    for motion_group_id, motion_group_setup in motion_group_setups.root.items():
        joint_positions = joint_positions_by_key.get(motion_group_id)
        if joint_positions is None:
            continue

        # Resolve DH parameters for this motion group's model.
        motion_group_model = motion_group_setup.motion_group_model.root
        dh_parameters = dh_parameters_cache.get(motion_group_model)
        if dh_parameters is None:
            kinematic_model = (
                await nova.api.motion_group_models_api.get_motion_group_kinematic_model(
                    motion_group_model=motion_group_model
                )
            )
            dh_parameters = kinematic_model.dh_parameters
            if dh_parameters is None:
                raise ValueError("DH parameters cannot be None")
            dh_parameters_cache[motion_group_model] = dh_parameters

        # Build a single JointTrajectory from the shared timestamps and this
        # motion group's joint positions.
        joint_trajectory = api.models.JointTrajectory(
            joint_positions=list(joint_positions.root),
            times=trajectory.times,
            locations=trajectory.locations,
        )

        tcp = motion_group_setup.tcp_offset or api.models.Pose(
            position=api.models.Vector3d([0, 0, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        )

        # Use the collision setups configured on the motion group as the safety
        # collision setup (link chain / tool geometry).
        setup_collision_setups: dict[str, api.models.CollisionSetup] = (
            dict(motion_group_setup.collision_setups.root)
            if motion_group_setup.collision_setups is not None
            else {}
        )
        safety_collision_setup = (
            next(iter(setup_collision_setups.values()))
            if setup_collision_setups
            else api.models.CollisionSetup()
        )

        # Build the detailed collision setups for this motion group from the
        # request's collision setups. Each layer contributes this motion group's
        # link-chain/tool colliders plus the static environment colliders.
        collision_setups_flattened: dict[str, api.models.CollisionSetup] = {}
        if collision_setups is not None:
            for setup_name, multi_setup in collision_setups.root.items():
                collision_motion_groups = (
                    multi_setup.collision_motion_groups_by_motion_group_key.root
                    if multi_setup.collision_motion_groups_by_motion_group_key is not None
                    else {}
                )
                collision_motion_group = collision_motion_groups.get(motion_group_id)
                collision_setups_flattened[f"{setup_name}_{motion_group_id}"] = api.models.CollisionSetup(
                    link_chain=collision_motion_group.link_chain
                    if collision_motion_group is not None
                    else None,
                    tool=collision_motion_group.tool
                    if collision_motion_group is not None
                    else None,
                    colliders=multi_setup.colliders,
                )

        await _log_motion(
            trajectory=joint_trajectory,
            tcp=tcp,
            motion_group_id=motion_group_id,
            motion_group_setup=motion_group_setup,
            dh_parameters=dh_parameters,
            collision_setups=collision_setups_flattened,
            safety_collision_setup=safety_collision_setup,
            time_offset=0,
            show_safety_link_chain=True,
            show_collision_link_chain=True,
        )