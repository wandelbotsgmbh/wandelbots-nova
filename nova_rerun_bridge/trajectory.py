import uuid
from enum import Enum, auto
from typing import Any

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation as R

from nova import MotionGroup, api
from nova.types import Pose
from nova_rerun_bridge.consts import TIME_INTERVAL_NAME
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.robot_visualizer import RobotVisualizer
from nova_rerun_bridge.urdf_visualizer import UrdfRobotVisualizer


class TimingMode(Enum):
    """Where a trajectory starts on the timeline, relative to the ones before
    it. Each motion group keeps its own clock.
    """

    RESET = auto()
    """Start at ``time_offset``; the clock carries on from there."""
    CONTINUE = auto()
    """Start after this motion group's last trajectory. The default."""
    SYNC = auto()
    """Start at ``time_offset`` and leave the clock alone, so several motion
    groups can be lined up at one instant without moving anyone on."""
    OVERRIDE = auto()
    """Start at ``time_offset``; the clock carries on from there. Same rule as
    :attr:`RESET`; both spellings exist."""


class MotionGroupTimeline:
    """Each motion group's own clock on the shared timeline."""

    def __init__(self) -> None:
        self.times: dict[str, float] = {}

    def start(self, motion_group_id: str, mode: TimingMode, time_offset: float) -> float:
        """When a trajectory logged now starts."""
        if mode is TimingMode.CONTINUE:
            return self.times.get(motion_group_id, 0.0) + time_offset
        return time_offset

    def advance(
        self, motion_group_id: str, start: float, duration: float, mode: TimingMode
    ) -> None:
        """Move the clock past a trajectory that starts at *start*."""
        if mode is TimingMode.SYNC:
            return
        self.times[motion_group_id] = start + duration


_timeline = MotionGroupTimeline()
"""The clock a caller gets when it does not bring its own."""

# State of the deprecated :func:`continue_after_sync`.
_last_end_time = 0.0
_last_offset = 0.0

_visualizer_cache: dict[str, RobotVisualizer] = {}


async def log_motion(
    trajectory: api.models.JointTrajectory,
    tcp: str,
    motion_group: MotionGroup,
    collision_setups: dict[str, api.models.CollisionSetup],
    time_offset: float = 0,
    timing_mode: TimingMode = TimingMode.CONTINUE,
    tool_asset: str | None = None,
    tool_assets: dict[str, str] | None = None,
    timeline: MotionGroupTimeline | None = None,
    show_safety_link_chain: bool = True,
    show_collision: bool | None = False,
    show_collision_link_chain: bool | None = None,
    show_collision_tool: bool | None = None,
):
    """
    Fetch and process a single motion for visualization.

    Args:
        trajectory: Joint trajectory to log
        tcp: TCP to log
        motion_group: Motion group to log
        collision_setups: Accepted and unused. The robot's collision geometry
            comes from the URDF, and a plan's collision objects are logged by
            the viewer through
            :func:`~nova_rerun_bridge.collision_scene.log_collision_setups`
        time_offset: Where this trajectory starts, read as the mode says
        timing_mode: How to place it against what was logged before
        timeline: The motion group clocks to read and advance; a caller that
            logs through :class:`~nova_rerun_bridge.NovaRerunBridge` shares its
            one, so the bridge's own accessors see the same times
        tool_asset: Tool mesh for *tcp*, if the caller has one
        tool_assets: Tool meshes per TCP, for a robot with more than one tool.
            Every TCP is its own frame in the URDF, so each mesh rides the arm
            on the TCP it belongs to. Takes precedence over *tool_asset*.
        show_safety_link_chain: Whether to show safety geometry
        show_collision: Whether to draw the URDF's collision geometry,
            see-through; None only where a link has no visual mesh
        show_collision_link_chain: Deprecated, use show_collision
        show_collision_tool: Deprecated, use show_collision
    """
    clock = timeline if timeline is not None else _timeline
    time_offset = clock.start(motion_group.id, timing_mode, time_offset)

    if show_collision_link_chain is not None or show_collision_tool is not None:
        import warnings

        warnings.warn(
            "show_collision_link_chain and show_collision_tool are deprecated; "
            "use show_collision, which covers both.",
            DeprecationWarning,
            stacklevel=2,
        )
        if show_collision is False:
            show_collision = bool(show_collision_link_chain or show_collision_tool)

    motion_group_setup = await motion_group.get_setup(tcp)
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
        position=(0, 0, 0), orientation=(0, 0, 0)
    )
    robot = DHRobot(dh_parameters=motion_group_description.dh_parameters, mounting=mounting)

    rr.reset_time()
    rr.set_time(TIME_INTERVAL_NAME, duration=time_offset)

    # Get or create visualizer from cache
    if motion_group.id not in _visualizer_cache:
        # Build tcp geometries
        tcp_geometries: dict[str, api.models.Collider] = {}
        if motion_group_description.safety_tool_colliders is not None:
            tool_colliders = motion_group_description.safety_tool_colliders.get(tcp)
            if tool_colliders is not None:
                tcp_geometries = dict(tool_colliders)

        # Build safety link chain
        safety_link_chain: list[Any] = []
        if motion_group_description.safety_link_colliders is not None:
            safety_link_chain = [list(motion_group_description.safety_link_colliders)]

        # Exported onto the TCP's frame, so the tool rides the arm.
        tools = tool_assets or ({tcp: tool_asset} if tool_asset else None)
        urdf = await UrdfRobotVisualizer.for_motion_group(motion_group, tool_assets=tools)
        if urdf is not None:
            # Which tool is mounted is not in the URDF; it is in this call.
            urdf.active_tcp = tcp
        _visualizer_cache[motion_group.id] = RobotVisualizer(
            urdf=urdf,
            robot=robot,
            robot_motion_group_id=motion_group.id,
            robot_model_geometries=safety_link_chain,
            tcp_geometries=tcp_geometries,
            static_transform=False,
            base_entity_path=f"motion/{motion_group_id}",
            show_safety_link_chain=show_safety_link_chain,
            show_collision=show_collision,
        )

    visualizer = _visualizer_cache[motion_group.id]
    if visualizer.urdf is not None:
        # A later motion may run a different tool; the safety volumes follow it.
        visualizer.urdf.active_tcp = tcp

    # Process trajectory points
    await log_trajectory(
        motion_id=motion_id,
        trajectory=trajectory,
        tcp=tcp,
        motion_group=motion_group,
        robot=robot,
        visualizer=visualizer,
        timer_offset=time_offset,
    )

    times = getattr(trajectory, "times", None)
    clock.advance(motion_group.id, time_offset, times[-1] if times else 0.0, timing_mode)

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
    tool_asset: str | None = None,
):
    """
    Process a single trajectory point and log relevant data.

    *tool_asset* is accepted and ignored here: a tool mesh belongs on its TCP's
    frame in the URDF, which is what makes it ride the robot. Pass it to
    :func:`log_motion`, which exports it there.
    """
    rr.reset_time()
    rr.set_time(TIME_INTERVAL_NAME, duration=timer_offset)

    times_column = get_times_column(trajectory, timer_offset)
    motion_group_id = motion_group.id

    # TODO: calculate tcp pose from joint positions
    joint_positions = [tuple(p) for p in trajectory.joint_positions]
    tcp_poses = await motion_group.forward_kinematics(joints=joint_positions, tcp=tcp)

    # Overlays go in the robot's frame: a reported pose is relative to what the
    # motion group is mounted on, and an entity outside the robot's frame graph
    # is not drawn at all.
    anchor = visualizer.world_frame
    pose = {motion_group_id: list(joint_positions[0])} if joint_positions else {}
    # A reported pose and a DH position start in different places: the first is
    # relative to what the motion group is mounted on, the second at the chain's
    # own base with the mounting the DH robot was given already applied.
    to_robot = visualizer.pose_frames(pose).get(motion_group_id)
    chain_base = visualizer.chain_bases(pose).get(motion_group_id)
    to_robot_dh = (
        chain_base @ np.linalg.inv(robot.pose_to_matrix(robot.mounting))
        if chain_base is not None
        else None
    )

    def place(points: list, transform) -> list:
        """Positions as the robot's own frame sees them."""
        if transform is None:
            return points
        placed = np.asarray(points, dtype=float) @ transform[:3, :3].T + transform[:3, 3]
        return placed.tolist()

    def in_robot_frame(points: list) -> list:
        """A reported pose, in the robot's frame."""
        return place(points, to_robot)

    def anchor_to_robot(entity_path: str) -> None:
        """Join the robot's frame graph, so the view keeps this entity."""
        if anchor:
            rr.log(entity_path, rr.CoordinateFrame(frame=anchor), static=True)

    positions = in_robot_frame([[p.position.x, p.position.y, p.position.z] for p in tcp_poses])

    anchor_to_robot(f"motion/{motion_group_id}/trajectory")
    rr.log(
        f"motion/{motion_group_id}/trajectory",
        rr.LineStrips3D([positions], colors=[[1.0, 1.0, 1.0, 1.0]]),
    )

    rr.log("logs/motion", rr.TextLog(f"{motion_group_id}/{motion_id}", level=rr.TextLogLevel.INFO))

    # Calculate and log joint positions
    line_segments_batch = []
    for joint_position in trajectory.joint_positions:
        robot_joint_positions = robot.calculate_joint_positions(joint_positions=joint_position)
        line_segments_batch.append([place(robot_joint_positions, to_robot_dh)])

    anchor_to_robot(f"motion/{motion_group_id}/dh_parameters")
    rr.send_columns(
        f"motion/{motion_group_id}/dh_parameters",
        indexes=[times_column],
        columns=rr.LineStrips3D.columns(strips=line_segments_batch),
    )
    rr.log(
        f"motion/{motion_group_id}/dh_parameters",
        rr.LineStrips3D.from_fields(clear_unset=True, colors=[0.5, 0.5, 0.5, 1.0]),
    )

    # Log the robot geometries. Naming the motion group matters for a robot the
    # API splits across several: the URDF holds every chain.
    visualizer.log_robot_geometries(
        trajectory=trajectory, times_column=times_column, motion_group_id=motion_group.id
    )

    # Log TCP pose/orientation
    log_tcp_pose(
        tcp_poses=tcp_poses,
        motion_group_id=motion_group_id,
        times_column=times_column,
        positions=positions,
        frame=anchor,
    )


def log_tcp_pose(
    tcp_poses: list[Pose],
    motion_group_id: str,
    times_column,
    tool_asset: str | None = None,
    positions: list | None = None,
    frame: str | None = None,
):
    """
    Log TCP pose (position + orientation) data.

    *positions* are the same poses already placed in the robot's frame, and
    *frame* is that frame; see :func:`log_trajectory` for why both are needed.
    *tool_asset* is accepted and ignored: a tool's mesh belongs in the URDF, on
    the TCP's own frame.
    """
    # Handle empty trajectory
    if not tcp_poses:
        return

    # Extract positions and orientations from the trajectory
    if positions is None:
        positions = [p.position.to_tuple() for p in tcp_poses]
    orientations = R.from_rotvec([p.orientation.to_tuple() for p in tcp_poses]).as_quat()

    # Log the TCP frame. The tool's own mesh is not drawn here: it is exported
    # onto the TCP's frame in the URDF, which puts it on the robot.
    tcp_entity_path = f"/motion/{motion_group_id}/tcp_position"
    if frame:
        rr.log(tcp_entity_path, rr.CoordinateFrame(frame=frame), static=True)
    rr.log(tcp_entity_path, rr.TransformAxes3D(axis_length=100))

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
        "continue_after_sync() is deprecated and will be removed in a future "
        "version: each motion group carries its own clock.",
        DeprecationWarning,
        stacklevel=2,
    )

    global _last_end_time, _last_offset
    effective_offset = _last_end_time + _last_offset
    _last_offset = 0
    _last_end_time = effective_offset
