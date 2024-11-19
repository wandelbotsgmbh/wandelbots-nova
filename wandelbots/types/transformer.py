# pylint: disable=no-name-in-module
import math
from functools import singledispatch

import wandelbots_api_client as wb
from wandelbots.types.motion import MotionSettings, Motion, Linear, Circular, PTP, JointPTP, Spline
from wandelbots.types.trajectory import MotionTrajectory


def motion_settings_to_rae_command_settings(motion_settings: MotionSettings) -> wb.models.CommandSettings:
    """Converts motion settings to RAE command settings

    Args:
        motion_settings: the motion parameters

    Returns: corresponding RAE command settings

    """

    command_settings = wb.models.CommandSettings()
    # TODO: betterproto does not generate this field as optional
    if motion_settings.blending == math.inf:
        command_settings.auto_blending = 100
    elif motion_settings.blending > 0:
        command_settings.position_blending = motion_settings.blending
    else:
        command_settings.position_blending = 0

    limits_override = wb.models.LimitsOverride()
    # cartesian space
    if motion_settings.velocity is not None:
        limits_override.tcp_velocity_limit = motion_settings.velocity
    if motion_settings.acceleration is not None:
        limits_override.tcp_acceleration_limit = motion_settings.acceleration
    if motion_settings.orientation_velocity is not None:
        limits_override.tcp_orientation_velocity_limit = motion_settings.orientation_velocity
    if motion_settings.orientation_acceleration is not None:
        limits_override.tcp_orientation_acceleration_limit = motion_settings.orientation_acceleration

    # joint space
    if motion_settings.joint_velocities is not None:
        limits_override.joint_velocity_limits = wb.models.Joints(joints=list(motion_settings.joint_velocities))
    if motion_settings.joint_accelerations is not None:
        limits_override.joint_acceleration_limits = wb.models.Joints(joints=list(motion_settings.joint_accelerations))

    command_settings.limits_override = limits_override

    return command_settings


# TODO: make optimize_approach setable in WS
def motion_trajectory_to_rae_plan_request(
    motion_group: str, start_joint_position: list[float], path: MotionTrajectory, tcp: str
) -> wb.models.PlanRequest:
    """Construct a RAE PathMotionCommand from a motion trajectory

    Segments are mapped to the according motion type. Note that points are mapped to point-to-point motions.
    If path does not start with a point, the motion chain starts nevertheless with a point-to-point motion.

    Option 1:
    P2P (0): * - A
    -> [A]

    Option 2:
    P2P (0): A - B
    -> [A, B]

    Option 3:
    LIN (0): A - B
    LIN (1): B - C
    CIR (2): C - D - E

    Args:
        motion_group: the motion group that the path belongs to
        start_joint_position: the start joint position
        path: the given motion trajectory
        tcp: the tool center point

    Returns:
        The transformed RAE paths
    """
    # edge case: the path has 1 segment with only one motion
    if len(path.motions) == 1:
        path.start.settings.blending = 0
        commands = [motion_to_rae_command(path.motions[0])]
    else:
        path.end.settings.blending = 0
        commands = [motion_to_rae_command(motion) for motion in path.motions[:-1]] + [
            motion_to_rae_command(path.motions[-1])
        ]

    return wb.models.PlanRequest(
        motion_group=motion_group,
        start_joint_position=wb.models.Joints(joints=start_joint_position),
        commands=commands,
        tcp=tcp,
    )


@singledispatch
def motion_to_rae_command(motion: Motion) -> wb.models.Command:
    """Converts a motion to a RAE path"""
    raise NotImplementedError(type(motion))


@motion_to_rae_command.register
def _(motion: Linear):
    return wb.models.Command(settings=motion_settings_to_rae_command_settings(motion.settings), line=motion.target)


@motion_to_rae_command.register
def _(motion: PTP):
    return wb.models.Command(
        settings=motion_settings_to_rae_command_settings(motion.settings), cartesian_ptp=motion.target
    )


@motion_to_rae_command.register
def _(motion: Circular):
    # TODO: return MoveCommand
    return wb.models.Command(
        settings=motion_settings_to_rae_command_settings(motion.settings),
        circle=wb.models.Circle(via_pose=motion.intermediate, target_pose=motion.target),
    )


@motion_to_rae_command.register
def _(motion: JointPTP):
    return wb.models.Command(
        settings=motion_settings_to_rae_command_settings(motion.settings),
        joint_ptp=wb.models.Joints(joints=list(motion.target)),
    )


@motion_to_rae_command.register
def _(motion: Spline):
    raise NotImplementedError("Spline object is broken -> has only one way point. Therefore, not implemented yet")
