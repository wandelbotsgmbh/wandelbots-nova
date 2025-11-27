from nova import api
from nova.types.motion_settings import MotionSettings


def update_motion_group_setup_with_motion_settings(
    motion_group_setup: api.models.MotionGroupSetup, settings: MotionSettings
):
    """Update motion group setup with motion settings.

    This function patches the motion group setup with TCP and joint limits from the provided motion settings.

    Args:
        motion_group_setup: The motion group setup to update
        settings: Motion settings containing limits to apply
    """
    tcp_settings = settings.as_tcp_cartesian_limits()

    if motion_group_setup.global_limits is None:
        motion_group_setup.global_limits = api.models.LimitSet()

    if motion_group_setup.global_limits.tcp is None:
        motion_group_setup.global_limits.tcp = tcp_settings
    else:
        setup_settings = motion_group_setup.global_limits.tcp
        # do patching
        motion_group_setup.global_limits.tcp.velocity = (
            settings.tcp_velocity_limit
            if settings.tcp_velocity_limit is not None
            else setup_settings.velocity
        )
        motion_group_setup.global_limits.tcp.acceleration = (
            settings.tcp_acceleration_limit
            if settings.tcp_acceleration_limit is not None
            else setup_settings.acceleration
        )
        motion_group_setup.global_limits.tcp.orientation_velocity = (
            settings.tcp_orientation_velocity_limit
            if settings.tcp_orientation_velocity_limit is not None
            else setup_settings.orientation_velocity
        )
        motion_group_setup.global_limits.tcp.orientation_acceleration = (
            settings.tcp_orientation_acceleration_limit
            if settings.tcp_orientation_acceleration_limit is not None
            else setup_settings.orientation_acceleration
        )

    joint_limits_from_settings = settings.as_joint_limits()
    if joint_limits_from_settings is not None:
        motion_group_setup.global_limits.joints = joint_limits_from_settings
