from nova import api
from nova.types.motion_settings import MotionSettings


def _patched_joint_limits(
    joint_limits: list[api.models.JointLimits] | None,
    override_with_joint_limits: list[api.models.JointLimits] | None,
) -> list[api.models.JointLimits] | None:
    """Merge joint limits from settings into existing joint limits.

    Args:
        joint_limits: Current joint limits from motion group setup
        override_with_joint_limits: New joint limits from motion settings

    Returns:
        Merged joint limits list

    Raises:
        ValueError: If joint counts don't match
    """
    if override_with_joint_limits is None:
        return joint_limits

    if joint_limits is None:
        return override_with_joint_limits

    if len(joint_limits) != len(override_with_joint_limits):
        raise ValueError(
            f"Joint count mismatch: existing setup has {len(joint_limits)} joints, "
            f"but settings specify {len(override_with_joint_limits)} joints"
        )

    for existing_joint, settings_joint in zip(joint_limits, override_with_joint_limits):
        existing_joint.position = (
            settings_joint.position
            if settings_joint.position is not None
            else existing_joint.position
        )
        existing_joint.velocity = (
            settings_joint.velocity
            if settings_joint.velocity is not None
            else existing_joint.velocity
        )
        existing_joint.acceleration = (
            settings_joint.acceleration
            if settings_joint.acceleration is not None
            else existing_joint.acceleration
        )
        existing_joint.torque = (
            settings_joint.torque if settings_joint.torque is not None else existing_joint.torque
        )

    return joint_limits


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

    motion_group_setup.global_limits.joints = _patched_joint_limits(
        motion_group_setup.global_limits.joints, settings.as_joint_limits()
    )
