from nova import api
from nova.types.motion_settings import MotionSettings


def _cap(value: float | None, maximum: float | None) -> float | None:
    """Return ``value`` capped at ``maximum``. An unset value or maximum means no change."""
    if value is None or maximum is None:
        return value
    return min(value, maximum)


def clamp_limits_override_to_global_limits(
    limits_override: api.models.LimitsOverride, global_limits: api.models.LimitSet | None
) -> api.models.LimitsOverride:
    """Return a copy of ``limits_override`` capped at the motion group's global max limits.

    A per-segment ``limits_override`` *replaces* the global limit for that segment, so a value
    above the motion group maximum makes the planner produce a trajectory the hardware cannot
    realize - which then gets down-scaled across its whole length at execution time. Capping
    each value at the corresponding global maximum avoids that while leaving lower
    (intentionally slower) values untouched. Unset fields and fields without a global maximum
    are left as-is. The input is not modified.
    """
    if global_limits is None:
        return limits_override

    # Empty fallbacks so a missing tcp/joint maximum simply means "do not cap".
    tcp = global_limits.tcp or api.models.CartesianLimits()
    joints = global_limits.joints or []

    def cap_joints(values: list[float] | None, attr: str) -> list[float] | None:
        if values is None or not joints:
            return values
        return [
            min(v, m) if i < len(joints) and (m := getattr(joints[i], attr)) is not None else v
            for i, v in enumerate(values)
        ]

    return limits_override.model_copy(
        update={
            "tcp_velocity_limit": _cap(limits_override.tcp_velocity_limit, tcp.velocity),
            "tcp_acceleration_limit": _cap(
                limits_override.tcp_acceleration_limit, tcp.acceleration
            ),
            "tcp_jerk_limit": _cap(limits_override.tcp_jerk_limit, tcp.jerk),
            "tcp_orientation_velocity_limit": _cap(
                limits_override.tcp_orientation_velocity_limit, tcp.orientation_velocity
            ),
            "tcp_orientation_acceleration_limit": _cap(
                limits_override.tcp_orientation_acceleration_limit, tcp.orientation_acceleration
            ),
            "tcp_orientation_jerk_limit": _cap(
                limits_override.tcp_orientation_jerk_limit, tcp.orientation_jerk
            ),
            "joint_velocity_limits": cap_joints(limits_override.joint_velocity_limits, "velocity"),
            "joint_acceleration_limits": cap_joints(
                limits_override.joint_acceleration_limits, "acceleration"
            ),
            "joint_jerk_limits": cap_joints(limits_override.joint_jerk_limits, "jerk"),
        }
    )


def clamp_motion_commands_to_global_limits(
    motion_commands: list[api.models.MotionCommand], global_limits: api.models.LimitSet | None
) -> list[api.models.MotionCommand]:
    """Return a new command list with each ``limits_override`` capped at the global maxima.

    Commands without a ``limits_override`` are passed through unchanged. Inputs are not modified.
    """
    if global_limits is None:
        return motion_commands
    return [
        command.model_copy(
            update={
                "limits_override": clamp_limits_override_to_global_limits(
                    command.limits_override, global_limits
                )
            }
        )
        if command.limits_override is not None
        else command
        for command in motion_commands
    ]


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
