"""Build and patch the :class:`MotionGroupSetup` that is sent to the planner.

This module is the single home for everything that fills a ``MotionGroupSetup`` before a
plan request, in particular the robot limits:

- **Controller global (max) limits** come from ``operation_limits.auto_limits`` and are placed on
  ``MotionGroupSetup.global_limits`` by :func:`controller_global_limits` /
  :func:`motion_group_setup_from_motion_group_description`. This is the ceiling the planner treats
  as the maximum for the whole request.
- **Per-motion user limits** (``MotionSettings``) are applied differently depending on the planning
  API used (see the planning methods in ``nova/cell/motion_group.py``):
    - ``plan_trajectory`` (collision-checked path) keeps ``global_limits`` as the controller max and
      sends the user limits as a per-segment ``limits_override``.
    - ``plan_collision_free`` has no per-segment override, so the user limits are folded into
      ``global_limits`` via :func:`update_motion_group_setup_with_motion_settings`.
"""

from nova import api
from nova.types.motion_settings import MotionSettings
from nova.utils.collision_setup import get_safety_collision_setup_from_motion_group_description


def controller_global_limits(
    motion_group_description: api.models.MotionGroupDescription,
) -> api.models.LimitSet:
    """Return the controller's maximum (auto-mode) limits for the motion group.

    This is the single source of truth for the value that ends up on
    ``MotionGroupSetup.global_limits`` — the ceiling the planner uses for a plan request.

    It is assumed the auto limits are always present. We also assume the motion player in RAE
    scales correctly if the planned trajectory is played back with different limits (due to a
    different robot mode) than the one used for planning.
    """
    assert motion_group_description.operation_limits.auto_limits is not None
    return motion_group_description.operation_limits.auto_limits


def motion_group_setup_from_motion_group_description(
    motion_group_description: api.models.MotionGroupDescription,
    tcp_name: str | None = None,
    payload: api.models.Payload | None = None,
) -> api.models.MotionGroupSetup:
    collision_setup = get_safety_collision_setup_from_motion_group_description(
        motion_group_description=motion_group_description, tcp_name=tcp_name
    )
    tcps = motion_group_description.tcps
    tcp_offset = tcps[tcp_name].pose if tcp_name is not None and tcps is not None else None
    return api.models.MotionGroupSetup(
        motion_group_model=motion_group_description.motion_group_model,
        cycle_time=motion_group_description.cycle_time or 8,
        mounting=motion_group_description.mounting,
        global_limits=controller_global_limits(motion_group_description),
        tcp_offset=tcp_offset,
        payload=payload,
        collision_setups=api.models.CollisionSetups({"safety": collision_setup}),
    )


def get_joint_position_limits_from_motion_group_setup(
    motion_group_setup: api.models.MotionGroupSetup,
) -> api.models.JointPositionLimits | None:
    """Extract joint position limits from motion group description, if available."""
    if motion_group_setup.global_limits is None or motion_group_setup.global_limits.joints is None:
        return None

    # TODO: does optional mean no limit applied for that joint?
    # will joint.position is not None cause issues by skipping joints without limits?
    joint_limit_range_list = [
        joint.position
        for joint in motion_group_setup.global_limits.joints
        if joint.position is not None
    ]
    return api.models.JointPositionLimits(root=joint_limit_range_list)


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
) -> api.models.MotionGroupSetup:
    """Return a copy of the motion group setup patched with the motion settings.

    This function patches the TCP and joint limits with the provided motion settings. The input
    ``motion_group_setup`` is not modified; a copy is returned (only ``global_limits`` is deep-copied).

    Used by the ``plan_collision_free`` path, which has no per-segment ``limits_override``: the
    user limits are folded into ``global_limits`` instead. The collision-checked
    ``plan_trajectory`` path does not use this — it sends per-segment overrides.

    Args:
        motion_group_setup: The motion group setup to patch
        settings: Motion settings containing limits to apply

    Returns:
        A new motion group setup with the settings applied.
    """
    motion_group_setup = motion_group_setup.model_copy(deep=False)
    if motion_group_setup.global_limits is not None:
        motion_group_setup.global_limits = motion_group_setup.global_limits.model_copy(deep=True)
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

    return motion_group_setup
