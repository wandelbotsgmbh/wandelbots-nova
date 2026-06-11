import pytest

from nova.api import models
from nova.types.motion_settings import DEFAULT_TCP_VELOCITY_LIMIT, MotionSettings
from nova.utils.motion_group_settings import (
    clamp_limits_override_to_global_limits,
    clamp_motion_commands_to_global_limits,
    update_motion_group_setup_with_motion_settings,
)


def test_tcp_limits_patching_with_none_setup():
    """Test TCP limits patching when setup has no existing TCP limits."""
    # Arrange
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"), cycle_time=8
    )
    settings = MotionSettings(
        tcp_velocity_limit=100.0,
        tcp_acceleration_limit=200.0,
        tcp_orientation_velocity_limit=1.5,
        tcp_orientation_acceleration_limit=3.0,
    )

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Assert
    assert motion_group_setup.global_limits.tcp is not None
    assert motion_group_setup.global_limits.tcp.velocity == 100.0
    assert motion_group_setup.global_limits.tcp.acceleration == 200.0
    assert motion_group_setup.global_limits.tcp.orientation_velocity == 1.5
    assert motion_group_setup.global_limits.tcp.orientation_acceleration == 3.0


def test_tcp_limits_patching_with_existing_setup():
    """Test TCP limits patching when setup has existing TCP limits."""
    # Arrange
    existing_tcp_limits = models.CartesianLimits(
        velocity=50.0, acceleration=100.0, orientation_velocity=1.0, orientation_acceleration=2.0
    )
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"),
        cycle_time=8,
        global_limits=models.LimitSet(tcp=existing_tcp_limits),
        collision_setups=None,
    )
    settings = MotionSettings(tcp_velocity_limit=200.0, tcp_orientation_acceleration_limit=5.0)

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Assert
    # these are updated
    assert motion_group_setup.global_limits.tcp.velocity == 200.0
    assert motion_group_setup.global_limits.tcp.orientation_acceleration == 5.0

    # there are not
    assert motion_group_setup.global_limits.tcp.acceleration == 100.0
    assert motion_group_setup.global_limits.tcp.orientation_velocity == 1.0


def test_tcp_limits_patching_all_none_in_settings():
    """Test TCP limits patching when all settings values are None."""
    # Arrange
    existing_tcp_limits = models.CartesianLimits(
        velocity=50.0, acceleration=100.0, orientation_velocity=1.0, orientation_acceleration=2.0
    )
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(tcp=existing_tcp_limits),
    )
    settings = MotionSettings()

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Updated to default velocity limit
    assert motion_group_setup.global_limits.tcp.velocity == DEFAULT_TCP_VELOCITY_LIMIT

    # unchanged
    assert motion_group_setup.global_limits.tcp.acceleration == 100.0
    assert motion_group_setup.global_limits.tcp.orientation_velocity == 1.0
    assert motion_group_setup.global_limits.tcp.orientation_acceleration == 2.0


def test_joint_limits_replacement_with_none_setup():
    """Test joint limits replacement when setup has no existing joint limits."""
    # Arrange
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test", cycle_time=8, collision_setups=None
    )
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0), joint_acceleration_limits=(4.0, 5.0, 6.0)
    )

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Assert
    assert motion_group_setup.global_limits.joints is not None
    assert motion_group_setup.global_limits.joints[0].velocity == 1.0
    assert motion_group_setup.global_limits.joints[1].velocity == 2.0
    assert motion_group_setup.global_limits.joints[2].velocity == 3.0

    assert motion_group_setup.global_limits.joints[0].acceleration == 4.0
    assert motion_group_setup.global_limits.joints[1].acceleration == 5.0
    assert motion_group_setup.global_limits.joints[2].acceleration == 6.0


def test_joint_limits_merging_with_existing_setup():
    """Test joint limits merging preserves position limits while updating velocity/acceleration."""
    # Arrange
    existing_joint_limits = [
        models.JointLimits(
            position=models.LimitRange(lower_limit=-3.14, upper_limit=3.14),
            velocity=10.0,
            acceleration=20.0,
        ),
        models.JointLimits(
            position=models.LimitRange(lower_limit=-2.0, upper_limit=2.0),
            velocity=30.0,
            acceleration=40.0,
        ),
        models.JointLimits(
            position=models.LimitRange(lower_limit=-1.5, upper_limit=1.5),
            velocity=50.0,
            acceleration=60.0,
        ),
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0), joint_acceleration_limits=(4.0, 5.0, 6.0)
    )

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Assert - velocity and acceleration updated, position preserved
    assert len(motion_group_setup.global_limits.joints) == 3

    # Joint 0: verify merge
    assert motion_group_setup.global_limits.joints[0].velocity == 1.0
    assert motion_group_setup.global_limits.joints[0].acceleration == 4.0
    assert motion_group_setup.global_limits.joints[0].position is not None
    assert motion_group_setup.global_limits.joints[0].position.lower_limit == -3.14
    assert motion_group_setup.global_limits.joints[0].position.upper_limit == 3.14

    # Joint 1: verify merge
    assert motion_group_setup.global_limits.joints[1].velocity == 2.0
    assert motion_group_setup.global_limits.joints[1].acceleration == 5.0
    assert motion_group_setup.global_limits.joints[1].position is not None
    assert motion_group_setup.global_limits.joints[1].position.lower_limit == -2.0
    assert motion_group_setup.global_limits.joints[1].position.upper_limit == 2.0

    # Joint 2: verify merge
    assert motion_group_setup.global_limits.joints[2].velocity == 3.0
    assert motion_group_setup.global_limits.joints[2].acceleration == 6.0
    assert motion_group_setup.global_limits.joints[2].position is not None
    assert motion_group_setup.global_limits.joints[2].position.lower_limit == -1.5
    assert motion_group_setup.global_limits.joints[2].position.upper_limit == 1.5


def test_no_joint_limits_in_settings():
    """Test that existing joint limits are preserved when settings has no joint limits."""
    # Arrange
    existing_joint_limits = [
        models.JointLimits(velocity=10.0, acceleration=20.0),
        models.JointLimits(velocity=30.0, acceleration=40.0),
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    settings = MotionSettings()

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Assert - Existing joint limits should be preserved
    assert motion_group_setup.global_limits.joints == existing_joint_limits


def test_joint_limits_raises_error_when_joint_count_differs():
    """Test that ValueError is raised when joint count differs (configuration error)."""
    # Arrange - 2 existing joints
    existing_joint_limits = [
        models.JointLimits(
            position=models.LimitRange(lower_limit=-3.14, upper_limit=3.14),
            velocity=10.0,
            acceleration=20.0,
        ),
        models.JointLimits(
            position=models.LimitRange(lower_limit=-2.0, upper_limit=2.0),
            velocity=30.0,
            acceleration=40.0,
        ),
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    # Settings has 3 joints (different count - configuration error)
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0), joint_acceleration_limits=(4.0, 5.0, 6.0)
    )

    # Act & Assert - Should raise ValueError
    with pytest.raises(ValueError, match="Joint count mismatch"):
        update_motion_group_setup_with_motion_settings(motion_group_setup, settings)


def test_joint_limits_merge_with_partial_settings():
    """Test merging when settings only specify velocity (no acceleration)."""
    # Arrange
    existing_joint_limits = [
        models.JointLimits(
            position=models.LimitRange(lower_limit=-3.14, upper_limit=3.14),
            velocity=10.0,
            acceleration=20.0,
        )
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    # Only velocity specified, no acceleration
    settings = MotionSettings(joint_velocity_limits=(5.0,))

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Assert - velocity updated, acceleration and position preserved
    joint = motion_group_setup.global_limits.joints[0]
    assert joint.velocity == 5.0  # Updated from settings
    assert joint.acceleration == 20.0  # Preserved (settings had None)
    assert joint.position is not None  # Preserved
    assert joint.position.lower_limit == -3.14
    assert joint.position.upper_limit == 3.14


def test_joint_limits_merge_preserves_position_and_torque():
    """Test that position and torque limits are preserved when settings updates velocity/accel.

    This is the primary bug fix test - settings.as_joint_limits() returns JointLimits
    with only velocity and acceleration set, and position/torque should be preserved.
    """
    # Arrange
    existing_joint_limits = [
        models.JointLimits(
            position=models.LimitRange(lower_limit=-1.57, upper_limit=1.57),
            velocity=100.0,
            acceleration=200.0,
            torque=50.0,
        )
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    settings = MotionSettings(joint_velocity_limits=(25.0,), joint_acceleration_limits=(75.0,))

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Assert - velocity and acceleration updated, position and torque preserved
    joint = motion_group_setup.global_limits.joints[0]
    assert joint.velocity == 25.0  # Updated from settings
    assert joint.acceleration == 75.0  # Updated from settings
    assert joint.position is not None  # Preserved
    assert joint.position.lower_limit == -1.57  # Preserved
    assert joint.position.upper_limit == 1.57  # Preserved
    assert joint.torque == 50.0  # Preserved


def test_clamp_tcp_limits_above_max_are_reduced():
    """TCP override values above the global max are clamped to the max."""
    global_limits = models.LimitSet(
        tcp=models.CartesianLimits(
            velocity=250.0,
            acceleration=1000.0,
            orientation_velocity=2.0,
            orientation_acceleration=5.0,
        )
    )
    override = models.LimitsOverride(
        tcp_velocity_limit=2500.0,  # way above max
        tcp_acceleration_limit=500.0,  # below max -> unchanged
        tcp_orientation_velocity_limit=10.0,  # above max
    )

    clamped = clamp_limits_override_to_global_limits(override, global_limits)

    assert clamped.tcp_velocity_limit == 250.0  # clamped
    assert clamped.tcp_acceleration_limit == 500.0  # untouched
    assert clamped.tcp_orientation_velocity_limit == 2.0  # clamped
    # input is not modified
    assert override.tcp_velocity_limit == 2500.0
    assert override.tcp_orientation_velocity_limit == 10.0


def test_clamp_keeps_unset_and_missing_max_values():
    """Unset override fields and fields without a global max are left untouched."""
    global_limits = models.LimitSet(
        tcp=models.CartesianLimits(velocity=250.0)  # only velocity has a max
    )
    override = models.LimitsOverride(
        tcp_velocity_limit=None,  # unset -> stays None
        tcp_acceleration_limit=9999.0,  # no max available -> untouched
    )

    clamped = clamp_limits_override_to_global_limits(override, global_limits)

    assert clamped.tcp_velocity_limit is None
    assert clamped.tcp_acceleration_limit == 9999.0


def test_clamp_joint_limits_element_wise():
    """Per-joint override values are clamped element-wise to the joint maxima."""
    global_limits = models.LimitSet(
        joints=[
            models.JointLimits(velocity=1.0, acceleration=4.0),
            models.JointLimits(velocity=2.0, acceleration=5.0),
            models.JointLimits(velocity=3.0, acceleration=6.0),
        ]
    )
    override = models.LimitsOverride(
        joint_velocity_limits=[10.0, 0.5, 100.0], joint_acceleration_limits=[1.0, 50.0, 60.0]
    )

    clamped = clamp_limits_override_to_global_limits(override, global_limits)

    assert clamped.joint_velocity_limits == [1.0, 0.5, 3.0]
    assert clamped.joint_acceleration_limits == [1.0, 5.0, 6.0]


def test_clamp_no_global_limits_is_noop():
    """When there are no global limits, the override values are returned unchanged."""
    override = models.LimitsOverride(tcp_velocity_limit=2500.0)

    clamped = clamp_limits_override_to_global_limits(override, None)

    assert clamped.tcp_velocity_limit == 2500.0


def test_clamp_motion_commands_skips_commands_without_override():
    """clamp_motion_commands_to_global_limits clamps overrides and skips None ones."""
    global_limits = models.LimitSet(tcp=models.CartesianLimits(velocity=250.0))
    cmd_with_override = models.MotionCommand(
        path=models.PathJointPTP(
            target_joint_position=models.Joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            path_definition_name="PathJointPTP",
        ),
        limits_override=models.LimitsOverride(tcp_velocity_limit=2500.0),
    )
    cmd_without_override = models.MotionCommand(
        path=models.PathJointPTP(
            target_joint_position=models.Joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            path_definition_name="PathJointPTP",
        ),
        limits_override=None,
    )

    clamped = clamp_motion_commands_to_global_limits(
        [cmd_with_override, cmd_without_override], global_limits
    )

    assert clamped[0].limits_override.tcp_velocity_limit == 250.0
    assert clamped[1].limits_override is None
    # inputs are not modified
    assert cmd_with_override.limits_override.tcp_velocity_limit == 2500.0
