from nova.api import models
from nova.types.motion_settings import DEFAULT_TCP_VELOCITY_LIMIT, MotionSettings
from nova.utils.motion_group_settings import update_motion_group_setup_with_motion_settings


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


def test_joint_limits_replacement_with_existing_setup():
    """Test joint limits replacement when setup has existing joint limits."""
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
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0), joint_acceleration_limits=(4.0, 5.0, 6.0)
    )

    # Act
    update_motion_group_setup_with_motion_settings(motion_group_setup, settings)

    # Assert - Entire joint limits list should be replaced
    assert motion_group_setup.global_limits.joints is not None

    assert motion_group_setup.global_limits.joints[0].velocity == 1.0
    assert motion_group_setup.global_limits.joints[1].velocity == 2.0
    assert motion_group_setup.global_limits.joints[2].velocity == 3.0

    assert motion_group_setup.global_limits.joints[0].acceleration == 4.0
    assert motion_group_setup.global_limits.joints[1].acceleration == 5.0
    assert motion_group_setup.global_limits.joints[2].acceleration == 6.0


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
