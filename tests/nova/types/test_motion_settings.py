import pytest

from nova.types.motion_settings import DEFAULT_TCP_VELOCITY_LIMIT, MotionSettings


def test_different_joint_limits_length_raises():
    with pytest.raises(ValueError):
        MotionSettings(
            joint_velocity_limits=[0.5, 0.5, 0.5], joint_acceleration_limits=[1.0, 1.0, 1.0, 1.0]
        )


def test_motion_settings_with_no_explicit_tcp_limits():
    motion_settings = MotionSettings()
    tcp_cartesian_limits = motion_settings.as_tcp_cartesian_limits()
    assert tcp_cartesian_limits.velocity is DEFAULT_TCP_VELOCITY_LIMIT


def test_motion_settings_tcp_cartesian_limits():
    motion_settings = MotionSettings(
        tcp_velocity_limit=1.0,
        tcp_acceleration_limit=2.0,
        tcp_orientation_velocity_limit=0.5,
        tcp_orientation_acceleration_limit=1.5,
    )

    cartesian_limits = motion_settings.as_tcp_cartesian_limits()
    assert cartesian_limits.velocity == motion_settings.tcp_velocity_limit
    assert cartesian_limits.acceleration == motion_settings.tcp_acceleration_limit
    assert cartesian_limits.orientation_velocity == motion_settings.tcp_orientation_velocity_limit
    assert (
        cartesian_limits.orientation_acceleration
        == motion_settings.tcp_orientation_acceleration_limit
    )


def test_motion_settings_as_joint_limits():
    motion_settings = MotionSettings()
    limits = motion_settings.as_joint_limits()
    assert limits is None


def test_joint_velocity_limits():
    motion_settings = MotionSettings(
        joint_velocity_limits=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        joint_acceleration_limits=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    )

    limits = motion_settings.as_joint_limits()
    assert limits is not None
    assert len(limits) == 6
    for i in range(6):
        assert limits[i].velocity == motion_settings.joint_velocity_limits[i]
        assert limits[i].acceleration == motion_settings.joint_acceleration_limits[i]


def test_blending_settings():
    """
    Test that only one type of blending setting can be set at a time.
    """
    with pytest.raises(ValueError, match="Can't set both blending_radius and blending_auto"):
        MotionSettings(blending_radius=10.0, blending_auto=50)

    with pytest.raises(ValueError, match="Can't set both blending_radius and blending_auto"):
        MotionSettings(blending_radius=10.0, min_blending_velocity=50)

    with pytest.raises(ValueError, match="Can't set both blending_radius and blending_auto"):
        MotionSettings(position_zone_radius=10.0, blending_auto=50)

    with pytest.raises(ValueError, match="Can't set both blending_radius and blending_auto"):
        MotionSettings(position_zone_radius=10.0, min_blending_velocity=50)
