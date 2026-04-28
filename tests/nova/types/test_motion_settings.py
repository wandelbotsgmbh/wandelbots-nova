import pytest

from nova.actions import cartesian_ptp
from nova.actions.container import CombinedActions
from nova.types import Pose
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


def test_zero_blending_values_are_treated_as_explicit_settings():
    """Zero blending values are valid explicit settings and must not be treated as missing."""
    zero_radius_settings = MotionSettings(blending_radius=0.0)
    zero_radius_blending = zero_radius_settings.as_blending_setting()
    assert zero_radius_settings.has_blending_settings() is True
    assert zero_radius_blending.position_zone_radius == 0.0

    zero_auto_settings = MotionSettings(blending_auto=0)
    zero_auto_blending = zero_auto_settings.as_blending_setting()
    assert zero_auto_settings.has_blending_settings() is True
    assert zero_auto_blending.min_velocity_in_percent == 0


def test_zero_blending_radius_still_conflicts_with_blending_auto():
    """Zero blending radius still conflicts with blending_auto during validation."""
    with pytest.raises(ValueError, match="Can't set both blending_radius and blending_auto"):
        MotionSettings(blending_radius=0.0, blending_auto=50)


def test_zero_scalar_limits_are_treated_as_overrides():
    """All scalar TCP limits should count as overrides even when explicitly set to zero."""
    assert MotionSettings(tcp_velocity_limit=0.0).has_limits_override() is True
    assert MotionSettings(tcp_acceleration_limit=0.0).has_limits_override() is True
    assert MotionSettings(tcp_orientation_velocity_limit=0.0).has_limits_override() is True
    assert MotionSettings(tcp_orientation_acceleration_limit=0.0).has_limits_override() is True


def test_to_motion_command_keeps_zero_limit_overrides():
    """Explicit zero-valued scalar limits should still be serialized as overrides."""
    target_pose = Pose((1.0, 2.0, 3.0, 0.0, 0.0, 0.0))
    test_cases = [
        {
            "description": "TCP velocity set to 0.0 should still create a limits override.",
            "settings": MotionSettings(tcp_velocity_limit=0.0),
            "field_name": "tcp_velocity_limit",
        },
        {
            "description": "TCP acceleration set to 0.0 should still create a limits override.",
            "settings": MotionSettings(tcp_acceleration_limit=0.0),
            "field_name": "tcp_acceleration_limit",
        },
        {
            "description": (
                "TCP orientation velocity set to 0.0 should still create a limits override."
            ),
            "settings": MotionSettings(tcp_orientation_velocity_limit=0.0),
            "field_name": "tcp_orientation_velocity_limit",
        },
        {
            "description": (
                "TCP orientation acceleration set to 0.0 should still create a limits override."
            ),
            "settings": MotionSettings(tcp_orientation_acceleration_limit=0.0),
            "field_name": "tcp_orientation_acceleration_limit",
        },
    ]

    for test_case in test_cases:
        combined_actions = CombinedActions(
            items=(cartesian_ptp(target_pose, settings=test_case["settings"]),)
        )

        motion_commands = combined_actions.to_motion_command()

        assert len(motion_commands) == 1
        assert motion_commands[0].limits_override is not None, test_case["description"]
        assert getattr(motion_commands[0].limits_override, test_case["field_name"]) == 0.0, (
            test_case["description"]
        )
