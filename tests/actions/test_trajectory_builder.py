import pytest

from nova.actions import TrajectoryMacher, cartesian_ptp, io_write, joint_ptp, linear
from nova.types import MotionSettings, Pose

# Move motion settings to module level
slow = MotionSettings(tcp_velocity_limit=50)
normal = MotionSettings(tcp_velocity_limit=250)
fast = MotionSettings(tcp_velocity_limit=500)


@pytest.fixture
def trajectory():
    home_joints = Pose((0, 0, 0, 0, 0, 0))
    target_pose = Pose((1, 2, 3, 0, 0, 0))

    with TrajectoryMacher(settings=slow) as t:
        # all actions use the slow settings
        t.move(joint_ptp(home_joints))  # 0
        t.move(cartesian_ptp(target_pose))  # 1
        t.move(linear(target_pose @ Pose((50, 100, 0, 0, 0, 0))))  # 2
        t.move(joint_ptp(home_joints))  # 3
        t.sequence(
            [
                joint_ptp(home_joints),  # 4
                cartesian_ptp(target_pose @ Pose((50, 100, 0, 0, 0, 0)), settings=fast),  # 5
                joint_ptp(home_joints),  # 6
            ]
        )

        with t.set(settings=normal):
            # all actions in the context use the normal settings except for the last two
            for i in range(2):
                t.sequence(
                    [
                        joint_ptp(home_joints),  # 7, 10
                        cartesian_ptp(target_pose @ Pose((50, i * 10, 0, 0, 0, 0))),  # 8, 11
                        joint_ptp(home_joints),  # 9, 12
                    ]
                )

            t.move(cartesian_ptp(target_pose @ Pose((50, 0, 0, 0, 0, 0)), settings=fast))  # 13
            t.move(joint_ptp(home_joints, settings=fast))  # 14

        # all actions use the slow settings again
        t.trigger(io_write(key="tool_out[0]", value=False))  # 15
        t.move(cartesian_ptp(target_pose @ Pose((50, 100, 0, 0, 0, 0)), settings=fast))  # 16
        t.move(joint_ptp(home_joints))  # 17
        t.trigger(io_write(key="tool_out[0]", value=True))  # 18

    # pprint.pprint(t.actions)
    return t


# Helper to extract settings from an action (assuming .settings or similar attribute)
def get_settings(action):
    # Try to get settings from action, or from action['settings'] if dict
    if hasattr(action, "settings"):
        return action.settings
    if isinstance(action, dict) and "settings" in action:
        return action["settings"]
    return None


def test_trajectory_builder(trajectory):
    actions = trajectory.actions
    assert len(actions) == 19

    # Expected settings for each action in order
    expected_settings = [
        slow,  # 0
        slow,  # 1
        slow,  # 2
        slow,  # 3
        slow,  # 4
        fast,  # 5
        slow,  # 6
        normal,  # 7
        normal,  # 8
        normal,  # 9
        normal,  # 10
        normal,  # 11
        normal,  # 12
        fast,  # 13
        fast,  # 14
        None,  # 15
        fast,  # 16
        slow,  # 17
        None,  # 18
    ]

    for idx, (action, expected) in enumerate(zip(actions, expected_settings)):
        actual = get_settings(action)
        assert actual == expected, f"Action {idx} has wrong settings: {actual} != {expected}"


def test_trajectory_builder_without_settings():
    """Test that TrajectoryBuilder works when initialized without settings."""
    from nova.actions.trajectory_builder import TrajectoryBuilder
    from nova.actions.motions import joint_ptp
    from nova.types import Pose
    
    tb = TrajectoryBuilder()
    
    tb.move(joint_ptp(Pose((0, 0, 0, 0, 0, 0))))
    
    assert len(tb.actions) == 1
    
    motion = tb.actions[0]
    assert motion.settings is not None
    assert motion.settings.tcp_velocity_limit == 50
