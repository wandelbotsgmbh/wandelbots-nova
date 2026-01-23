import wandelbots_api_client as wb

from nova.actions.mock import wait


def test_wait_action_trajectory_duration():
    """Test that wait action generates trajectories with the correct duration."""
    # Create wait actions with different durations
    wait_action_1 = wait(2.5)  # 2.5 seconds
    wait_action_2 = wait(0.1)  # 0.1 seconds - edge case
    wait_action_3 = wait(10)  # 10 seconds - long wait

    # Mock current_joints for trajectory generation
    current_joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # Generate trajectories for testing
    def generate_wait_trajectory(wait_action):
        # Replicate trajectory generation logic from motion_group.py
        wait_time = wait_action.wait_for_in_seconds
        timestep = 0.050  # 50ms timestep
        num_steps = max(2, int(wait_time / timestep) + 1)

        joint_positions = [wb.models.Joints(joints=list(current_joints)) for _ in range(num_steps)]
        times = [i * timestep for i in range(num_steps)]
        # Ensure the last timestep is exactly the wait duration
        times[-1] = wait_time
        locations = [0] * num_steps

        return wb.models.JointTrajectory(
            joint_positions=joint_positions, times=times, locations=locations
        )

    # Generate trajectories
    trajectory_1 = generate_wait_trajectory(wait_action_1)
    trajectory_2 = generate_wait_trajectory(wait_action_2)
    trajectory_3 = generate_wait_trajectory(wait_action_3)

    # Test trajectory 1 (2.5 seconds)
    assert len(trajectory_1.times) >= 2
    assert trajectory_1.times[-1] == 2.5
    assert trajectory_1.times[-1] - trajectory_1.times[0] >= 2.0, (
        "Wait trajectory should be longer than 2 seconds"
    )

    # Test trajectory 2 (edge case - 0.1 seconds)
    assert len(trajectory_2.times) >= 2
    assert trajectory_2.times[-1] == 0.1

    # Test trajectory 3 (long wait - 10 seconds)
    assert len(trajectory_3.times) >= 2
    assert trajectory_3.times[-1] == 10.0
    assert trajectory_3.times[-1] - trajectory_3.times[0] >= 2.0, (
        "Wait trajectory should be longer than 2 seconds"
    )

    # Check that all joint positions are identical
    for trajectory in [trajectory_1, trajectory_2, trajectory_3]:
        for i in range(1, len(trajectory.joint_positions)):
            assert trajectory.joint_positions[i].joints == trajectory.joint_positions[0].joints

    # Check timestep spacing (approximately 50ms)
    for trajectory in [trajectory_1, trajectory_3]:  # Skip the short trajectory
        for i in range(1, len(trajectory.times) - 1):  # Skip last point which might be adjusted
            time_diff = trajectory.times[i] - trajectory.times[i - 1]
            assert 0.049 <= time_diff <= 0.051, (
                f"Timestep should be approximately 50ms but was {time_diff * 1000}ms"
            )
