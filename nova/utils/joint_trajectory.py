from nova import api


def combine_trajectories(
    trajectories: list[api.models.JointTrajectory],
) -> api.models.JointTrajectory:
    """
    Combines multiple trajectories into one trajectory.
    """
    final_trajectory = trajectories[0]
    current_end_time = final_trajectory.times[-1]
    current_end_location = final_trajectory.locations[-1]

    for trajectory in trajectories[1:]:
        # Shift times and locations to continue from last endpoint
        shifted_times = [t + current_end_time for t in trajectory.times[1:]]  # Skip first point
        shifted_locations = [
            api.models.Location(location.root + current_end_location.root)
            for location in trajectory.locations[1:]
        ]  # Skip first point

        final_trajectory.times.extend(shifted_times)
        final_trajectory.joint_positions.extend(trajectory.joint_positions[1:])
        final_trajectory.locations.extend(shifted_locations)

        current_end_time = final_trajectory.times[-1]
        current_end_location = final_trajectory.locations[-1]

    return final_trajectory
