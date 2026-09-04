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
            location + current_end_location for location in trajectory.locations[1:]
        ]  # Skip first point

        final_trajectory.times.extend(shifted_times)
        final_trajectory.joint_positions.extend(trajectory.joint_positions[1:])
        final_trajectory.locations.extend(shifted_locations)

        current_end_time = final_trajectory.times[-1]
        current_end_location = final_trajectory.locations[-1]

    return final_trajectory


def combine_multi_trajectories(
    trajectories: list[api.models.MultiJointTrajectory],
) -> api.models.MultiJointTrajectory:
    """Concatenate synchronized multi-motion-group trajectories end to end.

    The multi-group counterpart of :func:`combine_trajectories`: every segment
    shares one ``times``/``locations`` across all its groups, and the seam shifts
    later segments to continue from the previous end — so the result keeps the
    single shared parameterization synchronized execution rests on. All segments
    must cover the same set of motion groups.
    """
    final_trajectory = trajectories[0]
    keys = set(final_trajectory.joint_positions_by_motion_group_key)
    current_end_time = final_trajectory.times[-1]
    current_end_location = final_trajectory.locations[-1]

    for trajectory in trajectories[1:]:
        if set(trajectory.joint_positions_by_motion_group_key) != keys:
            raise ValueError(
                "All segments must cover the same motion groups; got "
                f"{sorted(trajectory.joint_positions_by_motion_group_key)} vs {sorted(keys)}"
            )

        # Skip each segment's first point: it duplicates the previous seam.
        final_trajectory.times.extend(t + current_end_time for t in trajectory.times[1:])
        final_trajectory.locations.extend(
            location + current_end_location for location in trajectory.locations[1:]
        )
        for key, samples in trajectory.joint_positions_by_motion_group_key.items():
            final_trajectory.joint_positions_by_motion_group_key[key].extend(samples[1:])

        current_end_time = final_trajectory.times[-1]
        current_end_location = final_trajectory.locations[-1]

    return final_trajectory
