from nova import api


def normalize_pose(pose: api.models.Pose | None = None) -> api.models.Pose:
    """Convert pose to normalized format with Vector3d components."""
    # Default components
    default_position = api.models.Vector3d([0.0, 0.0, 0.0])
    default_orientation = api.models.RotationVector([0.0, 0.0, 0.0])

    # Use default pose if none provided
    if pose is None:
        return api.models.Pose(position=default_position, orientation=default_orientation)

    # Handle position conversion
    position = (
        pose.position
        if hasattr(pose, "position") and pose.position is not None
        else default_position
    )

    orientation = (
        pose.orientation
        if hasattr(pose, "orientation") and pose.orientation is not None
        else default_orientation
    )

    return api.models.Pose(position=position, orientation=orientation)
