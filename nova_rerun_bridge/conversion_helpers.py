from nova import api
from nova.types import Pose


def normalize_pose(pose: api.models.Pose | None = None) -> Pose:
    """Convert pose to normalized format with Vector3d components."""
    # Default components
    default_position = api.models.Vector3d([0.0, 0.0, 0.0])
    default_orientation = api.models.RotationVector([0.0, 0.0, 0.0])

    # Use default pose if none provided
    if pose is None:
        return Pose(tuple(default_position.root) + tuple(default_orientation.root))

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

    return Pose(tuple(position.root) + tuple(orientation.root))
