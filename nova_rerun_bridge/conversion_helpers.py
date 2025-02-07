from typing import Optional

from nova.api import models
from scipy.spatial.transform import Rotation as R


def normalize_pose(pose: Optional[models.Pose] = None) -> models.PlannerPose:
    """Convert pose to normalized format with Vector3d components."""
    # Default components
    default_position = models.Vector3d(x=0.0, y=0.0, z=0.0)
    default_orientation = models.Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)

    # Use default pose if none provided
    if pose is None:
        return models.PlannerPose(position=default_position, orientation=default_orientation)

    # Handle position conversion
    if isinstance(pose.position, list):
        position = models.Vector3d(
            x=float(pose.position[0]), y=float(pose.position[1]), z=float(pose.position[2])
        )
    else:
        position = (
            pose.position
            if hasattr(pose, "position") and pose.position is not None
            else default_position
        )

    # Handle orientation conversion
    if isinstance(pose.orientation, list):
        orientation = models.Vector3d(
            x=float(pose.orientation[0]), y=float(pose.orientation[1]), z=float(pose.orientation[2])
        )
    else:
        orientation = (
            pose.orientation
            if hasattr(pose, "orientation") and pose.orientation is not None
            else default_orientation
        )

    # Convert rotation vector to quaternion
    rot_vec = [orientation.x, orientation.y, orientation.z]
    r = R.from_rotvec(rot_vec)
    quat = r.as_quat()  # [x, y, z, w]

    return models.PlannerPose(
        position=position,
        orientation=models.Quaternion(
            x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])
        ),
    )
