from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
from wandelbots_api_client import models


def m_to_mm(value: float) -> float:
    """Convert centimeters to millimeters."""
    return value * 1000.0


def convert_position(position: list[float]) -> list[float]:
    """Convert position coordinates from m to mm."""
    return [m_to_mm(p) for p in position]


def quaternion_to_angle_axis(quaternion: list[float]) -> list[float]:
    """Convert quaternion [w, x, y, z] to angle-axis [rx, ry, rz].

    Args:
        quaternion: Quaternion in [w, x, y, z] format as used by robometrics
    Returns:
        Angle-axis representation [rx, ry, rz] in radians
    """
    # Convert [w,x,y,z] to [x,y,z,w] for scipy
    w, x, y, z = quaternion
    rot = Rotation.from_quat([x, y, z, w])
    return rot.as_rotvec()


def convert_pose_quaternion(pose: list[float]) -> tuple[list[float], list[float]]:
    """Convert pose from robometrics format to Nova format.

    The pose transformation order is:
    1. Convert position to mm
    2. Calculate orientation (quaternion to angle-axis)

    Args:
        pose: [x, y, z, w, x, y, z] list containing position in meters
             and quaternion in [w,x,y,z] format
    Returns:
        tuple of (position_mm, orientation_rad)
    """
    # First convert position from meters to mm
    position = convert_position(pose[:3])

    # Then handle orientation - quaternion already in [w,x,y,z] format
    angles = quaternion_to_angle_axis(pose[3:])

    return position, angles


def create_box_collider(name: str, cube: dict[str, Any]) -> tuple[str, models.Collider]:
    """Create a box collider with mm dimensions using ConvexHull2."""
    position, orientation = convert_pose_quaternion(cube["pose"])
    dims = convert_position(cube["dims"])

    # Create box vertices (8 corners)
    half_x, half_y, half_z = [d / 2 for d in dims]
    vertices = [
        [-half_x, -half_y, -half_z],
        [-half_x, -half_y, half_z],
        [-half_x, half_y, -half_z],
        [-half_x, half_y, half_z],
        [half_x, -half_y, -half_z],
        [half_x, -half_y, half_z],
        [half_x, half_y, -half_z],
        [half_x, half_y, half_z],
    ]

    # Apply rotation to vertices first
    rot = Rotation.from_rotvec(np.array(orientation))
    rotated_vertices = [rot.apply(v) for v in vertices]

    return name, models.Collider(
        shape=models.ColliderShape(
            models.ConvexHull2(vertices=rotated_vertices, shape_type="convex_hull")
        ),
        pose=models.Pose2(
            position=position,
            orientation=[0, 0, 0],  # Orientation already applied to vertices
        ),
    )


def create_cylinder_collider(name: str, cylinder: dict[str, Any]) -> tuple[str, models.Collider]:
    """Create a cylinder collider with mm dimensions using ConvexHull2."""
    position, orientation = convert_pose_quaternion(cylinder["pose"])
    radius = m_to_mm(cylinder["radius"])
    height = m_to_mm(cylinder["height"])

    # Create cylinder vertices (discretized circle at top and bottom)
    num_points = 16  # Number of points to approximate the circular cross-section
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

    # Create points for top and bottom circles
    top_points = [[radius * np.cos(a), radius * np.sin(a), height / 2] for a in angles]
    bottom_points = [[radius * np.cos(a), radius * np.sin(a), -height / 2] for a in angles]
    vertices = top_points + bottom_points

    # Apply rotation to vertices first
    rot = Rotation.from_rotvec(np.array(orientation))
    rotated_vertices = [rot.apply(v) for v in vertices]

    return name, models.Collider(
        shape=models.ColliderShape(
            models.ConvexHull2(vertices=rotated_vertices, shape_type="convex_hull")
        ),
        pose=models.Pose2(
            position=position,
            orientation=[0, 0, 0],  # Orientation already applied to vertices
        ),
    )
