import numpy as np
import rerun as rr

from nova import api
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.hull_visualizer import HullVisualizer


def log_safety_zones(
    motion_group_id: str, motion_group_description: api.models.MotionGroupDescription
) -> None:
    """
    Log hull outlines for the safety zones defined in the optimizer configuration.
    """
    if motion_group_description.safety_zones is None:
        return

    if motion_group_description.dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    mounting = motion_group_description.mounting or api.models.Pose(
        position=api.models.Vector3d([0, 0, 0]), orientation=api.models.RotationVector([0, 0, 0])
    )
    robot = DHRobot(motion_group_description.dh_parameters, mounting)

    zones = motion_group_description.safety_zones.root
    for zone_id, collider in zones.items():
        entity_path = f"{motion_group_id}/zones/zone_{zone_id}"
        polygons = collider_to_polygons(collider)

        if not polygons:
            continue

        accumulated = robot.pose_to_matrix(mounting)
        polygons = apply_transform_to_polygons(polygons, accumulated)

        # Log polygons as wireframe outlines
        if polygons:
            line_segments = [p.tolist() for p in polygons]  # convert numpy arrays to lists
            rr.log(
                entity_path,
                rr.LineStrips3D(
                    line_segments, radii=rr.Radius.ui_points(0.75), colors=[[221, 193, 193, 255]]
                ),
                static=True,
            )


def apply_transform_to_polygons(polygons, transform):
    """
    Apply a transformation matrix to a list of polygons.
    """
    transformed_polygons = []
    for polygon in polygons:
        # Convert polygon to homogeneous coordinates
        homogeneous_polygon = np.hstack((polygon, np.ones((polygon.shape[0], 1))))
        # Apply the transformation
        transformed_polygon = np.dot(transform, homogeneous_polygon.T).T
        # Convert back to 3D coordinates
        transformed_polygons.append(transformed_polygon[:, :3])
    return transformed_polygons


def collider_to_polygons(collider: api.models.Collider):
    """
    Convert a collider definition into convex hull polygons if possible.
    """
    shape = collider.shape
    if isinstance(shape, api.models.ConvexHull):
        points = [[vertex[0], vertex[1], vertex[2]] for vertex in shape.vertices]
        return HullVisualizer.compute_hull_outlines_from_points(np.array(points))

    return []
