import numpy as np
import rerun as rr

from nova.api import models
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.hull_visualizer import HullVisualizer


def log_safety_zones(motion_group: str, optimizer_config: models.OptimizerSetup):
    """
    Log hull outlines for the safety zones defined in the optimizer configuration.
    """
    if optimizer_config.safety_setup.safety_zones is None:
        return

    if optimizer_config.dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    mounting_transform = optimizer_config.mounting
    robot = DHRobot(optimizer_config.dh_parameters, optimizer_config.mounting)

    for zone in optimizer_config.safety_setup.safety_zones:
        geom = zone.geometry
        zone_id = zone.id
        entity_path = f"{motion_group}/zones/zone_{zone_id}"

        if geom.compound is not None:
            child_geoms = geom.compound.child_geometries
            polygons = HullVisualizer.compute_hull_outlines_from_geometries(child_geoms)
        elif geom.convex_hull is not None:

            class ChildWrapper:
                def __init__(self, convex_hull):
                    self.convex_hull = convex_hull

            c = ChildWrapper(geom.convex_hull)
            c.convex_hull = geom.convex_hull
            polygons = HullVisualizer.compute_hull_outlines_from_geometries([c])
        else:
            polygons = []

        accumulated = robot.pose_to_matrix(mounting_transform)
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
