import numpy as np
import rerun as rr

from nova import api
from nova_rerun_bridge import scene_colors
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.hull_visualizer import HullVisualizer

try:  # SciPy moved it; the bridge supports both places
    from scipy.spatial import QhullError
except ImportError:  # pragma: no cover
    from scipy.spatial.qhull import QhullError  # type: ignore[no-redef]


# A zone the robot may not enter is drawn as a see-through body, because that
# is what it is: a piece of the cell that is off limits. A zone it may not
# leave is drawn as an outline only, in the colour of a limit rather than of a
# forbidden place. Both colours come from the design system; see
# :mod:`nova_rerun_bridge.scene_colors`.


def log_safety_zones(
    motion_group_id: str, motion_group_description: api.models.MotionGroupDescription
) -> None:
    """
    Log the safety zones defined in the optimizer configuration.

    Each zone is drawn twice: its outline, and its body see-through.
    """
    if motion_group_description.safety_zones is None:
        return

    if motion_group_description.dh_parameters is None:
        raise ValueError("DH parameters cannot be None")

    mounting = motion_group_description.mounting or api.models.Pose(
        position=(0, 0, 0), orientation=(0, 0, 0)
    )
    robot = DHRobot(motion_group_description.dh_parameters, mounting)

    zones = motion_group_description.safety_zones
    for zone_id, collider in zones.items():
        entity_path = f"{motion_group_id}/zones/zone_{rr.escape_entity_path_part(zone_id)}"
        polygons = collider_to_polygons(collider)

        if not polygons:
            continue

        accumulated = robot.pose_to_matrix(mounting)
        polygons = apply_transform_to_polygons(polygons, accumulated)

        # Log polygons as wireframe outlines
        if polygons:
            keep_out = encloses_space(polygons)
            outline_color = (
                scene_colors.ZONE_KEEP_OUT_OUTLINE
                if keep_out
                else scene_colors.ZONE_KEEP_IN_OUTLINE
            )
            line_segments = [p.tolist() for p in polygons]  # convert numpy arrays to lists
            rr.log(
                entity_path,
                rr.LineStrips3D(
                    line_segments, radii=rr.Radius.ui_points(0.75), colors=[list(outline_color)]
                ),
                static=True,
            )
            if keep_out:
                log_zone_body(f"{entity_path}/body", polygons)


def encloses_space(polygons: list, flatness: float = 1e-3) -> bool:
    """Whether a zone is a body, or one flat face of a boundary.

    The API states a keep-out zone as a body and a keep-in zone as the faces of
    its boundary, one flat hull each. Which is which follows from the geometry:
    a face has no thickness, so its points are coplanar.
    """
    points = np.vstack(polygons)
    if len(points) < 4:
        return False
    singular = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    return bool(singular[0] > 0 and singular[2] / singular[0] > flatness)


def log_zone_body(entity_path: str, polygons: list) -> None:
    """Fill a zone's outline with a see-through body."""
    try:
        vertices, triangles, normals = HullVisualizer.compute_hull_mesh(polygons)
    except (ValueError, IndexError, QhullError):
        # A flat or degenerate zone has no body to fill; the outline stands.
        return
    rr.log(
        entity_path,
        rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=triangles,
            vertex_normals=normals,
            albedo_factor=scene_colors.ZONE_KEEP_OUT,
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
