"""Draw a Nova collider in Rerun.

One owner for the question "what does this collider look like", used both for a
robot's own volumes and for the colliders of a collision setup. Rerun has the
primitives, so a sphere is a sphere rather than a mesh built for the occasion;
only the hull shapes have to be assembled.

Everything here draws see-through: a collider wraps something, and an opaque
shell hides what it is meant to explain.
"""

from typing import Any

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation
import trimesh

from nova import api
from nova_rerun_bridge import scene_colors
from nova_rerun_bridge.hull_visualizer import HullVisualizer

VOLUME_ALPHA = scene_colors.SAFETY_VOLUME[3]
"""How see-through a collider is drawn."""

VOLUME_COLOR = scene_colors.SAFETY_VOLUME
"""Default colour for a collider: the controller's safety volumes."""

OVER_ROBOT = rr.components.FillMode.TransparentFillMajorWireframe
"""How a volume that sits on the robot is drawn: see-through, read from its
wireframe. A solid fill would paint over the model it encloses."""

IN_CELL = rr.components.FillMode.Solid
"""How a volume standing on its own in the cell is drawn."""

EDGE_ANGLE = np.radians(20.0)
"""How sharp a fold has to be to count as an edge of a collider."""

_PLANE_SIZE_MM = 5000.0
"""A plane is a half-space; drawn as a slab wide enough to read as one."""


def pose_matrix(pose: Any) -> np.ndarray:
    """A collider pose as a 4x4 transform.

    A collider's pose orientation is a ``RotationVector`` -- three components,
    not a quaternion, whatever the shape of the collider.
    """
    matrix = np.eye(4)
    if pose is None:
        return matrix
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is not None:
        matrix[:3, 3] = [float(value) for value in list(position)[:3]]
    if orientation is not None:
        rotvec = [float(value) for value in list(orientation)[:3]]
        if any(rotvec):
            matrix[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    return matrix


def log_collider(
    entity_path: str,
    collider: api.models.Collider,
    frame: np.ndarray | None = None,
    *,
    color: tuple[int, ...] = VOLUME_COLOR,
    fill_mode: Any = OVER_ROBOT,
    recording: rr.RecordingStream | None = None,
    static: bool = True,
    with_shape: bool = True,
) -> None:
    """Place one collider at *frame* and draw its shape there.

    *frame* is what the collider's pose is relative to: a link's frame for a
    robot's volumes, the identity for one given in cell coordinates.
    """
    placement = (np.eye(4) if frame is None else frame) @ pose_matrix(collider.pose)
    quaternion = Rotation.from_matrix(placement[:3, :3]).as_quat()
    rr.log(
        entity_path,
        rr.Transform3D(translation=placement[:3, 3].tolist(), quaternion=quaternion.tolist()),
        static=static,
        recording=recording,
    )
    if not with_shape:
        return
    archetype, edges = archetype_for(collider, color, fill_mode)
    if archetype is None:
        return
    rr.log(entity_path, archetype, static=True, recording=recording)
    if edges is not None:
        # A mesh archetype has no wireframe of its own, so its folds are drawn
        # as lines: same idea, done by hand.
        rr.log(
            f"{entity_path}/edges",
            rr.LineStrips3D(
                edges, colors=[list(color[:3]) + [255]], radii=rr.Radius.ui_points(0.5)
            ),
            static=True,
            recording=recording,
        )


def archetype_for(
    collider: api.models.Collider, color: tuple[int, ...], fill_mode: Any = OVER_ROBOT
) -> tuple[Any, np.ndarray | None]:
    """A collider's shape as the archetype that draws it, at the origin, plus
    the edges to draw with it when it comes out as a mesh.
    """
    shape = collider.shape
    fill = fill_mode
    solid = fill_mode is IN_CELL
    mesh_color = color if solid else (*color[:3], 45)

    if isinstance(shape, api.models.Sphere):
        radius = shape.radius
        return (
            rr.Ellipsoids3D(radii=[[radius, radius, radius]], colors=[color], fill_mode=fill),
            None,
        )
    if isinstance(shape, api.models.Box):
        return (
            rr.Boxes3D(
                half_sizes=[[shape.size_x / 2, shape.size_y / 2, shape.size_z / 2]],
                colors=[color],
                fill_mode=fill,
            ),
            None,
        )
    if isinstance(shape, api.models.Rectangle):
        return (
            rr.Boxes3D(
                half_sizes=[[shape.size_x / 2, shape.size_y / 2, 0.5]],
                colors=[color],
                fill_mode=fill,
            ),
            None,
        )
    if isinstance(shape, api.models.Plane):
        return (
            rr.Boxes3D(
                half_sizes=[[_PLANE_SIZE_MM / 2, _PLANE_SIZE_MM / 2, 0.5]],
                colors=[color],
                fill_mode=fill,
            ),
            None,
        )
    if isinstance(shape, api.models.Cylinder):
        # Rerun centres a cylinder on the entity origin, which is where the API
        # places it too.
        return (
            rr.Cylinders3D(
                lengths=[shape.height], radii=[shape.radius], colors=[color], fill_mode=fill
            ),
            None,
        )
    if isinstance(shape, api.models.Capsule):
        height = shape.cylinder_height
        # A Rerun capsule runs from the origin along +Z; the API gives its
        # centre, so slide it back by half its length.
        return (
            rr.Capsules3D(
                lengths=[height],
                radii=[shape.radius],
                translations=[[0.0, 0.0, -height / 2]],
                colors=[color],
                fill_mode=fill,
            ),
            None,
        )
    if isinstance(shape, api.models.ConvexHull):
        return hull_mesh(np.asarray(shape.vertices, dtype=np.float64), mesh_color, solid=solid)
    if isinstance(shape, api.models.RectangularCapsule):
        return hull_mesh(
            rectangular_capsule_points(
                shape.radius, shape.sphere_center_distance_x, shape.sphere_center_distance_y
            ),
            mesh_color,
            solid=solid,
        )
    return None, None


def hull_mesh(
    points: np.ndarray, color: tuple[int, ...], *, solid: bool = True
) -> tuple[Any, np.ndarray | None]:
    """The convex hull of *points* as a mesh, with its edges when see-through."""
    polygons = HullVisualizer.compute_hull_outlines_from_points(points)
    if not polygons:
        return None, None
    vertices, triangles, normals = HullVisualizer.compute_hull_mesh(polygons)
    mesh = rr.Mesh3D(
        vertex_positions=vertices,
        triangle_indices=triangles,
        vertex_normals=normals,
        albedo_factor=color,
    )
    return mesh, None if solid else hull_edges(vertices, triangles)


def hull_edges(vertices: Any, triangles: Any) -> np.ndarray | None:
    """A mesh's sharp folds as line strips, or nothing if it has none."""
    mesh = trimesh.Trimesh(
        vertices=np.asarray(vertices), faces=np.asarray(triangles), process=False
    )
    edges = np.asarray(mesh.face_adjacency_edges)
    angles = np.asarray(mesh.face_adjacency_angles)
    if len(edges) == 0:
        return None
    sharp = edges[angles > EDGE_ANGLE]
    if len(sharp) == 0:
        return None
    return np.asarray(mesh.vertices)[sharp]


def rectangular_capsule_points(radius: float, distance_x: float, distance_y: float) -> np.ndarray:
    """Points on the four spheres a rectangular capsule is the hull of."""
    axes = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    return np.array(
        [
            [sign_x * distance_x + radius * dx, sign_y * distance_y + radius * dy, radius * dz]
            for sign_x in (1, -1)
            for sign_y in (1, -1)
            for dx, dy, dz in axes
        ],
        dtype=np.float64,
    )
