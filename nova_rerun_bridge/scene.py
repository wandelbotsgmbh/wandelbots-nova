"""Static collision scene and robot-state visualization for the Rerun bridge.

This module is the single source of truth for turning NOVA colliders and robot
states into Rerun primitives. It is intentionally decoupled from live motion
groups so that scenes can be drawn from saved data without a running controller.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np
import rerun as rr
import trimesh
from scipy.spatial.transform import Rotation

from nova import api
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.hull_visualizer import HullVisualizer

logger = logging.getLogger(__name__)

DEFAULT_SCENE_COLOR = (221, 193, 193, 255)
COLLISION_HIGHLIGHT_COLOR = (255, 0, 0, 255)
WIRE_COLOR = (221, 193, 193, 255)


@dataclass(frozen=True)
class SceneEntity:
    """One drawable thing in a collision scene.

    Attributes:
        entity_path: Rerun entity path (already escaped where necessary).
        transform: 4x4 homogeneous transform from entity frame to world.
        collider: The NOVA collider geometry to draw.
        highlight: Whether to render this entity in the collision highlight color.
    """

    entity_path: str
    transform: np.ndarray
    collider: api.models.Collider
    highlight: bool = False


def _pose_to_matrix(pose: api.models.Pose | None) -> np.ndarray:
    """Build a 4x4 matrix from a NOVA pose, defaulting to identity."""
    if pose is None:
        return np.eye(4)
    return DHRobot([], api.models.Pose()).pose_to_matrix(pose)


def _rotvec_to_matrix(orientation: api.models.RotationVector | None) -> np.ndarray:
    """Convert a rotation-vector orientation to a 3x3 matrix."""
    if orientation is None:
        return np.eye(3)
    vec = np.asarray(orientation.root, dtype=float)
    angle = float(np.linalg.norm(vec))
    if angle < 1e-12:
        return np.eye(3)
    return Rotation.from_rotvec(vec).as_matrix()


def _collider_pose_matrix(collider: api.models.Collider) -> np.ndarray:
    """Return the world transform of a collider, taking its local pose into account."""
    return _pose_to_matrix(collider.pose)


def _make_entity_mesh(collider: api.models.Collider) -> trimesh.Trimesh | None:
    """Create an untransformed trimesh for a collider, or None for unsupported shapes."""
    shape = collider.shape

    if isinstance(shape, api.models.Box):
        extents = [float(shape.size_x), float(shape.size_y), float(shape.size_z)]
        return trimesh.creation.box(extents=extents)

    if isinstance(shape, api.models.Sphere):
        return trimesh.creation.icosphere(radius=float(shape.radius), subdivisions=2)

    if isinstance(shape, api.models.Cylinder):
        return trimesh.creation.cylinder(
            radius=float(shape.radius), height=float(shape.height), sections=24
        )

    if isinstance(shape, api.models.Capsule):
        return trimesh.creation.capsule(
            radius=float(shape.radius), height=float(shape.cylinder_height), count=[12, 16]
        )

    if isinstance(shape, api.models.ConvexHull):
        vertices = np.asarray([v.root for v in shape.vertices], dtype=float)
        if len(vertices) < 4:
            return None
        return trimesh.convex.convex_hull(vertices)

    if isinstance(shape, api.models.Rectangle):
        return trimesh.creation.box(extents=[float(shape.size_x), float(shape.size_y), 1.0])

    if isinstance(shape, api.models.RectangularCapsule):
        return _make_rectangular_capsule_mesh(
            radius=float(shape.radius),
            distance_x=float(shape.sphere_center_distance_x),
            distance_y=float(shape.sphere_center_distance_y),
        )

    return None


def _make_rectangular_capsule_mesh(
    radius: float, distance_x: float, distance_y: float
) -> trimesh.Trimesh:
    """Create a rectangular capsule as the convex hull of four corner spheres."""
    centers = np.array(
        [
            [distance_x / 2, distance_y / 2, 0.0],
            [distance_x / 2, -distance_y / 2, 0.0],
            [-distance_x / 2, distance_y / 2, 0.0],
            [-distance_x / 2, -distance_y / 2, 0.0],
        ]
    )
    all_points: list[np.ndarray] = []
    for center in centers:
        sphere = trimesh.creation.icosphere(radius=radius, subdivisions=2)
        all_points.append(sphere.vertices + center)
    points = np.vstack(all_points)
    return trimesh.convex.convex_hull(points)


def collider_to_rerun(
    collider: api.models.Collider,
    color: tuple[int, int, int, int] = DEFAULT_SCENE_COLOR,
    highlight_color: tuple[int, int, int, int] = COLLISION_HIGHLIGHT_COLOR,
    highlight: bool = False,
) -> tuple[list[rr.LineStrips3D] | None, rr.Mesh3D | rr.Boxes3D | rr.Ellipsoids3D | None]:
    """Convert a NOVA collider into Rerun primitives.

    Returns a tuple of (optional wireframe, solid primitive). The wireframe is returned as a
    list so callers can log it under a ``.../wireframe`` child path. Primitives are returned
    in the collider's local frame and must be placed via ``transform``.

    Args:
        collider: The NOVA collider to convert.
        color: Default RGBA color for non-highlighted rendering.
        highlight_color: RGBA color to use when ``highlight`` is True.
        highlight: Whether this collider is part of a collision pair.

    Returns:
        ``(wireframe_primitives, solid_primitive)`` where either may be None.
    """
    effective_color = highlight_color if highlight else color

    shape = collider.shape

    if isinstance(shape, api.models.Sphere):
        radii = [float(shape.radius), float(shape.radius), float(shape.radius)]
        solid = rr.Ellipsoids3D(radii=radii, centers=[[0.0, 0.0, 0.0]], colors=[effective_color])
        return (None, solid)

    if isinstance(shape, api.models.Box):
        solid = rr.Boxes3D(
            centers=[[0.0, 0.0, 0.0]],
            sizes=[float(shape.size_x), float(shape.size_y), float(shape.size_z)],
            colors=[effective_color],
        )
        return (None, solid)

    if isinstance(shape, api.models.Rectangle):
        half_x = float(shape.size_x) / 2
        half_y = float(shape.size_y) / 2
        vertices = np.array(
            [
                [-half_x, -half_y, 0.0],
                [half_x, -half_y, 0.0],
                [half_x, half_y, 0.0],
                [-half_x, half_y, 0.0],
            ]
        )
        loop = np.vstack([vertices, vertices[0]])
        wire = rr.LineStrips3D([loop.tolist()], colors=[effective_color])
        solid = rr.Boxes3D(
            centers=[[0.0, 0.0, 0.0]],
            sizes=[float(shape.size_x), float(shape.size_y), 1.0],
            colors=[effective_color],
        )
        return ([wire], solid)

    if isinstance(shape, api.models.Plane):
        # Represent an infinite plane as a large, thin rectangle.
        size = 5000.0
        wire = rr.LineStrips3D(
            [
                [
                    [-size, -size, 0.0],
                    [size, -size, 0.0],
                    [size, size, 0.0],
                    [-size, size, 0.0],
                    [-size, -size, 0.0],
                ]
            ],
            colors=[effective_color],
        )
        solid = rr.Boxes3D(
            centers=[[0.0, 0.0, 0.0]], sizes=[size * 2, size * 2, 1.0], colors=[effective_color]
        )
        return ([wire], solid)

    mesh = _make_entity_mesh(collider)
    if mesh is None:
        logger.warning("Unsupported collider shape type: %s", type(shape).__name__)
        return (None, None)

    polygons = HullVisualizer.compute_hull_outlines_from_points(np.array(mesh.vertices))
    wire_primitives: list[rr.LineStrips3D] = []
    if polygons:
        wire_primitives.append(
            rr.LineStrips3D([p.tolist() for p in polygons], colors=[effective_color])
        )

    solid = rr.Mesh3D(
        vertex_positions=mesh.vertices.tolist(),
        triangle_indices=mesh.faces.tolist(),
        vertex_normals=mesh.vertex_normals.tolist(),
        albedo_factor=effective_color[:3],
    )
    return (wire_primitives, solid)


def log_entity(entity: SceneEntity, static: bool = True) -> None:
    """Log a single scene entity to Rerun.

    Args:
        entity: The scene entity to draw.
        static: Whether the entity should be logged as static (default) or temporal.
    """
    wire_primitives, solid = collider_to_rerun(entity.collider, highlight=entity.highlight)

    if wire_primitives is not None:
        for i, wire in enumerate(wire_primitives):
            path = (
                f"{entity.entity_path}/wireframe"
                if len(wire_primitives) == 1
                else f"{entity.entity_path}/wireframe/{i}"
            )
            rr.log(path, wire, static=static)
            rr.log(
                path,
                rr.Transform3D(
                    mat3x3=entity.transform[:3, :3], translation=entity.transform[:3, 3]
                ),
                static=static,
            )

    if solid is not None:
        rr.log(entity.entity_path, solid, static=static)
        rr.log(
            entity.entity_path,
            rr.Transform3D(mat3x3=entity.transform[:3, :3], translation=entity.transform[:3, 3]),
            static=static,
        )


class RobotStateScene:
    """A static scene that places a robot and its environment colliders in world space.

    The scene is built from a motion-group description, a joint configuration, and an
    optional collision setup. It produces a list of :class:`SceneEntity` objects that can
    be logged with :func:`log_entity` or inspected for collisions before logging.

    Args:
        motion_group_description: Description containing DH parameters and safety/collision
            geometry for the robot.
        collision_setup: Optional collision setup with environment colliders and robot
            collision geometry.
        base_entity_path: Rerun entity path prefix for the scene.
    """

    def __init__(
        self,
        motion_group_description: api.models.MotionGroupDescription,
        collision_setup: api.models.CollisionSetup | None = None,
        base_entity_path: str = "scene",
    ) -> None:
        self.base_entity_path = base_entity_path.rstrip("/")
        self.dh_parameters = motion_group_description.dh_parameters or []
        self.mounting = motion_group_description.mounting or api.models.Pose(
            position=api.models.Vector3d([0, 0, 0]),
            orientation=api.models.RotationVector([0, 0, 0]),
        )
        self.robot = DHRobot(dh_parameters=self.dh_parameters, mounting=self.mounting)

        # Safety geometry from the controller (always available, lower fidelity).
        self.safety_link_colliders: dict[int, list[api.models.Collider]] = {}
        if motion_group_description.safety_link_colliders is not None:
            for link_index, collider_dict in enumerate(
                motion_group_description.safety_link_colliders
            ):
                self.safety_link_colliders[link_index] = list(collider_dict.root.values())

        self.safety_tcp_colliders: dict[str, dict[str, api.models.Collider]] = {}
        if motion_group_description.safety_tool_colliders is not None:
            for tcp_name, collider_dict in motion_group_description.safety_tool_colliders.items():
                self.safety_tcp_colliders[tcp_name] = dict(collider_dict.root)

        # Collision geometry from the optional collision setup (higher fidelity).
        self.collision_link_chain: list[dict[str, api.models.Collider]] = []
        self.collision_tool: dict[str, api.models.Collider] = {}
        if collision_setup is not None:
            if collision_setup.link_chain is not None:
                for link in collision_setup.link_chain.root:
                    self.collision_link_chain.append(dict(link.root))
            if collision_setup.tool is not None:
                self.collision_tool = dict(collision_setup.tool.root)

        self.environment_colliders: dict[str, api.models.Collider] = {}
        if collision_setup is not None and collision_setup.colliders is not None:
            self.environment_colliders = dict(collision_setup.colliders.root)

    def build_entities(
        self,
        joint_positions: list[float] | tuple[float, ...] | None = None,
        tcp_name: str = "Flange",
        show_safety_geometry: bool = True,
        show_collision_geometry: bool = True,
        show_environment: bool = True,
    ) -> list[SceneEntity]:
        """Build the scene entities for a robot state.

        Args:
            joint_positions: Joint values to pose the robot. Defaults to all zeros.
            tcp_name: TCP name whose geometry should be drawn.
            show_safety_geometry: Whether to include controller-reported safety geometry.
            show_collision_geometry: Whether to include collision-setup robot geometry.
            show_environment: Whether to include environment colliders from the setup.

        Returns:
            A list of scene entities in world space.
        """
        if joint_positions is None:
            joint_positions = [0.0] * len(self.dh_parameters)

        link_transforms = self.robot.compute_forward_kinematics(list(joint_positions))
        entities: list[SceneEntity] = []

        # Environment colliders live directly in world space.
        if show_environment:
            for collider_id, collider in self.environment_colliders.items():
                entities.append(
                    SceneEntity(
                        entity_path=f"{self.base_entity_path}/environment/{rr.escape_entity_path_part(collider_id)}",
                        transform=_collider_pose_matrix(collider),
                        collider=collider,
                    )
                )

        # Safety link geometry (from controller).
        if show_safety_geometry:
            for link_index, colliders in self.safety_link_colliders.items():
                if link_index >= len(link_transforms):
                    continue
                link_transform = link_transforms[link_index]
                for geom_index, collider in enumerate(colliders):
                    local_transform = _pose_to_matrix(collider.pose)
                    entities.append(
                        SceneEntity(
                            entity_path=f"{self.base_entity_path}/safety/links/link_{link_index}/geometry_{geom_index}",
                            transform=link_transform @ local_transform,
                            collider=collider,
                        )
                    )

            tcp_colliders = self.safety_tcp_colliders.get(tcp_name, {})
            if tcp_colliders:
                tcp_transform = link_transforms[-1]
                for collider_id, collider in tcp_colliders.items():
                    local_transform = _pose_to_matrix(collider.pose)
                    entities.append(
                        SceneEntity(
                            entity_path=f"{self.base_entity_path}/safety/tcp/{rr.escape_entity_path_part(collider_id)}",
                            transform=tcp_transform @ local_transform,
                            collider=collider,
                        )
                    )

        # Collision link geometry (from collision setup).
        if show_collision_geometry:
            for link_index, colliders in enumerate(self.collision_link_chain):
                if link_index >= len(link_transforms):
                    continue
                link_transform = link_transforms[link_index]
                for collider_id, collider in colliders.items():
                    local_transform = _pose_to_matrix(collider.pose)
                    entities.append(
                        SceneEntity(
                            entity_path=f"{self.base_entity_path}/collision/links/link_{link_index}/{rr.escape_entity_path_part(collider_id)}",
                            transform=link_transform @ local_transform,
                            collider=collider,
                        )
                    )

            if self.collision_tool:
                tcp_transform = link_transforms[-1]
                for collider_id, collider in self.collision_tool.items():
                    local_transform = _pose_to_matrix(collider.pose)
                    entities.append(
                        SceneEntity(
                            entity_path=f"{self.base_entity_path}/collision/tcp/{rr.escape_entity_path_part(collider_id)}",
                            transform=tcp_transform @ local_transform,
                            collider=collider,
                        )
                    )

        return entities


def log_scene(
    entities: list[SceneEntity], static: bool = True, clear_existing: bool = False
) -> None:
    """Log a list of scene entities to Rerun.

    Args:
        entities: Entities to log.
        static: Whether to log as static data.
        clear_existing: If True, clear any existing data under the common base paths.
    """
    if clear_existing and entities:
        base = entities[0].entity_path.split("/")[0]
        rr.log(base, rr.Clear(recursive=True))

    for entity in entities:
        log_entity(entity, static=static)
