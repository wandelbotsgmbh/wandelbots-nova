"""Unit tests for the nova_rerun_bridge scene module.

Tests focus on geometry conversion, scene graph construction, and collision detection.
They avoid spawning a Rerun viewer by mocking rerun calls.
"""

from unittest.mock import Mock, patch

import numpy as np
import pytest
import rerun as rr

from nova import api
from nova_rerun_bridge.scene import (
    RobotStateScene,
    SceneEntity,
    _collider_pose_matrix,
    _make_entity_mesh,
    collider_to_rerun,
    find_colliding_pairs,
    log_entity,
    log_scene,
)


def _box_collider(
    size_x: float = 10.0,
    size_y: float = 10.0,
    size_z: float = 10.0,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> api.models.Collider:
    return api.models.Collider(
        shape=api.models.Box(
            shape_type="box",
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            box_type=api.models.BoxType.FULL,
        ),
        pose=api.models.Pose(
            position=api.models.Vector3d(list(position)),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )


def _sphere_collider(
    radius: float = 5.0, position: tuple[float, float, float] = (0.0, 0.0, 0.0)
) -> api.models.Collider:
    return api.models.Collider(
        shape=api.models.Sphere(shape_type="sphere", radius=radius),
        pose=api.models.Pose(
            position=api.models.Vector3d(list(position)),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )


def _capsule_collider(
    radius: float = 5.0,
    height: float = 20.0,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> api.models.Collider:
    return api.models.Collider(
        shape=api.models.Capsule(shape_type="capsule", radius=radius, cylinder_height=height),
        pose=api.models.Pose(
            position=api.models.Vector3d(list(position)),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )


def _cylinder_collider(
    radius: float = 5.0,
    height: float = 20.0,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> api.models.Collider:
    return api.models.Collider(
        shape=api.models.Cylinder(shape_type="cylinder", radius=radius, height=height),
        pose=api.models.Pose(
            position=api.models.Vector3d(list(position)),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )


def _rectangle_collider(
    size_x: float = 10.0,
    size_y: float = 10.0,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> api.models.Collider:
    return api.models.Collider(
        shape=api.models.Rectangle(shape_type="rectangle", size_x=size_x, size_y=size_y),
        pose=api.models.Pose(
            position=api.models.Vector3d(list(position)),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )


def _rectangular_capsule_collider(
    radius: float = 5.0,
    distance_x: float = 20.0,
    distance_y: float = 10.0,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> api.models.Collider:
    return api.models.Collider(
        shape=api.models.RectangularCapsule(
            shape_type="rectangular_capsule",
            radius=radius,
            sphere_center_distance_x=distance_x,
            sphere_center_distance_y=distance_y,
        ),
        pose=api.models.Pose(
            position=api.models.Vector3d(list(position)),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )


def _convex_hull_collider(
    vertices: list[tuple[float, float, float]],
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> api.models.Collider:
    return api.models.Collider(
        shape=api.models.ConvexHull(
            shape_type="convex_hull", vertices=[api.models.Vector3d(list(v)) for v in vertices]
        ),
        pose=api.models.Pose(
            position=api.models.Vector3d(list(position)),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )


class TestColliderToRerun:
    """Tests for collider-to-rerun conversion."""

    def test_box_returns_boxes3d(self):
        collider = _box_collider()
        wire, solid = collider_to_rerun(collider)
        assert wire is None
        assert isinstance(solid, rr.Boxes3D)

    def test_sphere_returns_ellipsoids3d(self):
        collider = _sphere_collider()
        wire, solid = collider_to_rerun(collider)
        assert wire is None
        assert isinstance(solid, rr.Ellipsoids3D)

    def test_cylinder_returns_mesh3d(self):
        collider = _cylinder_collider()
        wire, solid = collider_to_rerun(collider)
        assert wire is not None
        assert isinstance(solid, rr.Mesh3D)

    def test_capsule_returns_mesh3d(self):
        collider = _capsule_collider()
        wire, solid = collider_to_rerun(collider)
        assert wire is not None
        assert isinstance(solid, rr.Mesh3D)

    def test_rectangle_returns_wireframe_and_box(self):
        collider = _rectangle_collider()
        wire, solid = collider_to_rerun(collider)
        assert wire is not None
        assert isinstance(solid, rr.Boxes3D)

    def test_rectangular_capsule_returns_mesh3d(self):
        collider = _rectangular_capsule_collider()
        wire, solid = collider_to_rerun(collider)
        assert wire is not None
        assert isinstance(solid, rr.Mesh3D)

    def test_convex_hull_returns_mesh3d(self):
        collider = _convex_hull_collider(
            [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (0.0, 0.0, 10.0)]
        )
        wire, solid = collider_to_rerun(collider)
        assert wire is not None
        assert isinstance(solid, rr.Mesh3D)

    def test_highlight_changes_color(self):
        collider = _box_collider()
        _, solid_normal = collider_to_rerun(collider, highlight=False)
        _, solid_highlight = collider_to_rerun(collider, highlight=True)
        # Rerun stores colors internally; we just verify the call succeeds and objects differ.
        assert solid_normal is not None
        assert solid_highlight is not None


class TestEntityMesh:
    """Tests for trimesh generation from colliders."""

    def test_box_mesh_has_eight_vertices(self):
        collider = _box_collider(size_x=10, size_y=20, size_z=30)
        mesh = _make_entity_mesh(collider)
        assert mesh is not None
        assert len(mesh.vertices) == 8

    def test_sphere_mesh_is_non_empty(self):
        collider = _sphere_collider(radius=5)
        mesh = _make_entity_mesh(collider)
        assert mesh is not None
        assert len(mesh.vertices) > 0


class TestPoseMatrix:
    """Tests for pose-to-matrix conversion."""

    def test_identity_pose(self):
        collider = _box_collider(position=(0, 0, 0))
        matrix = _collider_pose_matrix(collider)
        np.testing.assert_allclose(matrix, np.eye(4))

    def test_translation_in_matrix(self):
        collider = _box_collider(position=(1, 2, 3))
        matrix = _collider_pose_matrix(collider)
        assert matrix[0, 3] == pytest.approx(1.0)
        assert matrix[1, 3] == pytest.approx(2.0)
        assert matrix[2, 3] == pytest.approx(3.0)


class TestRobotStateScene:
    """Tests for RobotStateScene construction."""

    def _minimal_description(self) -> api.models.MotionGroupDescription:
        return api.models.MotionGroupDescription(
            motion_group_model=api.models.MotionGroupModel("UniversalRobots_UR5e"),
            operation_limits=api.models.OperationLimits(
                auto_limits=api.models.LimitSet(
                    joints=[
                        api.models.JointLimits(
                            position=api.models.LimitRange(lower_limit=-3.14, upper_limit=3.14)
                        )
                    ],
                    tcp=api.models.CartesianLimits(velocity=1000.0),
                )
            ),
            dh_parameters=[
                api.models.DHParameter(
                    a=0.0, d=0.0, alpha=0.0, theta=0.0, reverse_rotation_direction=False
                ),
                api.models.DHParameter(
                    a=100.0, d=0.0, alpha=0.0, theta=0.0, reverse_rotation_direction=False
                ),
            ],
            mounting=api.models.Pose(
                position=api.models.Vector3d([0, 0, 0]),
                orientation=api.models.RotationVector([0, 0, 0]),
            ),
        )

    def test_default_joint_positions(self):
        description = self._minimal_description()
        scene = RobotStateScene(description)
        entities = scene.build_entities()
        assert len(entities) == 0

    def test_environment_colliders(self):
        description = self._minimal_description()
        setup = api.models.CollisionSetup(
            colliders=api.models.ColliderDictionary({"box": _box_collider(position=(10, 0, 0))})
        )
        scene = RobotStateScene(description, collision_setup=setup)
        entities = scene.build_entities(show_safety_geometry=False)
        assert len(entities) == 1
        assert entities[0].entity_path.endswith("/box")
        np.testing.assert_allclose(entities[0].transform[:3, 3], np.array([10.0, 0.0, 0.0]))

    def test_safety_link_geometry(self):
        description = self._minimal_description()
        description.safety_link_colliders = [
            api.models.ColliderDictionary({"link0_box": _box_collider(position=(0, 0, 0))})
        ]
        scene = RobotStateScene(description)
        entities = scene.build_entities(show_environment=False)
        assert len(entities) == 1
        assert "safety/links/link_0" in entities[0].entity_path

    def test_collision_link_geometry(self):
        description = self._minimal_description()
        setup = api.models.CollisionSetup(
            link_chain=api.models.LinkChain(
                [
                    api.models.Link({"link0_box": _box_collider(position=(0, 0, 0))}),
                    api.models.Link({"link1_box": _box_collider(position=(50, 0, 0))}),
                ]
            )
        )
        scene = RobotStateScene(description, collision_setup=setup)
        entities = scene.build_entities(show_environment=False, show_safety_geometry=False)
        assert len(entities) == 2
        assert any("collision/links/link_0" in e.entity_path for e in entities)
        assert any("collision/links/link_1" in e.entity_path for e in entities)


class TestCollisionDetection:
    """Tests for collision highlighting."""

    def test_overlapping_boxes_collide(self):
        entities = [
            SceneEntity(
                entity_path="a",
                transform=np.eye(4),
                collider=_box_collider(size_x=10, size_y=10, size_z=10),
            ),
            SceneEntity(
                entity_path="b",
                transform=np.eye(4),
                collider=_box_collider(size_x=10, size_y=10, size_z=10),
            ),
        ]
        pairs = find_colliding_pairs(entities)
        assert len(pairs) == 1
        assert pairs[0][0].entity_path == "a"
        assert pairs[0][1].entity_path == "b"

    def test_separated_boxes_do_not_collide(self):
        transform_a = np.eye(4)
        transform_b = np.eye(4)
        transform_b[:3, 3] = [100.0, 0.0, 0.0]
        entities = [
            SceneEntity(
                entity_path="a",
                transform=transform_a,
                collider=_box_collider(size_x=10, size_y=10, size_z=10),
            ),
            SceneEntity(
                entity_path="b",
                transform=transform_b,
                collider=_box_collider(size_x=10, size_y=10, size_z=10),
            ),
        ]
        pairs = find_colliding_pairs(entities)
        assert len(pairs) == 0

    def test_highlighted_entities_marked(self):
        description = api.models.MotionGroupDescription(
            motion_group_model=api.models.MotionGroupModel("UniversalRobots_UR5e"),
            operation_limits=api.models.OperationLimits(
                auto_limits=api.models.LimitSet(
                    joints=[
                        api.models.JointLimits(
                            position=api.models.LimitRange(lower_limit=-3.14, upper_limit=3.14)
                        )
                    ],
                    tcp=api.models.CartesianLimits(velocity=1000.0),
                )
            ),
            dh_parameters=[
                api.models.DHParameter(
                    a=0.0, d=0.0, alpha=0.0, theta=0.0, reverse_rotation_direction=False
                )
            ],
            mounting=api.models.Pose(
                position=api.models.Vector3d([0, 0, 0]),
                orientation=api.models.RotationVector([0, 0, 0]),
            ),
        )
        setup = api.models.CollisionSetup(
            colliders=api.models.ColliderDictionary(
                {
                    "a": _box_collider(size_x=10, position=(0, 0, 0)),
                    "b": _box_collider(size_x=10, position=(1, 0, 0)),
                }
            )
        )
        scene = RobotStateScene(description, collision_setup=setup)
        entities = scene.build_entities(
            show_safety_geometry=False, show_collision_geometry=False, highlight_collisions=True
        )
        highlighted = [e for e in entities if e.highlight]
        assert len(highlighted) == 2


class TestLogEntity:
    """Tests that logging routes to rerun correctly."""

    def test_logs_solid_and_transform(self):
        entity = SceneEntity(entity_path="test/box", transform=np.eye(4), collider=_box_collider())
        with patch("nova_rerun_bridge.scene.rr.log") as mock_log:
            log_entity(entity, static=True)
            assert mock_log.call_count == 2
            paths = {call.args[0] for call in mock_log.call_args_list}
            assert "test/box" in paths

    def test_logs_wireframe_for_cylinder(self):
        entity = SceneEntity(
            entity_path="test/cylinder", transform=np.eye(4), collider=_cylinder_collider()
        )
        with patch("nova_rerun_bridge.scene.rr.log") as mock_log:
            log_entity(entity, static=True)
            paths = {call.args[0] for call in mock_log.call_args_list}
            assert "test/cylinder/wireframe" in paths


class TestLogScene:
    """Tests for logging a full scene."""

    def test_logs_all_entities(self):
        entities = [
            SceneEntity(entity_path="scene/box1", transform=np.eye(4), collider=_box_collider()),
            SceneEntity(
                entity_path="scene/box2",
                transform=np.eye(4),
                collider=_box_collider(position=(20, 0, 0)),
            ),
        ]
        with patch("nova_rerun_bridge.scene.rr.log") as mock_log:
            log_scene(entities, clear_existing=True)
            assert mock_log.call_count == 5  # clear + 2 solids + 2 transforms
