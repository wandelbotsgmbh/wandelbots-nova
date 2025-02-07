from typing import Any, Dict, List, Tuple

import numpy as np
import rerun as rr
import trimesh
from nova.api import models
from scipy.spatial.transform import Rotation

from nova_rerun_bridge import colors
from nova_rerun_bridge.conversion_helpers import normalize_pose
from nova_rerun_bridge.hull_visualizer import HullVisualizer


def log_collision_scenes(collision_scenes: Dict[str, models.CollisionScene]):
    for scene_id, scene in collision_scenes.items():
        entity_path = f"collision_scenes/{scene_id}"
        for collider_id, collider in scene.colliders.items():
            log_colliders_once(entity_path, {collider_id: collider})


def log_colliders_once(entity_path: str, colliders: Dict[str, models.Collider]):
    for collider_id, collider in colliders.items():
        pose = normalize_pose(collider.pose)

        if collider.shape.actual_instance.shape_type == "sphere":
            rr.log(
                f"{entity_path}/{collider_id}",
                rr.Ellipsoids3D(
                    radii=[
                        collider.shape.actual_instance.radius,
                        collider.shape.actual_instance.radius,
                        collider.shape.actual_instance.radius,
                    ],
                    centers=[[pose.position.x, pose.position.y, pose.position.z]],
                    colors=[(221, 193, 193, 255)],
                ),
                timeless=True,
            )

        elif collider.shape.actual_instance.shape_type == "box":
            rr.log(
                f"{entity_path}/{collider_id}",
                rr.Boxes3D(
                    centers=[[pose.position.x, pose.position.y, pose.position.z]],
                    half_sizes=[
                        collider.shape.actual_instance.size_x / 2,
                        collider.shape.actual_instance.size_y / 2,
                        collider.shape.actual_instance.size_z / 2,
                    ],
                    colors=[(221, 193, 193, 255)],
                ),
                timeless=True,
            )

        elif collider.shape.actual_instance.shape_type == "capsule":
            height = collider.shape.actual_instance.cylinder_height
            radius = collider.shape.actual_instance.radius

            # Generate trimesh capsule
            capsule = trimesh.creation.capsule(height=height, radius=radius, count=[6, 8])

            # Extract vertices and faces for solid visualization
            vertices = np.array(capsule.vertices)

            # Transform vertices to world position
            transform = np.eye(4)
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z - height / 2]
            rot_mat = Rotation.from_rotvec(
                np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z])
            )
            transform[:3, :3] = rot_mat.as_matrix()

            vertices = np.array([transform @ np.append(v, 1) for v in vertices])[:, :3]

            polygons = HullVisualizer.compute_hull_outlines_from_points(vertices)

            if polygons:
                line_segments = [p.tolist() for p in polygons]
                rr.log(
                    f"{entity_path}/{collider_id}",
                    rr.LineStrips3D(
                        line_segments,
                        radii=rr.Radius.ui_points(0.75),
                        colors=[[221, 193, 193, 255]],
                    ),
                    static=True,
                    timeless=True,
                )

        elif collider.shape.actual_instance.shape_type == "convex_hull":
            polygons = HullVisualizer.compute_hull_outlines_from_points(
                collider.shape.actual_instance.vertices
            )

            if polygons:
                line_segments = [p.tolist() for p in polygons]
                rr.log(
                    f"{entity_path}/{collider_id}",
                    rr.LineStrips3D(
                        line_segments, radii=rr.Radius.ui_points(1.5), colors=[colors.colors[2]]
                    ),
                    static=True,
                    timeless=True,
                )

                vertices, triangles, normals = HullVisualizer.compute_hull_mesh(polygons)

                rr.log(
                    f"{entity_path}/{collider_id}",
                    rr.Mesh3D(
                        vertex_positions=vertices,
                        triangle_indices=triangles,
                        vertex_normals=normals,
                        albedo_factor=[colors.colors[0]],
                    ),
                    static=True,
                    timeless=True,
                )


def extract_link_chain_and_tcp(collision_scenes: dict) -> Tuple[List[Any], List[Any]]:
    """Extract link chain and TCP from collision scenes."""
    # Get first scene (name can vary)
    scene = next(iter(collision_scenes.values()), None)
    if not scene:
        return [], []

    # Try to get motion groups
    motion_group = next(iter(scene.motion_groups.values()), None)
    if not motion_group:
        return [], []

    return (getattr(motion_group, "link_chain", []), getattr(motion_group, "tool", []))
