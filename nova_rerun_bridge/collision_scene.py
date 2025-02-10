from typing import Any, Dict, List, Tuple

import numpy as np
import rerun as rr
import trimesh
from scipy.spatial.transform import Rotation

from nova.api import models
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
            # Convert rotation vector to axis-angle format
            rot_vec = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z])
            angle = np.linalg.norm(rot_vec)
            if angle > 0:
                axis = rot_vec / angle
            else:
                axis = np.array([0.0, 0.0, 1.0])
                angle = 0.0

            rr.log(
                f"{entity_path}/{collider_id}",
                rr.Ellipsoids3D(
                    radii=[
                        collider.shape.actual_instance.radius,
                        collider.shape.actual_instance.radius,
                        collider.shape.actual_instance.radius,
                    ],
                    centers=[[pose.position.x, pose.position.y, pose.position.z]],
                    rotation_axis_angles=[[*axis, angle]],
                    colors=[(221, 193, 193, 255)],
                ),
                timeless=True,
            )

        elif collider.shape.actual_instance.shape_type == "rectangular_capsule":
            # Get parameters from the capsule
            radius = collider.shape.actual_instance.radius
            size_x = collider.shape.actual_instance.sphere_center_distance_x
            size_y = collider.shape.actual_instance.sphere_center_distance_y

            # Create sphere centers at the corners
            sphere_centers = np.array(
                [
                    [size_x / 2, size_y / 2, 0],  # top right
                    [size_x / 2, -size_y / 2, 0],  # bottom right
                    [-size_x / 2, size_y / 2, 0],  # top left
                    [-size_x / 2, -size_y / 2, 0],  # bottom left
                ]
            )

            # Create vertices for spheres at each corner
            vertices = []
            for center in sphere_centers:
                # Create a low-poly sphere at each corner
                sphere = trimesh.creation.icosphere(radius=radius, subdivisions=2)
                sphere_verts = sphere.vertices + center
                vertices.extend(sphere_verts)

            # Convert to numpy array for transformation
            vertices = np.array(vertices)

            # Transform vertices to world position
            transform = np.eye(4)
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            rot_mat = Rotation.from_rotvec(
                np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z])
            ).as_matrix()
            transform[:3, :3] = rot_mat
            vertices = np.array([transform @ np.append(v, 1) for v in vertices])[:, :3]

            # Create hull visualization
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

        elif collider.shape.actual_instance.shape_type == "rectangle":
            # Create vertices for a rectangle in XY plane
            half_x = collider.shape.actual_instance.size_x / 2
            half_y = collider.shape.actual_instance.size_y / 2
            vertices = np.array(
                [
                    [-half_x, -half_y, 0],  # bottom left
                    [half_x, -half_y, 0],  # bottom right
                    [half_x, half_y, 0],  # top right
                    [-half_x, half_y, 0],  # top left
                ]
            )

            # Transform vertices
            transform = np.eye(4)
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            rot_mat = Rotation.from_rotvec(
                np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z])
            ).as_matrix()
            transform[:3, :3] = rot_mat
            vertices = np.array([transform @ np.append(v, 1) for v in vertices])[:, :3]

            # Create line segments for the rectangle outline
            line_segments = [
                [vertices[0].tolist(), vertices[1].tolist()],
                [vertices[1].tolist(), vertices[2].tolist()],
                [vertices[2].tolist(), vertices[3].tolist()],
                [vertices[3].tolist(), vertices[0].tolist()],
            ]

            rr.log(
                f"{entity_path}/{collider_id}",
                rr.LineStrips3D(
                    line_segments, radii=rr.Radius.ui_points(0.75), colors=[[221, 193, 193, 255]]
                ),
                static=True,
                timeless=True,
            )

        elif collider.shape.actual_instance.shape_type == "box":
            # Create rotation matrix from orientation
            rot_mat = Rotation.from_rotvec(
                np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z])
            ).as_matrix()

            # Create box vertices
            half_sizes = [
                collider.shape.actual_instance.size_x / 2,
                collider.shape.actual_instance.size_y / 2,
                collider.shape.actual_instance.size_z / 2,
            ]
            box = trimesh.creation.box(extents=[s * 2 for s in half_sizes])
            vertices = np.array(box.vertices)

            # Transform vertices
            transform = np.eye(4)
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            transform[:3, :3] = rot_mat
            vertices = np.array([transform @ np.append(v, 1) for v in vertices])[:, :3]

            # Create hull visualization
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

        elif collider.shape.actual_instance.shape_type == "cylinder":
            height = collider.shape.actual_instance.height
            radius = collider.shape.actual_instance.radius

            # Generate trimesh capsule
            cylinder = trimesh.creation.cylinder(height=height, radius=radius)

            # Extract vertices and faces for solid visualization
            vertices = np.array(cylinder.vertices)

            # Transform vertices to world position
            transform = np.eye(4)
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
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

        elif collider.shape.actual_instance.shape_type == "capsule":
            height = collider.shape.actual_instance.cylinder_height
            radius = collider.shape.actual_instance.radius

            # Generate trimesh capsule
            capsule = trimesh.creation.capsule(height=height, radius=radius, count=[6, 8])

            # Extract vertices and faces for solid visualization
            vertices = np.array(capsule.vertices)

            # Transform vertices to world position
            transform = np.eye(4)
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
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
