import numpy as np
import rerun as rr
import trimesh
from scipy.spatial.transform import Rotation

from nova import api
from nova.types import Pose
from nova_rerun_bridge import colors
from nova_rerun_bridge.hull_visualizer import HullVisualizer


def log_collision_setups(collision_setups: dict[str, api.models.CollisionSetup]):
    for setup_id, setup in collision_setups.items():
        entity_path = f"collision_setups/{setup_id}"
        if setup.colliders:
            for collider_id, collider in setup.colliders.root.items():
                log_colliders_once(entity_path, {collider_id: collider})


def log_colliders_once(entity_path: str, colliders: dict[str, api.models.Collider]):
    for collider_id, collider in colliders.items():
        pose = Pose(collider.pose)

        if isinstance(collider.shape, api.models.Sphere):
            # Convert rotation vector to axis-angle format
            rot_vec = np.array(pose.orientation.to_tuple())
            angle = np.linalg.norm(rot_vec)
            if angle > 0:
                axis = rot_vec / angle
            else:
                axis = np.array([0.0, 0.0, 1.0])
                angle = 0.0  # type: ignore[assignment]

            rr.log(
                f"{entity_path}/{collider_id}",
                rr.Ellipsoids3D(
                    radii=[collider.shape.radius, collider.shape.radius, collider.shape.radius],
                    centers=[pose.position.to_tuple()],
                    rotation_axis_angles=[rr.RotationAxisAngle(axis=axis, angle=angle)],  # type: ignore
                    colors=[(221, 193, 193, 255)],
                ),
                static=True,
            )

        elif isinstance(collider.shape, api.models.RectangularCapsule):
            # Get parameters from the capsule
            radius = collider.shape.radius
            size_x = collider.shape.sphere_center_distance_x
            size_y = collider.shape.sphere_center_distance_y

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
            vertices = np.empty((0, 3))
            for center in sphere_centers:
                # Create a low-poly sphere at each corner
                sphere = trimesh.creation.icosphere(radius=radius, subdivisions=2)
                sphere_verts = sphere.vertices + center
                vertices = np.concatenate([vertices, sphere_verts])

            # Convert to numpy array for transformation
            vertices = np.array(vertices)

            # Transform vertices to world position
            transform = np.eye(4)
            transform[:3, 3] = pose.position.to_tuple()
            rot_mat = Rotation.from_rotvec(np.array(pose.orientation.to_tuple())).as_matrix()
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
                )

        elif isinstance(collider.shape, api.models.Rectangle):
            # Create vertices for a rectangle in XY plane
            half_x = collider.shape.size_x / 2
            half_y = collider.shape.size_y / 2
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
            transform[:3, 3] = pose.position.to_tuple()
            rot_mat = Rotation.from_rotvec(np.array(pose.orientation.to_tuple())).as_matrix()
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
            )

        elif isinstance(collider.shape, api.models.Box):
            # Create rotation matrix from orientation
            rot_mat = Rotation.from_rotvec(np.array(pose.orientation.to_tuple())).as_matrix()

            # Create box vertices
            half_sizes = [
                collider.shape.size_x / 2,
                collider.shape.size_y / 2,
                collider.shape.size_z / 2,
            ]
            box = trimesh.creation.box(extents=[s * 2 for s in half_sizes])
            vertices = np.array(box.vertices)

            # Transform vertices
            transform = np.eye(4)
            transform[:3, 3] = pose.position.to_tuple()
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
                )

        elif isinstance(collider.shape, api.models.Cylinder):
            height = collider.shape.height
            radius = collider.shape.radius

            # Generate trimesh capsule
            cylinder = trimesh.creation.cylinder(height=height, radius=radius)

            # Extract vertices and faces for solid visualization
            vertices = np.array(cylinder.vertices)

            # Transform vertices to world position
            transform = np.eye(4)
            transform[:3, 3] = pose.position.to_tuple()
            rot_mat = Rotation.from_rotvec(np.array(pose.orientation.to_tuple()))
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
                )

        elif isinstance(collider.shape, api.models.Capsule):
            height = collider.shape.cylinder_height
            radius = collider.shape.radius

            # Generate trimesh capsule
            capsule = trimesh.creation.capsule(height=height, radius=radius, count=[6, 8])

            # Extract vertices and faces for solid visualization
            vertices = np.array(capsule.vertices)

            # Transform vertices to world position
            transform = np.eye(4)
            transform[:3, 3] = pose.position.to_tuple()
            rot_mat = Rotation.from_rotvec(np.array(pose.orientation.to_tuple()))
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
                )

        elif isinstance(collider.shape, api.models.ConvexHull):
            # Transform vertices to world position
            vertices = np.array(collider.shape.vertices)
            transform = np.eye(4)
            transform[:3, 3] = pose.position.to_tuple()
            rot_mat = Rotation.from_rotvec(np.array(pose.orientation.to_tuple())).as_matrix()
            transform[:3, :3] = rot_mat

            # Apply transformation
            vertices = np.array([transform @ np.append(v, 1) for v in vertices])[:, :3]

            polygons = HullVisualizer.compute_hull_outlines_from_points(vertices)

            if polygons:
                line_segments = [p.tolist() for p in polygons]
                rr.log(
                    f"{entity_path}/{collider_id}",
                    rr.LineStrips3D(
                        line_segments, radii=rr.Radius.ui_points(1.5), colors=[colors.colors[2]]
                    ),
                    static=True,
                )

                vertices, triangles, normals = HullVisualizer.compute_hull_mesh(polygons)  # type: ignore

                rr.log(
                    f"{entity_path}/{collider_id}",
                    rr.Mesh3D(
                        vertex_positions=vertices,
                        triangle_indices=triangles,
                        vertex_normals=normals,
                        albedo_factor=[colors.colors[0]],  # type: ignore
                    ),
                    static=True,
                )


def extract_link_chain_and_tcp(
    collision_setups: dict[str, api.models.CollisionSetup],
) -> tuple[api.models.LinkChain | None, api.models.Tool | None]:
    """Extract link chain and TCP from collision scenes.
    Searches through all scenes for a matching motion group.
    """
    # Iterate through all scenes
    for setup_name, setup in collision_setups.items():
        # Check if this scene has the motion group we're looking for
        return setup.link_chain, setup.tool

    # If no matching motion group is found in any scene
    return None, None
