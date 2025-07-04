import re

import numpy as np
import rerun as rr
import trimesh
from scipy.spatial.transform import Rotation

from nova.api import models
from nova_rerun_bridge import colors
from nova_rerun_bridge.conversion_helpers import normalize_pose
from nova_rerun_bridge.dh_robot import DHRobot
from nova_rerun_bridge.helper_scripts.download_models import get_project_root
from nova_rerun_bridge.hull_visualizer import HullVisualizer


def get_model_path(model_name: str) -> str:
    """Get absolute path to model file in project directory"""
    return str(get_project_root() / "models" / f"{model_name}.glb")


class RobotVisualizer:
    def __init__(
        self,
        robot: DHRobot,
        robot_model_geometries: list[models.RobotLinkGeometry],
        tcp_geometries: list[models.Geometry],
        static_transform: bool = True,
        base_entity_path: str = "robot",
        albedo_factor: list = [255, 255, 255],
        collision_link_chain=None,
        collision_tcp=None,
        model_from_controller="",
        show_collision_link_chain: bool = False,
        show_safety_link_chain: bool = True,
    ):
        """
        :param robot: DHRobot instance
        :param robot_model_geometries: List of geometries for each link
        :param tcp_geometries: TCP geometries (similar structure to link geometries)
        :param static_transform: If True, transforms are logged as static, else temporal.
        :param base_entity_path: A base path prefix for logging the entities (e.g. motion group name)
        :param albedo_factor: A list representing the RGB values [R, G, B] to apply as the albedo factor.
        :param collision_link_chain: Collision link chain geometries for the robot
        :param collision_tcp: Collision TCP geometries
        :param model_from_controller: Model name from controller for loading robot mesh
        :param show_collision_link_chain: Whether to render robot collision mesh geometry
        :param show_safety_link_chain: Whether to render robot safety geometry (from controller)
        """
        self.robot = robot
        self.link_geometries: dict[int, list[models.Geometry]] = {}
        self.tcp_geometries: list[models.Geometry] = tcp_geometries
        self.logged_meshes: set[str] = set()
        self.static_transform = static_transform
        self.base_entity_path = base_entity_path.rstrip("/")
        self.albedo_factor = albedo_factor
        self.mesh_loaded = False
        self.collision_link_geometries = {}
        self.collision_tcp_geometries = collision_tcp
        self.show_collision_link_chain = show_collision_link_chain
        self.show_safety_link_chain = show_safety_link_chain

        # This will hold the names of discovered joints (e.g. ["robot_J00", "robot_J01", ...])
        self.joint_names: list[str] = []
        self.layer_nodes_dict: dict[str, list[str]] = {}
        self.parent_nodes_dict: dict[str, str] = {}

        # load mesh
        try:
            glb_path = get_model_path(model_from_controller)
            self.scene = trimesh.load_scene(glb_path, file_type="glb")
            self.mesh_loaded = True
            self.edge_data = self.scene.graph.transforms.edge_data

            # After loading, auto-discover any child nodes that match *_J0n
            self.discover_joints()
        except Exception as e:
            print(f"Failed to load mesh: {e}")

        # Group geometries by link
        for gm in robot_model_geometries:
            self.link_geometries.setdefault(gm.link_index, []).append(gm.geometry)

        # Group geometries by link
        self.collision_link_geometries = collision_link_chain

    def discover_joints(self):
        """
        Find all child node names that contain '_J0' followed by digits or '_FLG'.
        Store joints with their parent nodes and print layer information.
        """
        joint_pattern = re.compile(r"_J0(\d+)")
        flg_pattern = re.compile(r"_FLG")
        matches = []
        flg_nodes = []
        joint_parents = {}  # Store parent for each joint/FLG

        for (parent, child), data in self.edge_data.items():
            # Check for joints
            joint_match = joint_pattern.search(child)
            if joint_match:
                j_idx = int(joint_match.group(1))
                matches.append((j_idx, child))
                joint_parents[child] = parent

            # Check for FLG
            flg_match = flg_pattern.search(child)
            if flg_match:
                flg_nodes.append(child)
                joint_parents[child] = parent

        matches.sort(key=lambda x: x[0])
        self.joint_names = [name for _, name in matches] + flg_nodes

        # print("Discovered nodes:", self.joint_names)
        # Print layer information for each joint
        for joint in self.joint_names:
            self.get_nodes_on_same_layer(joint_parents[joint], joint)
            # print(f"\nNodes on same layer as {joint}:")
            # print(f"Parent node: {joint_parents[joint]}")
            # print(f"Layer nodes: {same_layer_nodes}")

    def get_nodes_on_same_layer(self, parent_node, joint):
        """
        Find nodes on same layer and only add descendants of link nodes.
        """
        same_layer = []
        # First get immediate layer nodes
        for (parent, child), data in self.edge_data.items():
            if parent == parent_node:
                if child == joint:
                    continue
                if "geometry" in data:
                    same_layer.append(data["geometry"])
                    self.parent_nodes_dict[data["geometry"]] = child

                # Get all descendants for this link
                parentChild = child
                stack = [child]
                while stack:
                    current = stack.pop()
                    for (p, c), data in self.edge_data.items():
                        if p == current:
                            if "geometry" in data:
                                same_layer.append(data["geometry"])
                                self.parent_nodes_dict[data["geometry"]] = parentChild
                            stack.append(c)

        self.layer_nodes_dict[joint] = same_layer
        return same_layer

    def geometry_pose_to_matrix(self, init_pose: models.PlannerPose):
        return self.robot.pose_to_matrix(init_pose)

    def compute_forward_kinematics(self, joint_values):
        """Compute link transforms using the robot's methods."""
        accumulated = self.robot.pose_to_matrix(self.robot.mounting)
        transforms = [accumulated.copy()]
        for dh_param, joint_rot in zip(self.robot.dh_parameters, joint_values.joints, strict=False):
            transform = self.robot.dh_transform(dh_param, joint_rot)
            accumulated = accumulated @ transform
            transforms.append(accumulated.copy())
        return transforms

    def rotation_matrix_to_axis_angle(self, Rm):
        """Use scipy for cleaner axis-angle extraction."""
        rot = Rotation.from_matrix(Rm)
        angle = rot.magnitude()
        axis = rot.as_rotvec() / angle if angle > 1e-8 else np.array([1.0, 0.0, 0.0])
        return axis, angle

    def gamma_lift_single_color(self, color: np.ndarray, gamma: float = 0.8) -> np.ndarray:
        """
        Apply gamma correction to a single RGBA color in-place.
        color: shape (4,) with [R, G, B, A] in 0..255, dtype=uint8
        gamma: < 1.0 brightens midtones, > 1.0 darkens them.
        """
        rgb_float = color[:3].astype(np.float32) / 255.0
        rgb_float = np.power(rgb_float, gamma)
        color[:3] = (rgb_float * 255.0).astype(np.uint8)

        return color

    def get_transform_matrix(self):
        """
        Creates a transformation matrix that converts from glTF's right-handed Y-up
        coordinate system to Rerun's right-handed Z-up coordinate system.

        Returns:
            np.ndarray: A 4x4 transformation matrix
        """
        # Convert from glTF's Y-up to Rerun's Z-up coordinate system
        return np.array(
            [
                [1.0, 0.0, 0.0, 0.0],  # X stays the same
                [0.0, 0.0, -1.0, 0.0],  # Y becomes -Z
                [0.0, 1.0, 0.0, 0.0],  # Z becomes Y
                [0.0, 0.0, 0.0, 1.0],  # Homogeneous coordinate
            ]
        )

    def init_mesh(self, entity_path: str, geom, joint_name):
        """Generic method to log a single geometry, either capsule or box."""

        if entity_path not in self.logged_meshes:
            if geom.metadata.get("node") not in self.parent_nodes_dict:
                return

            base_transform = np.eye(4)
            # if the dh parameters are not at 0,0,0 from the mesh we have to move the first mesh joint
            if "J00" in joint_name:
                base_transform_, _ = self.scene.graph.get(frame_to=joint_name)
                base_transform = base_transform_.copy()
            base_transform[:3, 3] *= 1000

            # if the mesh has the pivot not in the center, we need to adjust the transform
            cumulative_transform, _ = self.scene.graph.get(
                frame_to=self.parent_nodes_dict[geom.metadata.get("node")]
            )
            ctransform = cumulative_transform.copy()

            # scale positions to mm
            ctransform[:3, 3] *= 1000

            # scale mesh to mm
            transform = base_transform @ ctransform
            mesh_scale_matrix = np.eye(4)
            mesh_scale_matrix[:3, :3] *= 1000
            transform = transform @ mesh_scale_matrix
            transformed_mesh = geom.copy()

            transformed_mesh.apply_transform(transform)

            if transformed_mesh.visual is not None:
                transformed_mesh.visual = transformed_mesh.visual.to_color()

            vertex_colors = None
            if transformed_mesh.visual and hasattr(transformed_mesh.visual, "vertex_colors"):
                vertex_colors = transformed_mesh.visual.vertex_colors

            rr.log(
                entity_path,
                rr.Mesh3D(
                    vertex_positions=transformed_mesh.vertices,
                    triangle_indices=transformed_mesh.faces,
                    vertex_normals=getattr(transformed_mesh, "vertex_normals", None),
                    albedo_factor=self.gamma_lift_single_color(vertex_colors, gamma=0.5)
                    if vertex_colors is not None
                    else None,
                ),
            )

            self.logged_meshes.add(entity_path)

    def init_collision_geometry(
        self, entity_path: str, collider: models.Collider, pose: models.PlannerPose
    ):
        if entity_path in self.logged_meshes:
            return

        if isinstance(collider.shape.actual_instance, models.Sphere2):
            rr.log(
                f"{entity_path}",
                rr.Ellipsoids3D(
                    radii=[
                        collider.shape.actual_instance.radius,
                        collider.shape.actual_instance.radius,
                        collider.shape.actual_instance.radius,
                    ],
                    centers=[[pose.position.x, pose.position.y, pose.position.z]]
                    if pose.position
                    else [0, 0, 0],
                    colors=[(221, 193, 193, 255)],
                ),
            )

        elif isinstance(collider.shape.actual_instance, models.Box2):
            rr.log(
                f"{entity_path}",
                rr.Boxes3D(
                    centers=[[pose.position.x, pose.position.y, pose.position.z]]
                    if pose.position
                    else [0, 0, 0],
                    sizes=[
                        collider.shape.actual_instance.size_x,
                        collider.shape.actual_instance.size_y,
                        collider.shape.actual_instance.size_z,
                    ],
                    colors=[(221, 193, 193, 255)],
                ),
            )

        elif isinstance(collider.shape.actual_instance, models.Capsule2):
            height = collider.shape.actual_instance.cylinder_height
            radius = collider.shape.actual_instance.radius

            # Generate trimesh capsule
            capsule = trimesh.creation.capsule(height=height, radius=radius, count=[6, 8])

            # Extract vertices and faces for solid visualization
            vertices = np.array(capsule.vertices)

            # Transform vertices to world position
            transform = np.eye(4)
            if pose.position:
                transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z - height / 2]
            else:
                transform[:3, 3] = [0, 0, -height / 2]

            if collider.pose and collider.pose.orientation:
                rot_mat = Rotation.from_quat(
                    [
                        collider.pose.orientation[0],
                        collider.pose.orientation[1],
                        collider.pose.orientation[2],
                        collider.pose.orientation[3],
                    ]
                )
                transform[:3, :3] = rot_mat.as_matrix()

            vertices = np.array([transform @ np.append(v, 1) for v in vertices])[:, :3]

            polygons = HullVisualizer.compute_hull_outlines_from_points(vertices)

            if polygons:
                line_segments = [p.tolist() for p in polygons]
                rr.log(
                    f"{entity_path}",
                    rr.LineStrips3D(
                        line_segments,
                        radii=rr.Radius.ui_points(0.75),
                        colors=[[221, 193, 193, 255]],
                    ),
                    static=True,
                )

        elif isinstance(collider.shape.actual_instance, models.ConvexHull2):
            polygons = HullVisualizer.compute_hull_outlines_from_points(
                collider.shape.actual_instance.vertices
            )

            if polygons:
                line_segments = [p.tolist() for p in polygons]
                rr.log(
                    f"{entity_path}",
                    rr.LineStrips3D(
                        line_segments, radii=rr.Radius.ui_points(1.5), colors=[colors.colors[2]]
                    ),
                    static=True,
                )

                vertices, triangles, normals = HullVisualizer.compute_hull_mesh(polygons)

                rr.log(
                    f"{entity_path}",
                    rr.Mesh3D(
                        vertex_positions=vertices,
                        triangle_indices=triangles,
                        vertex_normals=normals,
                        albedo_factor=colors.colors[0],
                    ),
                    static=True,
                )

        self.logged_meshes.add(entity_path)

    def init_geometry(self, entity_path: str, geometry: models.Geometry):
        """Generic method to log a single geometry, either capsule or box."""

        if entity_path in self.logged_meshes:
            return

        # Sphere geometry
        if geometry.sphere:
            radius = geometry.sphere.radius
            rr.log(
                entity_path,
                rr.Ellipsoids3D(
                    radii=[radius, radius, radius],
                    centers=[0, 0, 0],
                    fill_mode=rr.components.FillMode.Solid,
                    colors=[(221, 193, 193, 255) if self.static_transform else self.albedo_factor],
                ),
            )

        # Box geometry
        elif geometry.box:
            rr.log(
                entity_path,
                rr.Boxes3D(
                    centers=[0, 0, 0],
                    fill_mode=rr.components.FillMode.Solid,
                    sizes=[geometry.box.size_x, geometry.box.size_y, geometry.box.size_z],
                    colors=[(221, 193, 193, 255) if self.static_transform else self.albedo_factor],
                ),
            )

        # Rectangle geometry
        elif geometry.rectangle:
            # Create a flat box with minimal height
            rr.log(
                entity_path,
                rr.Boxes3D(
                    fill_mode=rr.components.FillMode.Solid,
                    centers=[0, 0, 0],
                    sizes=[
                        geometry.rectangle.size_x,
                        geometry.rectangle.size_y,
                        1.0,  # Minimal height for visibility
                    ],
                    colors=[(221, 193, 193, 255) if self.static_transform else self.albedo_factor],
                ),
            )

        # Cylinder geometry
        elif geometry.cylinder:
            radius = geometry.cylinder.radius
            height = geometry.cylinder.height

            # Create cylinder mesh
            cylinder = trimesh.creation.cylinder(radius=radius, height=height, sections=16)
            vertex_normals = cylinder.vertex_normals.tolist()

            rr.log(
                entity_path,
                rr.Mesh3D(
                    vertex_positions=cylinder.vertices.tolist(),
                    triangle_indices=cylinder.faces.tolist(),
                    vertex_normals=vertex_normals,
                    albedo_factor=self.albedo_factor,
                ),
            )

        # Convex hull geometry
        elif geometry.convex_hull:
            polygons = HullVisualizer.compute_hull_outlines_from_points(
                [[v.x, v.y, v.z] for v in geometry.convex_hull.vertices]
            )

            if polygons:
                # First log wireframe outline
                line_segments = [p.tolist() for p in polygons]
                rr.log(
                    f"{entity_path}_wireframe",
                    rr.LineStrips3D(
                        line_segments,
                        radii=rr.Radius.ui_points(1.0),
                        colors=[colors.colors[2] if self.static_transform else self.albedo_factor],
                    ),
                    static=self.static_transform,
                )

                # Then log solid mesh
                vertices, triangles, normals = HullVisualizer.compute_hull_mesh(polygons)

                rr.log(
                    entity_path,
                    rr.Mesh3D(
                        vertex_positions=vertices,
                        triangle_indices=triangles,
                        vertex_normals=normals,
                        albedo_factor=self.albedo_factor,
                    ),
                    static=self.static_transform,
                )

        # Capsule geometry
        elif geometry.capsule:
            radius = geometry.capsule.radius
            height = geometry.capsule.cylinder_height

            # Slightly shrink the capsule if static to reduce z-fighting
            if self.static_transform:
                radius *= 0.99
                height *= 0.99

            # Create capsule and retrieve normals
            cap_mesh = trimesh.creation.capsule(radius=radius, height=height)
            vertex_normals = cap_mesh.vertex_normals.tolist()

            rr.log(
                entity_path,
                rr.Mesh3D(
                    vertex_positions=cap_mesh.vertices.tolist(),
                    triangle_indices=cap_mesh.faces.tolist(),
                    vertex_normals=vertex_normals,
                    albedo_factor=self.albedo_factor,
                ),
            )

        # Rectangular capsule geometry
        elif geometry.rectangular_capsule:
            radius = geometry.rectangular_capsule.radius
            distance_x = geometry.rectangular_capsule.sphere_center_distance_x
            distance_y = geometry.rectangular_capsule.sphere_center_distance_y

            # Create a rectangular capsule from its definition - a hull around 4 spheres
            # First, create the four spheres at the corners
            sphere_centers = [
                [distance_x, distance_y, 0],
                [distance_x, -distance_y, 0],
                [-distance_x, distance_y, 0],
                [-distance_x, -distance_y, 0],
            ]

            # Generate points to create a convex hull
            all_points = []
            for center in sphere_centers:
                # Generate points for each sphere (simplified with key points on the sphere)
                for dx, dy, dz in [
                    (1, 0, 0),
                    (-1, 0, 0),
                    (0, 1, 0),
                    (0, -1, 0),
                    (0, 0, 1),
                    (0, 0, -1),
                ]:
                    all_points.append(
                        [center[0] + radius * dx, center[1] + radius * dy, center[2] + radius * dz]
                    )

            # Use our hull visualizer to create outlines
            polygons = HullVisualizer.compute_hull_outlines_from_points(all_points)

            if polygons:
                # Log wireframe outline
                line_segments = [p.tolist() for p in polygons]
                rr.log(
                    f"{entity_path}_wireframe",
                    rr.LineStrips3D(
                        line_segments,
                        radii=rr.Radius.ui_points(1.0),
                        colors=[
                            (221, 193, 193, 255) if self.static_transform else self.albedo_factor
                        ],
                    ),
                    static=self.static_transform,
                )

                # Log solid mesh
                vertices, triangles, normals = HullVisualizer.compute_hull_mesh(polygons)

                rr.log(
                    entity_path,
                    rr.Mesh3D(
                        vertex_positions=vertices,
                        triangle_indices=triangles,
                        vertex_normals=normals,
                        albedo_factor=self.albedo_factor,
                    ),
                    static=self.static_transform,
                )

        # Plane geometry (simplified as a large, thin rectangle)
        elif geometry.plane:
            # Create a large, thin rectangle to represent an infinite plane
            size = 5000  # Large enough to seem infinite in the visualization
            rr.log(
                entity_path,
                rr.Boxes3D(
                    centers=[0, 0, 0],
                    sizes=[size, size, 1.0],  # Very thin in z direction
                    colors=[(200, 200, 220, 100) if self.static_transform else self.albedo_factor],
                ),
            )

        # Compound geometry - recursively process child geometries
        elif geometry.compound and geometry.compound.child_geometries:
            for i, child_geom in enumerate(geometry.compound.child_geometries):
                child_path = f"{entity_path}/child_{i}"
                self.init_geometry(child_path, child_geom)

        # Default fallback for unsupported geometry types
        else:
            # Fallback to a box
            rr.log(
                entity_path,
                rr.Boxes3D(
                    half_sizes=[[50, 50, 50]],
                    colors=[(255, 0, 0, 128)],  # Red, semi-transparent to indicate unknown type
                ),
            )

        self.logged_meshes.add(entity_path)

    def log_robot_geometry(self, joint_position):
        transforms = self.compute_forward_kinematics(joint_position)

        def log_geometry(entity_path, transform):
            translation = transform[:3, 3]
            Rm = transform[:3, :3]
            axis, angle = self.rotation_matrix_to_axis_angle(Rm)
            rr.log(
                entity_path,
                rr.InstancePoses3D(
                    translations=[translation.tolist()],
                    rotation_axis_angles=[
                        rr.RotationAxisAngle(axis=axis.tolist(), angle=float(angle))
                    ],
                ),
                static=self.static_transform,
            )

        # Log robot joint geometries
        if self.mesh_loaded:
            for link_index, joint_name in enumerate(self.joint_names):
                link_transform = transforms[link_index]

                # Get nodes on same layer using dictionary
                same_layer_nodes = self.layer_nodes_dict.get(joint_name)
                if not same_layer_nodes:
                    continue

                filtered_geoms = []
                for node_name in same_layer_nodes:
                    if node_name in self.scene.geometry:
                        geom = self.scene.geometry[node_name]
                        # Add metadata that would normally come from dump
                        geom.metadata = {"node": node_name}
                        filtered_geoms.append(geom)

                for geom in filtered_geoms:
                    entity_path = f"{self.base_entity_path}/visual/links/link_{link_index}/mesh/{geom.metadata.get('node')}"

                    # calculate the inverse transform to get the mesh in the correct position
                    cumulative_transform, _ = self.scene.graph.get(frame_to=joint_name)
                    ctransform = cumulative_transform.copy()
                    inverse_transform = np.linalg.inv(ctransform)

                    # DH theta is rotated, rotate mesh around z in direction of theta
                    rotation_matrix_z_4x4 = np.eye(4)
                    if len(self.robot.dh_parameters) > link_index:
                        rotation_z_minus_90 = Rotation.from_euler(
                            "z", self.robot.dh_parameters[link_index].theta, degrees=False
                        ).as_matrix()
                        rotation_matrix_z_4x4[:3, :3] = rotation_z_minus_90

                    # scale positions to mm
                    inverse_transform[:3, 3] *= 1000

                    root_transform = self.get_transform_matrix()

                    transform = root_transform @ inverse_transform

                    final_transform = link_transform @ rotation_matrix_z_4x4 @ transform

                    self.init_mesh(entity_path, geom, joint_name)
                    log_geometry(entity_path, final_transform)

        # Log link geometries (safety from controller)
        if self.show_safety_link_chain:
            for link_index, geometries in self.link_geometries.items():
                link_transform = transforms[link_index]
                for i, geom in enumerate(geometries):
                    entity_path = f"{self.base_entity_path}/safety_from_controller/links/link_{link_index}/geometry_{i}"
                    final_transform = link_transform @ self.geometry_pose_to_matrix(geom.init_pose)

                    self.init_geometry(entity_path, geom)
                    log_geometry(entity_path, final_transform)

        # Log TCP geometries (safety from controller)
        if self.show_safety_link_chain and self.tcp_geometries:
            tcp_transform = transforms[-1]  # the final frame transform
            for i, geom in enumerate(self.tcp_geometries):
                entity_path = f"{self.base_entity_path}/safety_from_controller/tcp/geometry_{i}"
                final_transform = tcp_transform @ self.geometry_pose_to_matrix(geom.init_pose)

                self.init_geometry(entity_path, geom)
                log_geometry(entity_path, final_transform)

    def log_robot_geometries(self, trajectory: list[models.TrajectorySample], times_column):
        """
        Log the robot geometries for each link and TCP as separate entities.

        Args:
            trajectory (List[wb.models.TrajectorySample]): The list of trajectory sample points.
            times_column (rr.TimeColumn): The time column associated with the trajectory points.
        """
        link_positions = {}
        link_rotations = {}

        def collect_geometry_data(entity_path, transform):
            """Helper to collect geometry data for a given entity."""
            translation = transform[:3, 3].tolist()
            Rm = transform[:3, :3]
            axis, angle = self.rotation_matrix_to_axis_angle(Rm)
            if entity_path not in link_positions:
                link_positions[entity_path] = []
                link_rotations[entity_path] = []
            link_positions[entity_path].append(translation)
            link_rotations[entity_path].append(rr.RotationAxisAngle(axis=axis, angle=angle))

        for point in trajectory:
            transforms = self.compute_forward_kinematics(point.joint_position)

            # Log robot joint geometries
            if self.mesh_loaded:
                for link_index, joint_name in enumerate(self.joint_names):
                    if link_index >= len(transforms):
                        break
                    link_transform = transforms[link_index]

                    # Get nodes on same layer using dictionary
                    same_layer_nodes = self.layer_nodes_dict.get(joint_name)
                    if not same_layer_nodes:
                        continue

                    filtered_geoms = []
                    for node_name in same_layer_nodes:
                        if node_name in self.scene.geometry:
                            geom = self.scene.geometry[node_name]
                            # Add metadata that would normally come from dump
                            geom.metadata = {"node": node_name}
                            filtered_geoms.append(geom)

                    for geom in filtered_geoms:
                        entity_path = f"{self.base_entity_path}/visual/links/link_{link_index}/mesh/{geom.metadata.get('node')}"

                        # calculate the inverse transform to get the mesh in the correct position
                        cumulative_transform, _ = self.scene.graph.get(frame_to=joint_name)
                        ctransform = cumulative_transform.copy()
                        inverse_transform = np.linalg.inv(ctransform)

                        # DH theta is rotated, rotate mesh around z in direction of theta
                        rotation_matrix_z_4x4 = np.eye(4)
                        if len(self.robot.dh_parameters) > link_index:
                            rotation_z_minus_90 = Rotation.from_euler(
                                "z", self.robot.dh_parameters[link_index].theta, degrees=False
                            ).as_matrix()
                            rotation_matrix_z_4x4[:3, :3] = rotation_z_minus_90

                        # scale positions to mm
                        inverse_transform[:3, 3] *= 1000

                        root_transform = self.get_transform_matrix()

                        transform = root_transform @ inverse_transform

                        final_transform = link_transform @ rotation_matrix_z_4x4 @ transform

                        self.init_mesh(entity_path, geom, joint_name)
                        collect_geometry_data(entity_path, final_transform)

            # Collect data for link geometries (safety from controller)
            if self.show_safety_link_chain:
                for link_index, geometries in self.link_geometries.items():
                    link_transform = transforms[link_index]
                    for i, geom in enumerate(geometries):
                        entity_path = f"{self.base_entity_path}/safety_from_controller/links/link_{link_index}/geometry_{i}"
                        final_transform = link_transform @ self.geometry_pose_to_matrix(
                            geom.init_pose
                        )
                        self.init_geometry(entity_path, geom)
                        collect_geometry_data(entity_path, final_transform)

            # Collect data for TCP geometries (safety from controller)
            if self.show_safety_link_chain and self.tcp_geometries:
                tcp_transform = transforms[-1]  # End-effector transform
                for i, geom in enumerate(self.tcp_geometries):
                    entity_path = f"{self.base_entity_path}/safety_from_controller/tcp/geometry_{i}"
                    final_transform = tcp_transform @ self.geometry_pose_to_matrix(geom.init_pose)
                    self.init_geometry(entity_path, geom)
                    collect_geometry_data(entity_path, final_transform)

            # Collect data for collision link geometries (only if enabled)
            if self.show_collision_link_chain and self.collision_link_geometries:
                for link_index, geometries in enumerate(self.collision_link_geometries):
                    link_transform = transforms[link_index]
                    for i, geom_id in enumerate(geometries):
                        entity_path = f"{self.base_entity_path}/collision/links/link_{link_index}/geometry_{geom_id}"

                        pose = normalize_pose(geometries[geom_id].pose)

                        final_transform = link_transform @ self.geometry_pose_to_matrix(pose)
                        self.init_collision_geometry(entity_path, geometries[geom_id], pose)
                        collect_geometry_data(entity_path, final_transform)

            # Collect data for collision TCP geometries (only if enabled)
            if self.show_collision_link_chain and self.collision_tcp_geometries:
                tcp_transform = transforms[-1]  # End-effector transform
                for i, geom_id in enumerate(self.collision_tcp_geometries):
                    entity_path = f"{self.base_entity_path}/collision/tcp/geometry_{geom_id}"

                    pose = normalize_pose(self.collision_tcp_geometries[geom_id].pose)
                    final_transform = tcp_transform @ self.geometry_pose_to_matrix(pose)

                    # tcp collision geometries are defined in flange frame
                    identity_pose = models.PlannerPose(
                        position=models.Vector3d(x=0, y=0, z=0),
                        orientation=models.Quaternion(x=0, y=0, z=0, w=1),
                    )
                    self.init_collision_geometry(
                        entity_path, self.collision_tcp_geometries[geom_id], identity_pose
                    )
                    collect_geometry_data(entity_path, final_transform)

        # Send collected columns for all geometries
        for entity_path, positions in link_positions.items():
            rr.send_columns(
                entity_path,
                indexes=[times_column],
                columns=[
                    *rr.Transform3D.columns(
                        translation=positions, rotation_axis_angle=link_rotations[entity_path]
                    )
                ],
            )
