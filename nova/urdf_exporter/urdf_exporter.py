import os
import re
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Union

import numpy as np
import trimesh
from wandelbots_api_client.models.optimizer_setup import OptimizerSetup

from nova.urdf_exporter.geometry_converter import GeometryConverter
from nova.urdf_exporter.joint_definitions_handler import JointDefinitionsHandler
from nova.urdf_exporter.mesh_exporter import MeshExporter
from nova.urdf_exporter.srdf_exporter import SRDFExporter
from nova_rerun_bridge.dh_robot import DHRobot


class URDFExporter:
    """
    Main class for exporting robot models to URDF format.
    Coordinates all the components required for a complete URDF export.
    """

    def __init__(
        self,
        robot: DHRobot,
        model_name: str = "robot",
        export_path: str = None,
        mesh_model: str = None,
        robot_model_geometries: list = None,
        tcp_geometries: list = None,
        collision_link_chain: dict = None,
        collision_tcp: dict = None,
        optimizer_setup: OptimizerSetup = None,
    ):
        """
        Initialize the URDF exporter.

        Args:
            robot: DHRobot instance with DH parameters
            model_name: Name of the robot model (used as the robot name in URDF)
            export_path: Path to save the URDF file (defaults to current directory)
            mesh_model: Path to a 3D mesh model (.glb file) to include in the URDF
            robot_model_geometries: List of geometries for each link
            tcp_geometries: TCP geometries (similar structure to link geometries)
            collision_link_chain: Collision geometries for links
            collision_tcp: Collision geometries for TCP
            optimizer_setup: OptimizerSetup object containing robot configuration data
        """
        self.robot = robot
        self.model_name = model_name or "robot"
        self.export_path = export_path or os.path.join(os.getcwd(), "robot_models", self.model_name)
        self.mesh_model = mesh_model
        self.optimizer_setup = optimizer_setup

        # Initialize supporting components
        self.mesh_exporter = MeshExporter()
        self.joint_handler = JointDefinitionsHandler(robot, optimizer_setup)

        # Initialize geometries
        self.link_geometries = {}
        self._process_geometries(robot_model_geometries or [])

        self.tcp_geometries = tcp_geometries or []
        self.collision_link_geometries = collision_link_chain or {}
        self.collision_tcp_geometries = collision_tcp or {}

        # Check if optimizer setup has geometries to use
        self._process_optimizer_geometries()

        # Root of the URDF XML tree
        self.root = None

        # Track exported meshes
        self.logged_meshes = set()

    def _process_geometries(self, geometries):
        """Process and group geometries by link index."""
        for gm in geometries:
            link_idx = getattr(gm, "link_index", None)
            if link_idx is not None:
                self.link_geometries.setdefault(link_idx, []).append(gm)

    def _process_optimizer_geometries(self):
        """Extract geometries from optimizer setup if available."""
        if not self.optimizer_setup or not hasattr(self.optimizer_setup, "safety_setup"):
            return

        safety = self.optimizer_setup.safety_setup

        # Process robot geometries
        if hasattr(safety, "robot_model_geometries") and safety.robot_model_geometries:
            self._process_geometries(safety.robot_model_geometries)

        # Process TCP geometries
        if hasattr(safety, "tcp_geometries") and safety.tcp_geometries:
            self.tcp_geometries.extend(safety.tcp_geometries)

    def create_urdf(self, export_meshes=True):
        """
        Generate the URDF structure for the robot.

        Args:
            export_meshes: Whether to export mesh files for collision geometries

        Returns:
            Root XML element of the URDF
        """
        # Create the root element
        self.root = ET.Element("robot")
        self.root.set("name", self.model_name)

        # Add a materials section
        self._add_materials()

        # Add a dummy link like in the Jaka example
        dummy_link = ET.Element("link")
        dummy_link.set("name", "dummy")
        self.root.append(dummy_link)

        # Create the base link
        base_link = self._create_base_link()
        self.root.append(base_link)

        # Add a fixed joint from dummy to base link
        dummy_joint = ET.Element("joint")
        dummy_joint.set("name", "dummy_joint")
        dummy_joint.set("type", "fixed")
        parent = ET.SubElement(dummy_joint, "parent")
        parent.set("link", "dummy")
        child = ET.SubElement(dummy_joint, "child")
        child.set("link", f"{self.model_name}_base_link")
        self.root.append(dummy_joint)

        prev_link_name = f"{self.model_name}_base_link"

        # Create links and joints based on DH parameters
        for i, dh_param in enumerate(self.robot.dh_parameters):
            link_name = f"{self.model_name}_link_{i}"
            joint_name = f"{self.model_name}_joint_{i}"

            # Create the joint between previous link and this link
            # Note: Convert DH parameters from mm to m for URDF standard
            joint = self.joint_handler.create_joint_element(
                joint_name, prev_link_name, link_name, dh_param, i, convert_mm_to_m=True
            )
            self.root.append(joint)

            # Create the link
            link = self._create_link(i, export_meshes)
            self.root.append(link)

            prev_link_name = link_name

        # Add TCP/end effector if provided
        if self.tcp_geometries:
            self._add_tcp_link(prev_link_name, export_meshes)

        # Add mesh model if specified
        if self.mesh_model and export_meshes:
            self._add_mesh_model()

        return self.root

    def _add_materials(self):
        """Add material definitions to the URDF."""
        # Robot material
        material = ET.SubElement(self.root, "material")
        material.set("name", "robot_material")
        color = ET.SubElement(material, "color")
        color.set("rgba", "0.7 0.7 0.7 1.0")

        # Collision material
        collision_mat = ET.SubElement(self.root, "material")
        collision_mat.set("name", "collision_material")
        color = ET.SubElement(collision_mat, "color")
        color.set("rgba", "1.0 0.5 0.5 0.5")

    def _create_base_link(self):
        """Create the base link for the robot."""
        # Create a simple link with a basic shape
        base_link = GeometryConverter.create_simple_link(
            f"{self.model_name}_base_link", shape_type="box", dimensions=[0.1, 0.1, 0.1]
        )

        # If mounting pose is provided in the optimizer setup, add it
        if self.optimizer_setup and hasattr(self.optimizer_setup, "mounting"):
            visual = base_link.find("visual")
            if visual is not None:
                GeometryConverter.add_origin_from_pose(visual, self.optimizer_setup.mounting)

        return base_link

    def _create_link(self, link_index, export_meshes=True):
        """
        Create a link element with its geometries.

        Args:
            link_index: Index of the link
            export_meshes: Whether to export mesh files for collision geometries

        Returns:
            An XML Element representing the link
        """
        link_name = f"{self.model_name}_link_{link_index}"
        link = ET.Element("link")
        link.set("name", link_name)

        # Add inertial properties
        GeometryConverter.add_inertial_elem(link)

        # Add visual geometries for this link
        if link_index in self.link_geometries:
            for i, geom in enumerate(self.link_geometries[link_index]):
                if hasattr(geom, "geometry") and geom.geometry:
                    visual = ET.SubElement(link, "visual")
                    # Set origin if specified
                    if hasattr(geom.geometry, "init_pose") and geom.geometry.init_pose:
                        GeometryConverter.add_origin_from_pose(visual, geom.geometry.init_pose)

                    # Add the geometry based on its type
                    if hasattr(geom.geometry, "capsule") and geom.geometry.capsule:
                        # Add capsule geometry as a cylinder
                        geometry = ET.SubElement(visual, "geometry")
                        cylinder = ET.SubElement(geometry, "cylinder")
                        cylinder.set("radius", str(geom.geometry.capsule.radius))
                        cylinder.set("length", str(geom.geometry.capsule.cylinder_height))

                        # Add material
                        GeometryConverter.add_material(visual, "robot_material")

        # Add collision geometries for this link
        if export_meshes and link_index in self.collision_link_geometries:
            for geom_id, collider in self.collision_link_geometries[link_index].items():
                GeometryConverter.add_collision_geometry(link, collider, self.model_name)

                # If it's a convex hull and we need to export it
                if (
                    hasattr(collider, "shape")
                    and hasattr(collider.shape, "actual_instance")
                    and hasattr(collider.shape.actual_instance, "vertices")
                ):
                    shape = collider.shape.actual_instance
                    if hasattr(shape, "vertices") and shape.vertices:
                        # Export the convex hull mesh
                        hull_name = f"{self.model_name}_link_{link_index}_hull_{geom_id}"
                        self.mesh_exporter.export_convex_hull(
                            shape.vertices, hull_name, self.export_path
                        )

        return link

    def _add_tcp_link(self, parent_link_name, export_meshes=True):
        """
        Add a TCP (tool center point) link if TCP geometries are provided.

        Args:
            parent_link_name: Name of the parent link
            export_meshes: Whether to export mesh files for collision geometries
        """
        tcp_link_name = f"{self.model_name}_tcp"

        # Create a fixed joint to the TCP
        tcp_joint = self.joint_handler.create_fixed_joint(
            f"{self.model_name}_tcp_joint", parent_link_name, tcp_link_name
        )

        # If TCP pose is provided in optimizer setup, use it
        if self.optimizer_setup and hasattr(self.optimizer_setup, "tcp"):
            tcp_pose = self.optimizer_setup.tcp
            if tcp_pose:
                position, rpy = GeometryConverter.geometry_pose_to_xyz_rpy(tcp_pose)
                origin = ET.SubElement(tcp_joint, "origin")
                origin.set("xyz", f"{position[0]} {position[1]} {position[2]}")
                origin.set("rpy", f"{rpy[0]} {rpy[1]} {rpy[2]}")

        self.root.append(tcp_joint)

        # Create the TCP link
        tcp_link = ET.Element("link")
        tcp_link.set("name", tcp_link_name)

        # Add inertial properties
        GeometryConverter.add_inertial_elem(tcp_link)

        # Add TCP geometries
        for idx, geom in enumerate(self.tcp_geometries):
            if hasattr(geom, "capsule") and geom.capsule:
                GeometryConverter.add_capsule_visual(
                    tcp_link, geom.capsule, getattr(geom, "init_pose", None)
                )

        # Add TCP collision geometries
        if export_meshes and self.collision_tcp_geometries:
            for geom_id, collider in self.collision_tcp_geometries.items():
                GeometryConverter.add_collision_geometry(tcp_link, collider, self.model_name)

                # Export convex hulls if needed
                if (
                    hasattr(collider, "shape")
                    and hasattr(collider.shape, "actual_instance")
                    and hasattr(collider.shape.actual_instance, "vertices")
                ):
                    shape = collider.shape.actual_instance
                    if hasattr(shape, "vertices") and shape.vertices:
                        hull_name = f"{self.model_name}_tcp_hull_{geom_id}"
                        self.mesh_exporter.export_convex_hull(
                            shape.vertices, hull_name, self.export_path
                        )

        self.root.append(tcp_link)

    def _add_mesh_model(self):
        """Add the 3D mesh model to the URDF with individual link meshes."""
        if not self.mesh_model or not os.path.exists(self.mesh_model):
            return

        try:
            print(f"Loading mesh model: {self.mesh_model}")
            scene = trimesh.load_scene(self.mesh_model, file_type="glb")

            # Identify link meshes
            link_meshes = self._identify_link_meshes(scene)

            # Create mesh directory
            mesh_dir = self.mesh_exporter.ensure_mesh_directory(self.export_path)

            # Export and add meshes to links
            self._export_and_add_link_meshes(scene, link_meshes, mesh_dir)

        except Exception as e:
            print(f"Error processing mesh model: {e}")

    def _identify_link_meshes(self, scene) -> dict[str, list[str]]:
        """
        Identify individual link meshes from the scene.

        Returns:
            Dictionary mapping link names to lists of mesh node names
        """
        # Find all mesh geometries in the scene
        link_meshes = {}
        joint_pattern = re.compile(r"_J0(\d+)")
        flg_pattern = re.compile(r"_FLG")
        joint_parents = {}  # Store parent for each joint/FLG

        # First, identify all joints and their parent nodes
        for (parent, child), data in scene.graph.transforms.edge_data.items():
            # Check for joints (nodes like "_J00", "_J01", etc.)
            joint_match = joint_pattern.search(child)
            if joint_match:
                j_idx = int(joint_match.group(1))
                link_name = f"{self.model_name}_link_{j_idx}"
                joint_parents[child] = parent
                link_meshes[link_name] = []

            # Check for FLG (typically the flange/TCP)
            flg_match = flg_pattern.search(child)
            if flg_match:
                link_name = f"{self.model_name}_tcp"
                joint_parents[child] = parent
                link_meshes[link_name] = []

        # Now find geometries associated with each joint's layer
        for joint_name, parent_node in joint_parents.items():
            link_name = (
                f"{self.model_name}_link_{joint_pattern.search(joint_name).group(1)}"
                if joint_pattern.search(joint_name)
                else f"{self.model_name}_tcp"
            )

            # Get all nodes on the same layer
            same_layer_nodes = []

            # Find immediate children of the parent node
            for (p, c), data in scene.graph.transforms.edge_data.items():
                if p == parent_node:
                    if c == joint_name:
                        continue
                    if "geometry" in data:
                        same_layer_nodes.append(data["geometry"])

                    # Get all descendants for this link
                    stack = [c]
                    while stack:
                        current = stack.pop()
                        for (p2, c2), data2 in scene.graph.transforms.edge_data.items():
                            if p2 == current:
                                if "geometry" in data2:
                                    same_layer_nodes.append(data2["geometry"])
                                stack.append(c2)

            link_meshes[link_name] = same_layer_nodes

        # Add base link (index 0) if missing
        base_link_name = f"{self.model_name}_base_link"
        if base_link_name not in link_meshes:
            # Try to find base components (often named with "base")
            base_nodes = []
            for node_name in scene.geometry.keys():
                if "base" in node_name.lower():
                    base_nodes.append(node_name)
            if base_nodes:
                link_meshes[base_link_name] = base_nodes

        return link_meshes

    def _export_and_add_link_meshes(self, scene, link_meshes, mesh_dir):
        """
        Export individual meshes for each link and add them to the URDF.
        Merges geometries for each link where possible.

        Args:
            scene: The loaded 3D scene
            link_meshes: Dictionary mapping link names to lists of mesh node names
            mesh_dir: Directory for mesh exports
        """
        for link_name, mesh_nodes in link_meshes.items():
            # Skip if no meshes for this link
            if not mesh_nodes:
                continue

            # Find the link in the URDF
            link_elem = None
            for link in self.root.findall("link"):
                if link.get("name") == link_name:
                    link_elem = link
                    break

            if not link_elem:
                continue

            # Try to merge meshes for this link when possible
            try:
                # Create a combined mesh for this link
                combined_mesh = self._merge_link_meshes(scene, mesh_nodes)

                # Export the combined mesh as STL
                mesh_filename = f"{link_name}_combined.stl"
                mesh_path = os.path.join(mesh_dir, mesh_filename)
                combined_mesh.export(mesh_path, file_type="stl")

                # Add visual element for the combined mesh
                visual = ET.SubElement(link_elem, "visual")
                origin = ET.SubElement(visual, "origin")
                origin.set("xyz", "0 0 0")
                origin.set("rpy", "0 0 0")

                # Add geometry element
                geometry = ET.SubElement(visual, "geometry")
                mesh_elem = ET.SubElement(geometry, "mesh")

                # Use relative path for the mesh
                relative_path = f"../meshes/{os.path.basename(mesh_path)}"
                mesh_elem.set("filename", relative_path)
                mesh_elem.set("scale", "1 1 1")

                # Add material
                GeometryConverter.add_material(visual, "robot_material")

                # Add collision element
                collision = ET.SubElement(link_elem, "collision")
                c_origin = ET.SubElement(collision, "origin")
                c_origin.set("xyz", "0 0 0")
                c_origin.set("rpy", "0 0 0")
                c_geometry = ET.SubElement(collision, "geometry")
                c_mesh = ET.SubElement(c_geometry, "mesh")
                c_mesh.set("filename", relative_path)
                c_mesh.set("scale", "1 1 1")

            except Exception as e:
                print(
                    f"Failed to merge meshes for {link_name}, falling back to individual exports: {e}"
                )

                # Fallback: Process each mesh for this link individually
                for i, node_name in enumerate(mesh_nodes):
                    if node_name in scene.geometry:
                        # Skip if we've already processed this mesh
                        mesh_id = f"{link_name}_{node_name}"
                        if mesh_id in self.logged_meshes:
                            continue

                        self.logged_meshes.add(mesh_id)

                        # Get the mesh
                        mesh = scene.geometry[node_name]

                        # Export as STL
                        mesh_filename = f"{link_name}_part_{i}.stl"
                        mesh_path = os.path.join(mesh_dir, mesh_filename)

                        # Apply transformation to the mesh
                        transformed_mesh = mesh.copy()
                        if hasattr(scene.graph, "get"):
                            try:
                                matrix, _ = scene.graph.get(frame_to=node_name)

                                # First apply transformation to preserve positions
                                transform = matrix.copy()
                                transformed_mesh.apply_transform(transform)

                                # Then scale the mesh to mm (separate step to avoid scaling translation twice)
                                scale_matrix = np.eye(4)
                                scale_matrix[0, 0] = scale_matrix[1, 1] = scale_matrix[2, 2] = (
                                    1000.0
                                )
                                transformed_mesh.apply_transform(scale_matrix)

                            except Exception as e:
                                print(f"Error transforming mesh {node_name}: {e}")

                        # Export the mesh as STL
                        transformed_mesh.export(mesh_path, file_type="stl")

                        # Create visual element
                        visual = ET.SubElement(link_elem, "visual")
                        origin = ET.SubElement(visual, "origin")
                        origin.set("xyz", "0 0 0")
                        origin.set("rpy", "0 0 0")

                        # Add geometry element
                        geometry = ET.SubElement(visual, "geometry")
                        mesh_elem = ET.SubElement(geometry, "mesh")

                        # Use relative path for the mesh
                        relative_path = f"../meshes/{os.path.basename(mesh_path)}"
                        mesh_elem.set("filename", relative_path)
                        mesh_elem.set("scale", "1 1 1")  # Scale is already applied to mesh

                        # Add material
                        GeometryConverter.add_material(visual, "robot_material")

                        # Add collision element
                        collision = ET.SubElement(link_elem, "collision")
                        c_origin = ET.SubElement(collision, "origin")
                        c_origin.set("xyz", "0 0 0")
                        c_origin.set("rpy", "0 0 0")
                        c_geometry = ET.SubElement(collision, "geometry")
                        c_mesh = ET.SubElement(c_geometry, "mesh")
                        c_mesh.set("filename", relative_path)
                        c_mesh.set("scale", "1 1 1")  # Scale is already applied to mesh

    def _merge_link_meshes(self, scene, mesh_nodes):
        """
        Merge multiple mesh geometries into a single mesh.

        Args:
            scene: The loaded 3D scene
            mesh_nodes: List of mesh node names to merge

        Returns:
            A single trimesh object containing all merged geometries
        """
        # Collect all meshes
        meshes = []

        # Process each mesh
        for node_name in mesh_nodes:
            if node_name in scene.geometry:
                mesh = scene.geometry[node_name]
                transformed_mesh = mesh.copy()

                # Transform the mesh from local to world coordinates
                if hasattr(scene.graph, "get"):
                    try:
                        matrix, _ = scene.graph.get(frame_to=node_name)

                        # First apply transformation
                        transform = matrix.copy()
                        transformed_mesh.apply_transform(transform)

                        # Scale mesh based on size
                        bounds = transformed_mesh.bounding_box.bounds
                        bbox_size = bounds[1] - bounds[0]
                        max_size = np.max(bbox_size)

                        # If mesh dimensions are tiny (likely already in meters), scale up
                        # If mesh dimensions are very large (in mm), scale down
                        if max_size < 0.01:  # Too small in meters
                            scale_matrix = np.eye(4)
                            scale_matrix[0, 0] = scale_matrix[1, 1] = scale_matrix[2, 2] = 1000.0
                            transformed_mesh.apply_transform(scale_matrix)
                        elif max_size > 10:  # Too large in meters (likely in mm)
                            scale_matrix = np.eye(4)
                            scale_matrix[0, 0] = scale_matrix[1, 1] = scale_matrix[2, 2] = 0.001
                            transformed_mesh.apply_transform(scale_matrix)

                        meshes.append(transformed_mesh)
                    except Exception as e:
                        print(f"Error transforming mesh {node_name}: {e}")

        if not meshes:
            raise ValueError("No valid meshes to merge")

        # Use trimesh.util.concatenate to merge all meshes into one
        return trimesh.util.concatenate(meshes)

    def export_urdf(self, filename=None, export_meshes=True, create_package=False):
        """
        Export the URDF model to a file.

        Args:
            filename: Name of the URDF file (defaults to model_name.urdf)
            export_meshes: Whether to export mesh files for collision geometries
            create_package: Parameter kept for backward compatibility (no effect)

        Returns:
            Path to the exported URDF file
        """
        if self.root is None:
            self.create_urdf(export_meshes=export_meshes)

        # Use provided filename or generate a default one
        if filename is None:
            filename = f"{self.model_name}.urdf"

        # Make sure export directories exist
        # Ensure the URDF directory exists
        export_dir = os.path.join(self.export_path, "urdf")
        os.makedirs(export_dir, exist_ok=True)
        full_path = os.path.join(export_dir, filename)

        # Ensure the mesh directory exists (will be ../meshes relative to the URDF)
        mesh_dir = self.mesh_exporter.ensure_mesh_directory(self.export_path)

        # Pretty-print the XML with proper indentation
        rough_string = ET.tostring(self.root, "utf-8")
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")

        # Write to file
        with open(full_path, "w") as f:
            f.write(pretty_xml)

        print(f"URDF file exported to: {full_path}")
        print(f"Mesh files exported to: {mesh_dir}")

        return full_path

    def export_srdf(self, filename=None):
        """
        Export a Semantic Robot Description Format (SRDF) file.

        Args:
            filename: Name of the SRDF file (defaults to model_name.srdf)

        Returns:
            Path to the exported SRDF file
        """
        srdf_exporter = SRDFExporter(self.robot, self.model_name, self.optimizer_setup)
        return srdf_exporter.export(self.export_path, filename)


def create_urdf_exporter_from_optimizer_setup(
    optimizer_setup: OptimizerSetup,
    model_name: str = None,
    export_path: str = None,
    mesh_model: str = None,
) -> tuple[URDFExporter, str]:
    """
    Create a URDF exporter from an optimizer setup.

    Args:
        optimizer_setup: OptimizerSetup containing robot configuration
        model_name: Name for the exported robot model
        export_path: Path to export the URDF files
        mesh_model: Path to a mesh model file to include

    Returns:
        URDFExporter instance and default model name
    """
    # Check if we have DH parameters
    if not optimizer_setup.dh_parameters:
        raise ValueError("Optimizer setup does not contain DH parameters")

    # Create DHRobot instance from optimizer setup
    dh_robot = DHRobot(
        dh_parameters=optimizer_setup.dh_parameters, mounting=optimizer_setup.mounting
    )

    # Create URDFExporter
    exporter = URDFExporter(
        robot=dh_robot,
        model_name=model_name,
        export_path=export_path,
        mesh_model=mesh_model,
        optimizer_setup=optimizer_setup,
    )
    return exporter, model_name


def get_model_path_from_name(model_name: str) -> str:
    """
    Get the path to the 3D model file based on the model name.

    Args:
        model_name: Name of the robot model

    Returns:
        Path to the model file or None if not found
    """
    try:
        # Import here to avoid circular imports
        from nova_rerun_bridge.helper_scripts.download_models import get_project_root

        model_path = Path(get_project_root()) / "models" / f"{model_name}.glb"
        if model_path.exists():
            return str(model_path)
        print(f"Warning: Model file for {model_name} not found at {model_path}")
    except Exception as e:
        print(f"Could not locate model path: {e}")
    return None


def export_urdf_from_optimizer_setup(
    optimizer_setup: OptimizerSetup,
    model_name: str = None,
    export_path: str = None,
    mesh_model: str = None,
    create_ros_package: bool = False,  # Parameter kept for backward compatibility (no effect)
    export_srdf: bool = True,
) -> Union[str, tuple[str, str]]:
    """
    Create and export a URDF from an OptimizerSetup.

    Args:
        optimizer_setup: OptimizerSetup containing robot configuration
        model_name: Name for the exported robot model
        export_path: Path to export the URDF files
        mesh_model: Path to a mesh model file to include
        create_ros_package: Parameter kept for backward compatibility (no effect)
        export_srdf: Whether to also export an SRDF file

    Returns:
        Path to the exported URDF file and SRDF file (if requested) or just URDF path
    """
    # Create exporter from optimizer setup
    exporter, model_name = create_urdf_exporter_from_optimizer_setup(
        optimizer_setup, model_name, export_path, mesh_model
    )

    # If mesh_model was not provided but we have a model_name, try to locate the model
    if not exporter.mesh_model and model_name:
        exporter.mesh_model = get_model_path_from_name(model_name)
        if exporter.mesh_model:
            print(f"Using model: {exporter.mesh_model}")

    # Export URDF
    urdf_path = exporter.export_urdf()

    # Export SRDF if requested
    if export_srdf:
        srdf_path = exporter.export_srdf()
        return urdf_path, srdf_path

    return urdf_path
