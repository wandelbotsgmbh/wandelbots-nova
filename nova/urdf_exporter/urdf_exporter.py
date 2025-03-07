import os
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET

from wandelbots_api_client.models.optimizer_setup import OptimizerSetup

from nova_rerun_bridge.dh_robot import DHRobot


class URDFExporter:
    """
    URDF exporter that converts DH parameters to URDF using the standard DH representation
    with separate active and passive joints to properly represent the DH transformations.

    Implementation matches the JS implementation with the precise DH parameter handling.
    """

    def __init__(
        self,
        robot: DHRobot,
        model_name: str = "example_robot",
        export_path: str = None,
        optimizer_setup: OptimizerSetup = None,
    ):
        """
        Initialize the URDF exporter.

        Args:
            robot: DHRobot instance with DH parameters
            model_name: Name of the robot model
            export_path: Path to save the URDF file
            optimizer_setup: OptimizerSetup with robot configuration
        """
        self.robot = robot
        self.model_name = model_name or "example_robot"
        self.export_path = export_path or os.path.join(os.getcwd(), "robot_models", self.model_name)
        self.optimizer_setup = optimizer_setup
        self.root = None
        self.modified_dh = False  # Default to standard DH parameters

    def create_urdf(self):
        """
        Generate URDF structure for the robot based on DH parameters.

        Returns:
            Root XML element of the URDF
        """
        # Create the root element
        self.root = ET.Element("robot")
        self.root.set("name", self.model_name)

        # Create initial setup - base links and joints
        self._create_initial_setup()

        # Process all DH rows
        for i, dh_param in enumerate(self.robot.dh_parameters):
            # Convert to row dictionary format like in JS implementation
            row_dict = {
                "row_no": i + 1,
                "name": i + 1,
                "previous_name": i,
                "th": dh_param.theta,
                "d": dh_param.d / 1000.0,  # Convert mm to m
                "a": dh_param.a / 1000.0,  # Convert mm to m
                "alpha": dh_param.alpha,
                "R": not getattr(
                    dh_param, "reverse_rotation_direction", False
                ),  # True for revolute, False for prismatic
                "type": "revolute",  # Default, will be updated if needed
            }

            # Update joint type based on R
            if not row_dict["R"]:
                row_dict["type"] = "prismatic"

            # Add all XML elements for this DH row
            self._add_dh_row_elements(row_dict)

        return self.root

    def _create_initial_setup(self):
        """Create the initial base links and joints."""
        # Add materials
        self._add_materials()

        # Create link0_passive (base link)
        base_link = ET.SubElement(self.root, "link")
        base_link.set("name", "link0_passive")
        visual = ET.SubElement(base_link, "visual")
        material = ET.SubElement(visual, "material")
        material.set("name", "blue")
        geometry = ET.SubElement(visual, "geometry")
        origin = ET.SubElement(geometry, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "0 0 0")
        cylinder = ET.SubElement(geometry, "cylinder")
        cylinder.set("length", "0.0")
        cylinder.set("radius", "0.0")

        # Create link0_x_axis (base X-axis visualization)
        x_axis_link = ET.SubElement(self.root, "link")
        x_axis_link.set("name", "link0_x_axis")
        visual = ET.SubElement(x_axis_link, "visual")
        material = ET.SubElement(visual, "material")
        material.set("name", "red")
        geometry = ET.SubElement(visual, "geometry")
        origin = ET.SubElement(geometry, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "0 0 0")
        cylinder = ET.SubElement(geometry, "cylinder")
        cylinder.set("length", "0.0")
        cylinder.set("radius", "0.0")

        # Create q0_x joint (base X-axis visualization joint)
        x_axis_joint = ET.SubElement(self.root, "joint")
        x_axis_joint.set("name", "q0_x")
        x_axis_joint.set("type", "fixed")
        origin = ET.SubElement(x_axis_joint, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "0 1.571 0")  # 90 degrees around Y
        parent = ET.SubElement(x_axis_joint, "parent")
        parent.set("link", "link0_passive")
        child = ET.SubElement(x_axis_joint, "child")
        child.set("link", "link0_x_axis")

        # Handle mounting if available
        if self.robot.mounting and hasattr(self.robot.mounting, "position"):
            # Create a world link and mounting joint
            world_link = ET.SubElement(self.root, "link")
            world_link.set("name", "world")

            mount_joint = ET.SubElement(self.root, "joint")
            mount_joint.set("name", "world_mount")
            mount_joint.set("type", "fixed")

            parent = ET.SubElement(mount_joint, "parent")
            parent.set("link", "world")

            child = ET.SubElement(mount_joint, "child")
            child.set("link", "link0_passive")

            origin = ET.SubElement(mount_joint, "origin")

            pos = self.robot.mounting.position
            x = pos.x / 1000.0 if pos.x else 0.0
            y = pos.y / 1000.0 if pos.y else 0.0
            z = pos.z / 1000.0 if pos.z else 0.0
            origin.set("xyz", f"{x} {y} {z}")

            # Handle rotation if available
            if hasattr(self.robot.mounting, "rotation"):
                rot = self.robot.mounting.rotation
                roll = rot.roll if hasattr(rot, "roll") else 0.0
                pitch = rot.pitch if hasattr(rot, "pitch") else 0.0
                yaw = rot.yaw if hasattr(rot, "yaw") else 0.0
                origin.set("rpy", f"{roll} {pitch} {yaw}")
            else:
                origin.set("rpy", "0 0 0")

    def _add_materials(self):
        """Add materials for visualization."""
        # Red material for X-axis
        red_material = ET.SubElement(self.root, "material")
        red_material.set("name", "red")
        red_color = ET.SubElement(red_material, "color")
        red_color.set("rgba", "1 0 0 1")

        # Blue material for links
        blue_material = ET.SubElement(self.root, "material")
        blue_material.set("name", "blue")
        blue_color = ET.SubElement(blue_material, "color")
        blue_color.set("rgba", "0 0 .8 1")

    def _add_dh_row_elements(self, row_dict):
        """
        Add all XML elements for a DH parameter row.

        Args:
            row_dict: Dictionary with DH parameters and metadata
        """
        # Step 1: Create main link (no visual)
        main_link = ET.SubElement(self.root, "link")
        main_link.set("name", f"link{row_dict['name']}")

        # Step 2: Create X-axis visualization link
        x_link = ET.SubElement(self.root, "link")
        x_link.set("name", f"link{row_dict['name']}_x_axis")
        visual = ET.SubElement(x_link, "visual")
        origin = ET.SubElement(visual, "origin")
        origin.set("xyz", "0 0 0.25")
        origin.set("rpy", "0 0 0")
        material = ET.SubElement(visual, "material")
        material.set("name", "red")
        geometry = ET.SubElement(visual, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        cylinder.set("length", "0.5")
        cylinder.set("radius", "0.05")

        # Step 3: Create active joint (implements d translation and theta rotation)
        active_joint = ET.SubElement(self.root, "joint")
        active_joint.set("name", f"q{row_dict['name']}")
        active_joint.set("type", row_dict["type"])

        # Origin - implement d translation and theta rotation
        origin = ET.SubElement(active_joint, "origin")
        origin.set("xyz", f"0 0 {row_dict['d']}")
        origin.set("rpy", f"0 0 {row_dict['th']}")

        # Connect previous passive link to current link
        parent = ET.SubElement(active_joint, "parent")
        parent.set("link", f"link{row_dict['previous_name']}_passive")
        child = ET.SubElement(active_joint, "child")
        child.set("link", f"link{row_dict['name']}")

        # Set joint axis
        axis = ET.SubElement(active_joint, "axis")
        axis.set("xyz", "0 0 1")

        # Add joint limits
        self._add_joint_limits(active_joint, row_dict["previous_name"])

        # Step 4: Create passive joint (implements a translation and alpha rotation)
        passive_joint = ET.SubElement(self.root, "joint")
        passive_joint.set("name", f"q{row_dict['row_no']}_passive")
        passive_joint.set("type", "fixed")

        # Origin - implement a translation and alpha rotation
        origin = ET.SubElement(passive_joint, "origin")
        origin.set("xyz", f"{row_dict['a']} 0 0")
        origin.set("rpy", f"{row_dict['alpha']} 0 0")

        # Connect current link to current passive link
        parent = ET.SubElement(passive_joint, "parent")
        parent.set("link", f"link{row_dict['name']}")
        child = ET.SubElement(passive_joint, "child")
        child.set("link", f"link{row_dict['name']}_passive")

        # Step 5: Create passive link (with visual)
        passive_link = ET.SubElement(self.root, "link")
        passive_link.set("name", f"link{row_dict['name']}_passive")
        visual = ET.SubElement(passive_link, "visual")
        origin = ET.SubElement(visual, "origin")
        origin.set("xyz", "0 0 0.25")
        origin.set("rpy", "0 0 0")
        material = ET.SubElement(visual, "material")
        material.set("name", "blue")
        geometry = ET.SubElement(visual, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        cylinder.set("length", "0.5")
        cylinder.set("radius", "0.05")

        # Step 6: Create X-axis visualization joint
        x_joint = ET.SubElement(self.root, "joint")
        x_joint.set("name", f"q{row_dict['row_no']}_x")
        x_joint.set("type", "fixed")
        origin = ET.SubElement(x_joint, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "0 1.571 0")  # 90 degrees around Y
        parent = ET.SubElement(x_joint, "parent")
        parent.set("link", f"link{row_dict['name']}_passive")
        child = ET.SubElement(x_joint, "child")
        child.set("link", f"link{row_dict['name']}_x_axis")

    def _add_joint_limits(self, joint_element, joint_index):
        """Add joint limits to the joint element."""
        limit = ET.SubElement(joint_element, "limit")

        # Try to get joint limits from optimizer setup
        if self.optimizer_setup and hasattr(self.optimizer_setup, "joint_constraints"):
            if joint_index < len(self.optimizer_setup.joint_constraints):
                constraint = self.optimizer_setup.joint_constraints[joint_index]
                limit.set("lower", str(constraint.lower_bound))
                limit.set("upper", str(constraint.upper_bound))
                limit.set("velocity", "1.0")  # Default value
                limit.set("effort", "100.0")  # Default value
                return

        # Default limits if nothing from optimizer setup
        limit.set("lower", "-3.14159")
        limit.set("upper", "3.14159")
        limit.set("velocity", "1.0")
        limit.set("effort", "100.0")

    def export_urdf(self, filename=None):
        """
        Export the URDF model to a file.

        Args:
            filename: Name of the URDF file

        Returns:
            Path to the exported URDF file
        """
        if self.root is None:
            self.create_urdf()

        # Use provided filename or default
        if filename is None:
            filename = f"{self.model_name}.urdf"

        # Create export directory
        export_dir = os.path.join(self.export_path, "urdf")
        os.makedirs(export_dir, exist_ok=True)
        full_path = os.path.join(export_dir, filename)

        # Write pretty XML to file
        rough_string = ET.tostring(self.root, "utf-8")
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")

        with open(full_path, "w") as f:
            f.write(pretty_xml)

        print(f"URDF file exported to: {full_path}")
        return full_path


def create_urdf_exporter_from_optimizer_setup(
    optimizer_setup: OptimizerSetup, model_name: str = None, export_path: str = None
) -> tuple[URDFExporter, str]:
    """
    Create a URDF exporter from an optimizer setup.

    Args:
        optimizer_setup: OptimizerSetup containing robot configuration
        model_name: Name for the exported robot model
        export_path: Path to export the URDF files

    Returns:
        URDFExporter instance and model name
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
        optimizer_setup=optimizer_setup,
    )
    return exporter, model_name


def export_urdf_from_optimizer_setup(
    optimizer_setup: OptimizerSetup, model_name: str = None, export_path: str = None
) -> str:
    """
    Create and export a URDF from an OptimizerSetup.

    Args:
        optimizer_setup: OptimizerSetup containing robot configuration
        model_name: Name for the exported robot model
        export_path: Path to export the URDF files

    Returns:
        Path to the exported URDF file
    """
    # Create exporter from optimizer setup
    exporter, model_name = create_urdf_exporter_from_optimizer_setup(
        optimizer_setup, model_name, export_path
    )

    # Export URDF
    urdf_path = exporter.export_urdf()
    return urdf_path
