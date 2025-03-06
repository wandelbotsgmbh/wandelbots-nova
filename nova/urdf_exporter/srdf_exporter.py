import os
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET

from wandelbots_api_client.models.optimizer_setup import OptimizerSetup

from nova_rerun_bridge.dh_robot import DHRobot


class SRDFExporter:
    """
    Generates Semantic Robot Description Format (SRDF) files to complement URDFs.
    SRDF provides additional semantic information about robots for motion planning.
    """

    def __init__(self, robot: DHRobot, model_name: str, optimizer_setup: OptimizerSetup = None):
        """
        Initialize the SRDF exporter.

        Args:
            robot: DHRobot instance with DH parameters
            model_name: Name of the robot model
            optimizer_setup: Optional optimizer setup containing safety zones and additional info
        """
        self.robot = robot
        self.model_name = model_name
        self.optimizer_setup = optimizer_setup
        self.root = None
        self.tcp_present = False

        # Check if a TCP is defined in the optimizer setup
        if optimizer_setup and hasattr(optimizer_setup, "tcp"):
            self.tcp_present = True

    def create_srdf(self):
        """
        Create the SRDF XML structure.

        Returns:
            The root XML element of the SRDF
        """
        # Create the root element
        self.root = ET.Element("robot")
        self.root.set("name", self.model_name)

        # Add robot groups
        self._add_robot_groups()

        # Add group states (predefined positions)
        self._add_group_states()

        # Add end effectors
        self._add_end_effectors()

        # Add disabled collisions
        self._add_disabled_collisions()

        return self.root

    def _add_robot_groups(self):
        """Add robot groups to the SRDF."""
        # Create a group for the main manipulator
        group = ET.SubElement(self.root, "group")
        group.set("name", "manipulator")

        # Add chain element to define the manipulator from base to end effector
        chain = ET.SubElement(group, "chain")
        chain.set("base_link", f"{self.model_name}_base_link")

        # Determine the end effector link - either TCP or last robot link
        if self.tcp_present:
            chain.set("tip_link", f"{self.model_name}_tcp")
        else:
            chain.set("tip_link", f"{self.model_name}_link_{len(self.robot.dh_parameters) - 1}")

        # If there are additional subgroups defined in optimizer setup, add them
        # This is a placeholder for future extensions
        if self.optimizer_setup and hasattr(self.optimizer_setup, "subgroups"):
            for subgroup in self.optimizer_setup.subgroups:
                pass  # Handle subgroups if needed

    def _add_group_states(self):
        """Add predefined robot group states (positions) to the SRDF."""
        # Add default "home" position (all zeros)
        self._add_group_state("home", "manipulator", [0.0] * len(self.robot.dh_parameters))

        # Add a "straight" position if we have joint limits
        if hasattr(self.robot, "joint_limits") or (
            self.optimizer_setup
            and hasattr(self.optimizer_setup.safety_setup, "global_limits")
            and hasattr(self.optimizer_setup.safety_setup.global_limits, "joint_position_limits")
        ):
            # Calculate middle position between joint limits
            middle_positions = []

            if hasattr(self.robot, "joint_limits"):
                for limit in self.robot.joint_limits:
                    middle = (limit.upper + limit.lower) / 2
                    middle_positions.append(middle)
            else:
                for limit in self.optimizer_setup.safety_setup.global_limits.joint_position_limits:
                    middle = (limit.upper_limit + limit.lower_limit) / 2
                    middle_positions.append(middle)

            self._add_group_state("straight", "manipulator", middle_positions)

    def _add_group_state(self, state_name, group_name, joint_positions):
        """
        Add a specific group state (predefined joint position).

        Args:
            state_name: Name for the state
            group_name: Name of the group
            joint_positions: List of joint position values
        """
        group_state = ET.SubElement(self.root, "group_state")
        group_state.set("name", state_name)
        group_state.set("group", group_name)

        # Add each joint position
        for i, pos in enumerate(joint_positions):
            joint = ET.SubElement(group_state, "joint")
            joint.set("name", f"{self.model_name}_joint_{i}")
            joint.set("value", str(pos))

    def _add_end_effectors(self):
        """Add end effector definitions to the SRDF."""
        # Only add end effector if TCP is present
        if self.tcp_present:
            end_effector = ET.SubElement(self.root, "end_effector")
            end_effector.set("name", "end_effector")
            end_effector.set("parent_link", f"{self.model_name}_tcp")
            end_effector.set("group", "manipulator")
        else:
            # Otherwise use the last link as end effector
            end_effector = ET.SubElement(self.root, "end_effector")
            end_effector.set("name", "end_effector")
            last_link = f"{self.model_name}_link_{len(self.robot.dh_parameters) - 1}"
            end_effector.set("parent_link", last_link)
            end_effector.set("group", "manipulator")

    def _add_disabled_collisions(self):
        """Add disabled collision pairs to the SRDF."""
        # Disable collisions between adjacent links
        for i in range(len(self.robot.dh_parameters)):
            if i > 0:  # Skip base link
                disable_collision = ET.SubElement(self.root, "disable_collisions")
                disable_collision.set("link1", f"{self.model_name}_link_{i - 1}")
                disable_collision.set("link2", f"{self.model_name}_link_{i}")
                disable_collision.set("reason", "Adjacent")

        # Disable collisions between base and last link if they're far apart
        # This can help with planning for some robots
        if len(self.robot.dh_parameters) > 2:
            disable_collision = ET.SubElement(self.root, "disable_collisions")
            disable_collision.set("link1", f"{self.model_name}_base_link")
            disable_collision.set(
                "link2", f"{self.model_name}_link_{len(self.robot.dh_parameters) - 1}"
            )
            disable_collision.set("reason", "Never")

        # If there is a TCP, disable collision with last link
        if self.tcp_present:
            disable_collision = ET.SubElement(self.root, "disable_collisions")
            disable_collision.set(
                "link1", f"{self.model_name}_link_{len(self.robot.dh_parameters) - 1}"
            )
            disable_collision.set("link2", f"{self.model_name}_tcp")
            disable_collision.set("reason", "Adjacent")

        # Add additional disabled collisions from optimizer setup if available
        # This is a placeholder for future extensions
        if self.optimizer_setup and hasattr(self.optimizer_setup, "disabled_collisions"):
            for disabled in self.optimizer_setup.disabled_collisions:
                pass  # Handle disabled collisions from optimizer setup

    def export(self, output_path, filename=None):
        """
        Export the SRDF to a file.

        Args:
            output_path: Directory to export to
            filename: Name of the SRDF file (defaults to model_name.srdf)

        Returns:
            Path to the exported SRDF file
        """
        if self.root is None:
            self.create_srdf()

        # Use provided filename or generate a default one
        if filename is None:
            filename = f"{self.model_name}.srdf"

        # Make sure the output directory exists
        os.makedirs(output_path, exist_ok=True)
        full_path = os.path.join(output_path, filename)

        # Pretty-print the XML with proper indentation
        rough_string = ET.tostring(self.root, "utf-8")
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")

        # Write to file
        with open(full_path, "w") as f:
            f.write(pretty_xml)

        print(f"SRDF file exported to: {full_path}")
        return full_path
