import xml.etree.ElementTree as ET

from wandelbots_api_client.models.optimizer_setup import OptimizerSetup

from nova_rerun_bridge.dh_robot import DHRobot


class JointDefinitionsHandler:
    """
    Manages joint definitions for URDF, including joint types, limits,
    and transformations from DH parameters.
    """

    def __init__(self, robot: DHRobot, optimizer_setup: OptimizerSetup = None):
        """
        Initialize the joint definitions handler.

        Args:
            robot: DHRobot instance with DH parameters
            optimizer_setup: Optional optimizer setup containing joint limits and safety params
        """
        self.robot = robot
        self.optimizer_setup = optimizer_setup
        self.joint_types = self._determine_joint_types()
        self.joint_limits = self._extract_joint_limits()

    def _determine_joint_types(self):
        """
        Determine joint types based on DH parameters or additional info.
        Default to revolute joints, but can be customized as needed.

        Returns:
            List of joint types for each joint
        """
        # For now we default to revolute joints, but this could be extended
        # to detect prismatic joints or fixed joints based on DH parameters
        return ["revolute"] * len(self.robot.dh_parameters)

    def _extract_joint_limits(self):
        """
        Extract joint limits from optimizer setup or other sources.

        Returns:
            List of dictionaries containing joint limits
        """
        joint_limits = []

        # Try to extract from optimizer setup if available
        if self.optimizer_setup and hasattr(self.optimizer_setup, "safety_setup"):
            safety = self.optimizer_setup.safety_setup
            if hasattr(safety, "global_limits") and safety.global_limits:
                limits = safety.global_limits

                # Position limits
                if hasattr(limits, "joint_position_limits") and limits.joint_position_limits:
                    for limit in limits.joint_position_limits:
                        joint_limits.append(
                            {"lower": limit.lower_limit, "upper": limit.upper_limit}
                        )

                # Velocity limits
                if hasattr(limits, "joint_velocity_limits") and limits.joint_velocity_limits:
                    for i, vel in enumerate(limits.joint_velocity_limits):
                        if i < len(joint_limits):
                            joint_limits[i]["velocity"] = vel

                # Torque/effort limits
                if hasattr(limits, "joint_torque_limits") and limits.joint_torque_limits:
                    for i, effort in enumerate(limits.joint_torque_limits):
                        if i < len(joint_limits):
                            joint_limits[i]["effort"] = effort

        # If no joint limits from optimizer, check robot object
        if not joint_limits and hasattr(self.robot, "joint_limits"):
            for limit in self.robot.joint_limits:
                joint_limits.append(
                    {
                        "lower": limit.lower,
                        "upper": limit.upper,
                        "velocity": getattr(limit, "velocity", 1.0),
                        "effort": getattr(limit, "effort", 100.0),
                    }
                )

        # If still no limits, create default ones
        if not joint_limits:
            joint_limits = [
                {"lower": -3.14, "upper": 3.14, "velocity": 1.0, "effort": 100.0}
            ] * len(self.robot.dh_parameters)

        return joint_limits

    def create_joint_element(
        self,
        joint_name,
        parent_link,
        child_link,
        dh_param,
        joint_index,
        add_dynamics=False,
        convert_mm_to_m=False,
    ):
        """
        Create a joint XML element for URDF.

        Args:
            joint_name: Name for the joint
            parent_link: Name of the parent link
            child_link: Name of the child link
            dh_param: DH parameter for the joint
            joint_index: Index of the joint
            add_dynamics: Whether to add dynamics tags with damping and friction
            convert_mm_to_m: Whether to convert from mm to m for URDF standard

        Returns:
            An XML Element for the joint
        """
        joint = ET.Element("joint")
        joint.set("name", joint_name)

        # Use the pre-determined joint type
        joint_type = (
            self.joint_types[joint_index] if joint_index < len(self.joint_types) else "revolute"
        )
        joint.set("type", joint_type)

        parent = ET.SubElement(joint, "parent")
        parent.set("link", parent_link)

        child = ET.SubElement(joint, "child")
        child.set("link", child_link)

        # Set the joint origin based on DH parameters
        origin = ET.SubElement(joint, "origin")

        # DH convention: translate along x by a, along z by d, rotate around x by alpha, then around z by theta
        if convert_mm_to_m:
            # Convert mm to m for URDF standard
            a_value = dh_param.a / 1000.0
            d_value = dh_param.d / 1000.0
        else:
            a_value = dh_param.a
            d_value = dh_param.d

        origin.set("xyz", f"{a_value} 0 {d_value}")
        origin.set("rpy", f"{dh_param.alpha} 0 {dh_param.theta}")

        # Set the joint axis
        axis = ET.SubElement(joint, "axis")
        axis.set("xyz", "0 0 1")  # Z-axis rotation for a standard revolute joint

        # Add joint limits if available
        self._add_joint_limits(joint, joint_index)

        # Add dynamics tags if requested
        if add_dynamics:
            dynamics = ET.SubElement(joint, "dynamics")
            dynamics.set("damping", "0.0")
            dynamics.set("friction", "0.0")

        # Add information about reverse rotation direction if applicable
        if hasattr(dh_param, "reverse_rotation_direction") and dh_param.reverse_rotation_direction:
            mimic = ET.SubElement(joint, "mimic")
            mimic.set("joint", joint_name)  # Reference self for visualization
            mimic.set("multiplier", "-1")
            mimic.set("offset", "0")

        return joint

    def _add_joint_limits(self, joint_element, joint_index):
        """
        Add joint limits to a joint element.

        Args:
            joint_element: Joint XML element
            joint_index: Index of the joint

        Returns:
            The joint element with limits added
        """
        # Check if we have limits for this joint
        if joint_index < len(self.joint_limits) and self.joint_limits[joint_index] is not None:
            limits = self.joint_limits[joint_index]
            limit_elem = ET.SubElement(joint_element, "limit")

            limit_elem.set("lower", str(limits.get("lower", -3.14)))
            limit_elem.set("upper", str(limits.get("upper", 3.14)))
            limit_elem.set("velocity", str(limits.get("velocity", 1.0)))
            limit_elem.set("effort", str(limits.get("effort", 100.0)))
        else:
            # Add default limits
            limit_elem = ET.SubElement(joint_element, "limit")
            limit_elem.set("lower", "-3.14")
            limit_elem.set("upper", "3.14")
            limit_elem.set("velocity", "1.0")
            limit_elem.set("effort", "100.0")

        return joint_element

    def create_fixed_joint(self, joint_name, parent_link, child_link, xyz=None, rpy=None):
        """
        Create a fixed joint element between two links.

        Args:
            joint_name: Name for the joint
            parent_link: Name of the parent link
            child_link: Name of the child link
            xyz: Optional xyz offset as [x, y, z]
            rpy: Optional rpy rotation as [roll, pitch, yaw]

        Returns:
            An XML Element for the fixed joint
        """
        joint = ET.Element("joint")
        joint.set("name", joint_name)
        joint.set("type", "fixed")

        parent = ET.SubElement(joint, "parent")
        parent.set("link", parent_link)

        child = ET.SubElement(joint, "child")
        child.set("link", child_link)

        # Add origin if specified
        if xyz is not None or rpy is not None:
            xyz = xyz or [0, 0, 0]
            rpy = rpy or [0, 0, 0]

            origin = ET.SubElement(joint, "origin")
            origin.set("xyz", f"{xyz[0]} {xyz[1]} {xyz[2]}")
            origin.set("rpy", f"{rpy[0]} {rpy[1]} {rpy[2]}")

        return joint
