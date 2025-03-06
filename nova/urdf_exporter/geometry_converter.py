import xml.etree.ElementTree as ET

from scipy.spatial.transform import Rotation

from nova.api import models


class GeometryConverter:
    """
    Converts different geometry types (spheres, boxes, capsules, convex hulls) to
    URDF-compatible representations.
    """

    @staticmethod
    def geometry_pose_to_xyz_rpy(pose: models.PlannerPose):
        """Convert a planner pose to xyz and rpy values for URDF."""
        position = [0.0, 0.0, 0.0]
        if pose and pose.position:
            position = [pose.position.x, pose.position.y, pose.position.z]

        rpy = [0, 0, 0]
        if pose and pose.orientation:
            # Convert quaternion to euler angles (roll, pitch, yaw)
            rot = Rotation.from_quat(
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            )
            rpy = rot.as_euler("xyz")

        return position, rpy

    @staticmethod
    def add_origin_from_pose(parent_elem, pose):
        """Add an origin XML element based on a pose."""
        if pose:
            position, rpy = GeometryConverter.geometry_pose_to_xyz_rpy(pose)
            origin = ET.SubElement(parent_elem, "origin")
            origin.set("xyz", f"{position[0]} {position[1]} {position[2]}")
            origin.set("rpy", f"{rpy[0]} {rpy[1]} {rpy[2]}")
        return parent_elem

    @staticmethod
    def add_inertial_elem(link_elem, mass=1.0):
        """Add default inertial properties to a link."""
        inertial = ET.SubElement(link_elem, "inertial")

        mass_elem = ET.SubElement(inertial, "mass")
        mass_elem.set("value", str(mass))

        inertia = ET.SubElement(inertial, "inertia")
        inertia.set("ixx", "0.1")
        inertia.set("ixy", "0")
        inertia.set("ixz", "0")
        inertia.set("iyy", "0.1")
        inertia.set("iyz", "0")
        inertia.set("izz", "0.1")

        # Center of mass at origin by default
        origin = ET.SubElement(inertial, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "0 0 0")

        return inertial

    @staticmethod
    def add_material(parent_elem, material_name="default_material", color=None):
        """Add material properties to a visual or collision element."""
        material = ET.SubElement(parent_elem, "material")
        material.set("name", material_name)

        if color:
            color_elem = ET.SubElement(material, "color")
            rgba_str = " ".join(str(c) for c in color)
            color_elem.set("rgba", rgba_str)

        return material

    @staticmethod
    def add_collision_geometry(parent, collider, mesh_prefix=None):
        """Add a collision geometry to a link element."""
        if not collider:
            return None

        collision = ET.SubElement(parent, "collision")

        # Set the origin if specified
        GeometryConverter.add_origin_from_pose(collision, collider.pose)

        # Add the geometry based on its type
        geometry = ET.SubElement(collision, "geometry")
        shape = collider.shape.actual_instance

        if isinstance(shape, models.Sphere2):
            sphere = ET.SubElement(geometry, "sphere")
            sphere.set("radius", str(shape.radius))

        elif isinstance(shape, models.Box2):
            box = ET.SubElement(geometry, "box")
            box.set("size", f"{shape.size_x} {shape.size_y} {shape.size_z}")

        elif isinstance(shape, models.Capsule2):
            # Approximate capsule with cylinder for URDF
            cylinder = ET.SubElement(geometry, "cylinder")
            cylinder.set("radius", str(shape.radius))
            cylinder.set("length", str(shape.cylinder_height))

        elif isinstance(shape, models.ConvexHull2) and mesh_prefix:
            # For convex hull, create a mesh reference
            mesh = ET.SubElement(geometry, "mesh")
            mesh_filename = f"{mesh_prefix}_convex_hull_{id(shape)}.stl"
            # Use relative path instead of file:// URI
            mesh.set("filename", f"../meshes/{mesh_filename}")
            mesh.set("scale", "1 1 1")

        # Add material with lower opacity for collision geometry
        GeometryConverter.add_material(collision, "collision_material", color=[1.0, 0.5, 0.5, 0.5])

        return collision

    @staticmethod
    def add_capsule_visual(parent, capsule, pose=None):
        """Add a capsule geometry to a link element as a visual element."""
        if not capsule:
            return None

        visual = ET.SubElement(parent, "visual")

        # Add pose if specified
        if pose:
            GeometryConverter.add_origin_from_pose(visual, pose)

        # Add cylinder geometry
        geometry = ET.SubElement(visual, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        cylinder.set("radius", str(capsule.radius))
        cylinder.set("length", str(capsule.cylinder_height))

        # Set material
        GeometryConverter.add_material(visual, "robot_material", color=[0.7, 0.7, 0.7, 1.0])

        return visual

    @staticmethod
    def add_mesh_visual(parent, mesh_filename, pose=None, scale=None):
        """Add a mesh visual element to a link."""
        visual = ET.SubElement(parent, "visual")

        # Add pose if specified
        if pose:
            GeometryConverter.add_origin_from_pose(visual, pose)

        # Add mesh geometry
        geometry = ET.SubElement(visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        mesh.set("filename", mesh_filename)
        if scale:
            scale_str = " ".join(str(s) for s in scale)
            mesh.set("scale", scale_str)

        # Set material
        GeometryConverter.add_material(visual, "robot_material")

        return visual

    @staticmethod
    def create_simple_link(name, shape_type="box", dimensions=None, mass=1.0):
        """
        Create a simple link with one visual element.

        Args:
            name: Link name
            shape_type: One of "box", "cylinder", "sphere"
            dimensions: Shape dimensions (size_x,y,z for box, radius,length for cylinder, radius for sphere)
            mass: Mass value for inertial properties
        """
        link = ET.Element("link")
        link.set("name", name)

        # Add inertial
        GeometryConverter.add_inertial_elem(link, mass)

        # Add visual
        visual = ET.SubElement(link, "visual")
        geometry = ET.SubElement(visual, "geometry")

        if shape_type == "box":
            dimensions = dimensions or [0.1, 0.1, 0.1]
            box = ET.SubElement(geometry, "box")
            box.set("size", f"{dimensions[0]} {dimensions[1]} {dimensions[2]}")

        elif shape_type == "cylinder":
            dimensions = dimensions or [0.05, 0.1]  # radius, length
            cylinder = ET.SubElement(geometry, "cylinder")
            cylinder.set("radius", str(dimensions[0]))
            cylinder.set("length", str(dimensions[1]))

        elif shape_type == "sphere":
            dimensions = dimensions or [0.05]  # radius
            sphere = ET.SubElement(geometry, "sphere")
            sphere.set("radius", str(dimensions[0]))

        # Add material
        GeometryConverter.add_material(visual, "robot_material")

        return link
