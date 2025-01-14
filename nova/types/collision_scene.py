from typing import Literal

import pydantic
import wandelbots_api_client as wb

from nova.types.pose import Pose


class DhParameter(pydantic.BaseModel):
    """Denavit-Hartenberg parameters for a single joint.
    Applied in d/theta --> a/alpha order."""

    a: float = 0.0  # [mm]
    alpha: float = 0.0  # [rad]
    d: float = 0.0  # [mm]
    theta: float = 0.0  # [rad]
    reverse_rotation_direction: bool = False


class Sphere(pydantic.BaseModel):
    shape_type: Literal["sphere"] = (
        "sphere"  # this allows shape type identification from Union of shapes
    )
    radius: float


class Box(pydantic.BaseModel):
    shape_type: Literal["box"] = "box"
    size_x: float
    size_y: float
    size_z: float
    type: Literal["HOLLOW", "FULL"] = "FULL"


class Rectangle(pydantic.BaseModel):
    shape_type: Literal["rectangle"] = "rectangle"
    size_x: float
    size_y: float


class Plane(pydantic.BaseModel):
    shape_type: Literal["plane"] = "plane"


class Cylinder(pydantic.BaseModel):
    shape_type: Literal["cylinder"] = "cylinder"
    radius: float
    height: float


class Capsule(pydantic.BaseModel):
    shape_type: Literal["capsule"] = "capsule"
    radius: float
    cylinder_height: float


class RectangularCapsule(pydantic.BaseModel):
    shape_type: Literal["rectangular_capsule"] = "rectangular_capsule"
    radius: float
    sphere_center_distance_x: float
    sphere_center_distance_y: float


class ConvexHull(pydantic.BaseModel):
    shape_type: Literal["convex_hull"] = "convex_hull"
    vertices: list[wb.models.Vector3d]


class Collider(pydantic.BaseModel):
    shape: Sphere | Box | Rectangle | Plane | Cylinder | Capsule | RectangularCapsule | ConvexHull
    pose: Pose
    margin: float = 0.0  # [mm] This will increase the size of the shape in all dimensions. Can be used to keep a safe distance to the shape.


class CollisionRobotConfiguration(pydantic.BaseModel):
    """Configuration of a robot in the collision scene.

    Default link shapes are provided for all supported robots. Set use_default_link_shapes to True to apply them.

    link_attachements are additional shapes that are attached to the link reference frames.
    The reference frame of the link_attachements is obtained after applying all sets of DH-parameters
    from base to (and including) the specified index.
    Adjacent links in the chain are not checked for collision.

    The tool is treated like the last link in the chain and all its shapes are attached to the flange frame.
    """

    # TODO: https://docs.jacobirobotics.com/motion/robot.html#item-obstacle
    #     These guys have an item obstcale attached at the tcp pose --> seems convenient for handling tools

    use_default_link_shapes: bool = True  # if True, default shapes are used for all links
    # shapes to attach to link reference frames, additionally to default shapes
    link_attachements: dict[int, dict[str, Collider]] = {}
    tool: dict[str, Collider] = {}  # shapes that make up the tool


class CollisionRobot(pydantic.BaseModel):
    """A robot in the collision scene is a single chain of articulated rigid bodies with defined shapes.
    The end of the chain is called flange and all tool shapes and tool center points (TCP) are attached to the flange.
    Adjacent links in the chain are not checked for collision.
    """

    mounting: Pose  # pose of the robot base w.r.t. the scene root
    dh_parameters: list[DhParameter] = []
    joint_positions: list[float] = []  # [rad], size should match dh_parameters
    links: dict[int, dict[str, Collider]] = {}  # shapes attached to link reference frames
    tool: dict[str, Collider] = {}  # shapes that make up the tool


class CollisionScene(pydantic.BaseModel):
    """A Collision scene is a collection of static colliders and robots with a shared reference frame.
    Assumptions:
    - everything is defined w.r.t. scene root
    - robot identifier matches a robot identifier --> the robot is placed in the scene via its mounting
    - tool colliders are defined in the flange frames and link_attachement colliders are defined in the respective link frames
    """

    static_colliders: dict[str, Collider] = {}
    robots: dict[str, CollisionRobot] = {}

    def add_static_collider(self, identifier: str, collider: Collider):
        self.static_colliders[identifier] = collider


# initial_args (json) -> store (objects)

"""
scene = read(arg...)
add_static_collider(scene, "box", Box(size_x=1, size_y=1, size_z=1), Pose.from_tuple([0, 0, 0, 0, 0, 0]))
"""

__all__ = [
    "Box",
    "Capsule",
    "Collider",
    "CollisionRobot",
    "CollisionRobotConfiguration",
    "CollisionScene",
    "ConvexHull",
    "Cylinder",
    "Plane",
    "Rectangle",
    "RectangularCapsule",
    "Sphere",
]
