import pydantic
from abc import ABC
from typing import Literal, Union
from wandelbots.types.pose import Pose

# TODO: derive motions from corresponding API models
import wandelbots_api_client as ws


class MotionSettings(pydantic.BaseModel):
    """Settings to customize motions.

    Attributes:
        velocity: the cartesian velocity of the TCP on the segment in mm/s
        acceleration: the cartesian acceleration of the TCP on the segment in mm/s
        orientation_velocity: the (tcp) orientation velocity on the segment in rad/s
        orientation_acceleration: the tcp orientation acceleration on the segment in rad/s
        joint_velocities: the joint velocities on the segment in rad/s
        joint_accelerations: the joint accelerations on the segment in rad/s
        blending: the blending radius to connect the previous segment to this one. If blending == 0, blending is
            disabled. If blending == math.inf, blending is enabled and the radius is automatically determined
            based on the robot velocity.
        optimize_approach: define approach axis as DoF and optimize for executable path
    """

    velocity: float | None = pydantic.Field(None, ge=0)
    acceleration: float | None = pydantic.Field(None, ge=0)
    orientation_velocity: float | None = pydantic.Field(None, ge=0)
    orientation_acceleration: float | None = pydantic.Field(None, ge=0)
    joint_velocities: tuple[float, ...] | None = None
    joint_accelerations: tuple[float, ...] | None = None
    blending: float = pydantic.Field(0, ge=0)
    optimize_approach: bool = False

    @classmethod
    def field_to_varname(cls, field):
        return f"__ms_{field}"


MS = MotionSettings
PoseOrVectorTuple = Union[tuple[float, float, float, float, float, float], tuple[float, float, float]]


class Motion(pydantic.BaseModel, ABC):
    """Base model of a motion

    Args:
        type: the type of the motion
        settings: the settings of the motion

    """

    type: Literal["linear", "ptp", "circular", "joint_ptp", "spline"]
    target: Pose | tuple[float, ...]
    settings: MotionSettings = MotionSettings()

    @property
    def is_cartesian(self):
        return isinstance(self.target, Pose)


class Linear(Motion):
    """A linear motion

    Examples:
    >>> Linear(target=Pose.from_tuple((1, 2, 3, 4, 5, 6)), settings=MotionSettings(velocity=10))
    Linear(type='linear', target=Pose(position=Position(x=1.0, y=2.0, z=3.0), orientation=Orientation(x=4.0, y=5.0, z=6.0), coordinate_system=None), settings=MotionSettings(velocity=10.0, acceleration=None, orientation_velocity=None, orientation_acceleration=None, joint_velocities=None, joint_accelerations=None, blending=0, optimize_approach=False))

    """

    type: Literal["linear"] = "linear"
    target: Pose

    @pydantic.model_serializer
    def custom_serialize(self):
        return {
            "target_pose": {
                "position": list(self.target.position.to_tuple()),
                "orientation": list(self.target.orientation.to_tuple()),
            },
            "path_definition_name": "PathLine",
        }


def lin(target: PoseOrVectorTuple, settings: MotionSettings = MotionSettings()) -> Linear:
    """Convenience function to create a linear motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the linear motion

    Examples:
    >>> ms = MotionSettings(velocity=10)
    >>> assert lin((1, 2, 3, 4, 5, 6), settings=ms) == Linear(target=Pose.from_tuple((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert lin((1, 2, 3)) == lin((1, 2, 3, 0, 0, 0))

    """
    t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
    return Linear(target=Pose.from_tuple(t), settings=settings)


class PTP(Motion):
    """A point-to-point motion

    Examples:
    >>> PTP(target=Pose.from_tuple((1, 2, 3, 4, 5, 6)), settings=MotionSettings(velocity=30))
    PTP(type='ptp', target=Pose(position=Position(x=1.0, y=2.0, z=3.0), orientation=Orientation(x=4.0, y=5.0, z=6.0), coordinate_system=None), settings=MotionSettings(velocity=30.0, acceleration=None, orientation_velocity=None, orientation_acceleration=None, joint_velocities=None, joint_accelerations=None, blending=0, optimize_approach=False))

    """

    type: Literal["ptp"] = "ptp"

    @pydantic.model_serializer
    def custom_serialize(self):
        return {
            "target_pose": {
                "position": list(self.target.position.to_tuple()),
                "orientation": list(self.target.orientation.to_tuple()),
            },
            "path_definition_name": "PathCartesianPTP",
        }


def ptp(target: PoseOrVectorTuple, settings: MotionSettings = MotionSettings()) -> PTP:
    """Convenience function to create a point-to-point motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the point-to-point motion

    Examples:
    >>> ms = MotionSettings(acceleration=10)
    >>> assert ptp((1, 2, 3, 4, 5, 6), settings=ms) == PTP(target=Pose.from_tuple((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert ptp((1, 2, 3)) == ptp((1, 2, 3, 0, 0, 0))

    """
    t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
    return PTP(target=Pose.from_tuple(t), settings=settings)


class Circular(Motion):
    """A circular motion

    Args:
        intermediate: the intermediate pose

    """

    type: Literal["circular"] = "circular"
    intermediate: Pose

    @pydantic.model_serializer
    def custom_serialize(self):
        return {
            "target_pose": {
                "position": list(self.target.position.to_tuple()),
                "orientation": list(self.target.orientation.to_tuple()),
            },
            "path_definition_name": "PathCircle",
        }


def cir(
    target: PoseOrVectorTuple, intermediate: PoseOrVectorTuple, settings: MotionSettings = MotionSettings()
) -> Circular:
    """Convenience function to create a circular motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        intermediate: the intermediate pose or vector. If the intermediate is a vector, the orientation is set to
            (0, 0, 0).
        settings: the motion settings

    Returns: the circular motion

    Examples:
    >>> ms = MotionSettings(acceleration=10)
    >>> assert cir((1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), settings=ms) == Circular(target=Pose.from_tuple((1, 2, 3, 4, 5, 6)), intermediate=Pose.from_tuple((7, 8, 9, 10, 11, 12)), settings=ms)
    >>> assert cir((1, 2, 3), (4, 5, 6)) == cir((1, 2, 3, 0, 0, 0), (4, 5, 6, 0, 0, 0))

    """
    t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
    i = (*intermediate, 0.0, 0.0, 0.0) if len(intermediate) == 3 else intermediate
    return Circular(target=Pose.from_tuple(t), intermediate=Pose.from_tuple(i), settings=settings)


class JointPTP(Motion):
    """A joint PTP motion

    Examples:
    >>> JointPTP(target=(1, 2, 3, 4, 5, 6), settings=MotionSettings(velocity=30))
    JointPTP(type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(velocity=30.0, acceleration=None, orientation_velocity=None, orientation_acceleration=None, joint_velocities=None, joint_accelerations=None, blending=0, optimize_approach=False))

    """

    type: Literal["joint_ptp"] = "joint_ptp"

    @pydantic.model_serializer
    def custom_serialize(self):
        return {
            "target_pose": {
                "position": list(self.target.position.to_tuple()),
                "orientation": list(self.target.orientation.to_tuple()),
            },
            "path_definition_name": "PathJointPTP",
        }


def jnt(target: tuple[float, ...], settings: MotionSettings = MotionSettings()) -> JointPTP:
    """Convenience function to create a joint PTP motion

    Args:
        target: the target joint configuration
        settings: the motion settings

    Returns: the joint PTP motion

    Examples:
    >>> ms = MotionSettings(acceleration=10)
    >>> assert jnt((1, 2, 3, 4, 5, 6), settings=ms) == JointPTP(target=(1, 2, 3, 4, 5, 6), settings=ms)

    """
    return JointPTP(target=target, settings=settings)


class Spline(Motion):
    """A spline motion

    Args:
        path_parameter: the path parameter between 0 and 1
        time: the time in seconds

    """

    type: Literal["spline"] = "spline"
    path_parameter: float = pydantic.Field(1, ge=0)
    time: float | None = pydantic.Field(default=None, ge=0)

    @pydantic.model_serializer
    def custom_serialize(self):
        return {
            "target_pose": {
                "position": list(self.target.position.to_tuple()),
                "orientation": list(self.target.orientation.to_tuple()),
            },
            "path_definition_name": "PathCubicSpline",
        }


def spl(
    target: PoseOrVectorTuple, settings: MotionSettings = MotionSettings(), path_parameter: float = 1, time=None
) -> Spline:
    """Convenience function to create a spline motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings
        path_parameter: the path parameter between 0 and 1
        time: the time in seconds

    Returns: the spline motion

    Examples:
    >>> ms = MotionSettings(acceleration=10)
    >>> assert spl((1, 2, 3, 4, 5, 6), settings=ms) == Spline(target=Pose.from_tuple((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert spl((1, 2, 3)) == spl((1, 2, 3, 0, 0, 0))

    """
    t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
    return Spline(target=Pose.from_tuple(t), settings=settings, path_parameter=path_parameter, time=time)
