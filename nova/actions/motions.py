from abc import ABC
from typing import Literal

import pydantic
import wandelbots_api_client as wb

from nova import api
from nova.actions.base import Action
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose

PoseOrVectorTuple = (
    Pose | tuple[float, float, float, float, float, float] | tuple[float, float, float]
)


class CollisionFreeMotion(Action):
    """A motion that is collision free"""

    type: Literal["collision_free_ptp"] = "collision_free_ptp"
    target: Pose | tuple[float, ...]
    settings: MotionSettings | None = None
    collision_scene: wb.models.CollisionScene | None = None

    def to_api_model(self) -> api.models.PlanCollisionFreePTPRequestTarget:
        return wb.models.PlanCollisionFreePTPRequestTarget(
            self.target._to_wb_pose2() if isinstance(self.target, Pose) else list(self.target)
        )

    def is_motion(self) -> bool:
        return True


def collision_free(
    target: Pose | tuple[float, ...],
    settings: MotionSettings | None = None,
    collision_scene: wb.models.CollisionScene | None = None,
) -> CollisionFreeMotion:
    return CollisionFreeMotion(target=target, settings=settings, collision_scene=collision_scene)


class Motion(Action, ABC):
    """Base model of a motion

    Args:
        type: the type of the motion
        settings: the settings of the motion

    """

    type: Literal["linear", "cartesian_ptp", "circular", "joint_ptp", "spline"]
    target: Pose | tuple[float, ...]
    settings: MotionSettings | None = None
    collision_scene: wb.models.CollisionScene | None = None

    @property
    def is_cartesian(self):
        return isinstance(self.target, Pose)

    def is_motion(self) -> bool:
        return True


class Linear(Motion):
    """A linear motion

    Examples:
    >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=10))
    Linear(type='linear', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=10.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """

    type: Literal["linear"] = "linear"
    target: Pose

    def to_api_model(self):
        """Serialize the model to the API model

        Examples:
        >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=10)).to_api_model()
        PathLine(target_pose=Pose2(position=[1, 2, 3], orientation=[4, 5, 6]), path_definition_name='PathLine')
        """
        return api.models.PathLine(
            target_pose=api.models.Pose2(**self.target.model_dump()),
            path_definition_name="PathLine",
        )


def linear(
    target: PoseOrVectorTuple,
    settings: MotionSettings | None = None,
    collision_scene: wb.models.CollisionScene | None = None,
) -> Linear:
    """Convenience function to create a linear motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the linear motion

    Examples:
    >>> ms = MotionSettings(tcp_velocity_limit=10)
    >>> assert linear((1, 2, 3, 4, 5, 6), settings=ms) == Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert linear((1, 2, 3)) == linear((1, 2, 3, 0, 0, 0))
    >>> assert linear(Pose((1, 2, 3, 4, 5, 6)), settings=ms) == linear((1, 2, 3, 4, 5, 6), settings=ms)
    >>> Action.from_dict(linear((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    Linear(type='linear', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """
    if not isinstance(target, Pose):
        t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
        target = Pose(t)

    return Linear(target=target, settings=settings, collision_scene=collision_scene)


lin = linear


class CartesianPTP(Motion):
    """A point-to-point motion

    Examples:
    >>> CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30))
    CartesianPTP(type='cartesian_ptp', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """

    type: Literal["cartesian_ptp"] = "cartesian_ptp"

    def to_api_model(self) -> api.models.PathCartesianPTP:
        """Serialize the model to the API model

        Examples:
        >>> CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30)).to_api_model()
        PathCartesianPTP(target_pose=Pose2(position=[1, 2, 3], orientation=[4, 5, 6]), path_definition_name='PathCartesianPTP')
        """
        if not isinstance(self.target, Pose):
            raise ValueError("Target must be a Pose object")
        return api.models.PathCartesianPTP(
            target_pose=api.models.Pose2(**self.target.model_dump()),
            path_definition_name="PathCartesianPTP",
        )


def cartesian_ptp(
    target: PoseOrVectorTuple,
    settings: MotionSettings | None = None,
    collision_scene: wb.models.CollisionScene | None = None,
) -> CartesianPTP:
    """Convenience function to create a point-to-point motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the point-to-point motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert cartesian_ptp((1, 2, 3, 4, 5, 6), settings=ms) == CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert cartesian_ptp((1, 2, 3)) == cartesian_ptp((1, 2, 3, 0, 0, 0))
    >>> assert cartesian_ptp(Pose((1, 2, 3, 4, 5, 6)), settings=ms) == cartesian_ptp((1, 2, 3, 4, 5, 6), settings=ms)
    >>> Action.from_dict(cartesian_ptp((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    CartesianPTP(type='cartesian_ptp', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """
    if not isinstance(target, Pose):
        t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
        target = Pose(t)

    return CartesianPTP(target=target, settings=settings, collision_scene=collision_scene)


ptp = cartesian_ptp


class Circular(Motion):
    """A circular motion

    Args:
        intermediate: the intermediate pose

    """

    type: Literal["circular"] = "circular"
    intermediate: Pose

    def to_api_model(self) -> api.models.PathCircle:
        """Serialize the model to a dictionary

        Examples:
        >>> Circular(target=Pose((1, 2, 3, 4, 5, 6)), intermediate=Pose((10, 20, 30, 40, 50, 60)), settings=MotionSettings(tcp_velocity_limit=30)).to_api_model()
        PathCircle(via_pose=Pose2(position=[10, 20, 30], orientation=[40, 50, 60]), target_pose=Pose2(position=[1, 2, 3], orientation=[4, 5, 6]), path_definition_name='PathCircle')
        """
        if not isinstance(self.target, Pose):
            raise ValueError("Target must be a Pose object")
        if not isinstance(self.intermediate, Pose):
            raise ValueError("Intermediate must be a Pose object")
        return api.models.PathCircle(
            target_pose=api.models.Pose2(**self.target.model_dump()),
            via_pose=api.models.Pose2(**self.intermediate.model_dump()),
            path_definition_name="PathCircle",
        )


def circular(
    target: PoseOrVectorTuple,
    intermediate: PoseOrVectorTuple,
    settings: MotionSettings | None = None,
    collision_scene: wb.models.CollisionScene | None = None,
) -> Circular:
    """Convenience function to create a circular motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        intermediate: the intermediate pose or vector. If the intermediate is a vector, the orientation is set to
            (0, 0, 0).
        settings: the motion settings

    Returns: the circular motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert circular((1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), settings=ms) == Circular(target=Pose((1, 2, 3, 4, 5, 6)), intermediate=Pose((7, 8, 9, 10, 11, 12)), settings=ms)
    >>> assert circular((1, 2, 3), (4, 5, 6)) == circular((1, 2, 3, 0, 0, 0), (4, 5, 6, 0, 0, 0))
    >>> Action.from_dict(circular((1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), MotionSettings()).model_dump())
    Circular(type='circular', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None, intermediate=Pose(position=Vector3d(x=7, y=8, z=9), orientation=Vector3d(x=10, y=11, z=12)))

    """
    if not isinstance(target, Pose):
        t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
        target = Pose(t)

    if not isinstance(intermediate, Pose):
        i = (*intermediate, 0.0, 0.0, 0.0) if len(intermediate) == 3 else intermediate
        intermediate = Pose(i)

    return Circular(
        target=target, intermediate=intermediate, settings=settings, collision_scene=collision_scene
    )


cir = circular


class JointPTP(Motion):
    """A joint PTP motion

    Examples:
    >>> JointPTP(target=(1, 2, 3, 4, 5, 6), settings=MotionSettings(tcp_velocity_limit=30))
    JointPTP(type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """

    type: Literal["joint_ptp"] = "joint_ptp"

    def to_api_model(self) -> api.models.PathJointPTP:
        """Serialize the model to the API model

        Examples:
        >>> JointPTP(target=(1, 2, 3, 4, 5, 6, 7), settings=MotionSettings(tcp_velocity_limit=30)).to_api_model()
        PathJointPTP(target_joint_position=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], path_definition_name='PathJointPTP')
        """
        if not isinstance(self.target, tuple):
            raise ValueError("Target must be a tuple object")
        return api.models.PathJointPTP(
            target_joint_position=list(self.target), path_definition_name="PathJointPTP"
        )


def joint_ptp(
    target: tuple[float, ...],
    settings: MotionSettings | None = None,
    collision_scene: wb.models.CollisionScene | None = None,
) -> JointPTP:
    """Convenience function to create a joint PTP motion

    Args:
        target: the target joint configuration
        settings: the motion settings

    Returns: the joint PTP motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert joint_ptp((1, 2, 3, 4, 5, 6), settings=ms) == JointPTP(target=(1, 2, 3, 4, 5, 6), settings=ms)
    >>> Action.from_dict(joint_ptp((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    JointPTP(type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """
    return JointPTP(target=target, settings=settings, collision_scene=collision_scene)


jnt = joint_ptp


class Spline(Motion):
    """A spline motion

    Args:
        path_parameter: the path parameter between 0 and 1
        time: the time in seconds

    """

    type: Literal["spline"] = "spline"
    path_parameter: float = pydantic.Field(1, ge=0)
    time: float | None = pydantic.Field(default=None, ge=0)

    def to_api_model(self):
        raise NotImplementedError("Spline motion is not implemented yet")


def spline(
    target: PoseOrVectorTuple,
    settings: MotionSettings | None = None,
    path_parameter: float = 1,
    time=None,
    collision_scene: wb.models.CollisionScene | None = None,
) -> Spline:
    """Convenience function to create a spline motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings
        path_parameter: the path parameter between 0 and 1
        time: the time in seconds

    Returns: the spline motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert spline((1, 2, 3, 4, 5, 6), settings=ms) == Spline(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert spline((1, 2, 3)) == spline((1, 2, 3, 0, 0, 0))
    >>> Action.from_dict(spline((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    Spline(type='spline', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None, path_parameter=1.0, time=None)

    """
    if not isinstance(target, Pose):
        t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
        target = Pose(t)

    return Spline(
        target=target,
        settings=settings,
        path_parameter=path_parameter,
        time=time,
        collision_scene=collision_scene,
    )


spl = spline
