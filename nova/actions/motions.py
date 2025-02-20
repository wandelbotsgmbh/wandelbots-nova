from abc import ABC, abstractmethod
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
    """
    A motion that is collision free.
    """

    type: Literal["collision_free_ptp"] = "collision_free_ptp"
    target: Pose | tuple[float, ...]
    settings: MotionSettings = MotionSettings()
    collision_scene: wb.models.CollisionScene | None = None

    def _to_api_model(self) -> api.models.PlanCollisionFreePTPRequestTarget:
        return wb.models.PlanCollisionFreePTPRequestTarget(
            self.target._to_wb_pose2() if isinstance(self.target, Pose) else list(self.target)
        )

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()

    def is_motion(self) -> bool:
        return True


def collision_free(
    target: Pose | tuple[float, ...],
    settings: MotionSettings = MotionSettings(),
    collision_scene: wb.models.CollisionScene | None = None,
) -> CollisionFreeMotion:
    return CollisionFreeMotion(target=target, settings=settings, collision_scene=collision_scene)


class Motion(Action, ABC):
    """Base model of a motion

    Args:
        type: the type of the motion
        settings: the settings of the motion

    """

    type: Literal["linear", "ptp", "circular", "joint_ptp", "spline"]
    target: Pose | tuple[float, ...]
    settings: MotionSettings = MotionSettings()
    collision_scene: wb.models.CollisionScene | None = None

    @property
    def is_cartesian(self):
        return isinstance(self.target, Pose)

    def is_motion(self) -> bool:
        return True


class UnresolvedMotion(Motion, ABC):
    @abstractmethod
    async def resolve(
        self,
        initial_joints: tuple[float, ...],
        collision_scene: wb.models.CollisionScene | None,
        configuration: dict,
        moving_robot_identifier: str,
    ) -> tuple[list[Motion], tuple[float, ...]] | None:
        """Convert the motion to a list of motion primitives

        Args:
            initial_joints: Joint positions at start of motion
            collision_scene: The collision scene used to check collisions
            configuration: E.g. data of physical setup of robot system, cell, etc.
            moving_robot_identifier: The identifier of the robot that is moving in the scene

        Returns:
            Tuple of resolved motions and the joint position at the end of the motions. None, if the motion can't be resolved

        """


class Linear(Motion):
    """A linear motion

    Examples:
    >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=10))
    Linear(type='linear', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=10.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """

    type: Literal["linear"] = "linear"
    target: Pose

    def _to_api_model(self) -> api.models.PathLine:
        """Serialize the model to the API model

        Examples:
        >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=10))._to_api_model()
        PathLine(target_pose=Pose2(position=[1, 2, 3], orientation=[4, 5, 6]), path_definition_name='PathLine')
        """
        return api.models.PathLine(
            target_pose=api.models.Pose2(**self.target.model_dump()),
            path_definition_name="PathLine",
        )

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()


def lin(
    target: PoseOrVectorTuple,
    settings: MotionSettings = MotionSettings(),
    collision_scene: wb.models.CollisionScene | None = None,
) -> Linear:
    """Convenience function to create a linear motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the linear motion

    Examples:
    >>> ms = MotionSettings(tcp_velocity_limit=10)
    >>> assert lin((1, 2, 3, 4, 5, 6), settings=ms) == Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert lin((1, 2, 3)) == lin((1, 2, 3, 0, 0, 0))
    >>> assert lin(Pose((1, 2, 3, 4, 5, 6)), settings=ms) == lin((1, 2, 3, 4, 5, 6), settings=ms)

    """
    if not isinstance(target, Pose):
        t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
        target = Pose(t)

    return Linear(target=target, settings=settings, collision_scene=collision_scene)


class PTP(Motion):
    """A point-to-point motion

    Examples:
    >>> PTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30))
    PTP(type='ptp', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """

    type: Literal["ptp"] = "ptp"

    def _to_api_model(self) -> api.models.PathCartesianPTP:
        """Serialize the model to the API model

        Examples:
        >>> PTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30))._to_api_model()
        PathCartesianPTP(target_pose=Pose2(position=[1, 2, 3], orientation=[4, 5, 6]), path_definition_name='PathCartesianPTP')
        """
        if not isinstance(self.target, Pose):
            raise ValueError("Target must be a Pose object")
        return api.models.PathCartesianPTP(
            target_pose=api.models.Pose2(**self.target.model_dump()),
            path_definition_name="PathCartesianPTP",
        )

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()


def ptp(
    target: PoseOrVectorTuple,
    settings: MotionSettings = MotionSettings(),
    collision_scene: wb.models.CollisionScene | None = None,
) -> PTP:
    """Convenience function to create a point-to-point motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the point-to-point motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert ptp((1, 2, 3, 4, 5, 6), settings=ms) == PTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert ptp((1, 2, 3)) == ptp((1, 2, 3, 0, 0, 0))
    >>> assert ptp(Pose((1, 2, 3, 4, 5, 6)), settings=ms) == ptp((1, 2, 3, 4, 5, 6), settings=ms)

    """
    if not isinstance(target, Pose):
        t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
        target = Pose(t)

    return PTP(target=target, settings=settings, collision_scene=collision_scene)


class Circular(Motion):
    """A circular motion

    Args:
        intermediate: the intermediate pose

    """

    type: Literal["circular"] = "circular"
    intermediate: Pose

    def _to_api_model(self) -> api.models.PathCircle:
        """Serialize the model to a dictionary

        Examples:
        >>> Circular(target=Pose((1, 2, 3, 4, 5, 6)), intermediate=Pose((10, 20, 30, 40, 50, 60)), settings=MotionSettings(tcp_velocity_limit=30))._to_api_model()
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

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()


def cir(
    target: PoseOrVectorTuple,
    intermediate: PoseOrVectorTuple,
    settings: MotionSettings = MotionSettings(),
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
    >>> assert cir((1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), settings=ms) == Circular(target=Pose((1, 2, 3, 4, 5, 6)), intermediate=Pose((7, 8, 9, 10, 11, 12)), settings=ms)
    >>> assert cir((1, 2, 3), (4, 5, 6)) == cir((1, 2, 3, 0, 0, 0), (4, 5, 6, 0, 0, 0))

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


class JointPTP(Motion):
    """A joint PTP motion

    Examples:
    >>> JointPTP(target=(1, 2, 3, 4, 5, 6), settings=MotionSettings(tcp_velocity_limit=30))
    JointPTP(type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(min_blending_velocity=None, position_zone_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None), collision_scene=None)

    """

    type: Literal["joint_ptp"] = "joint_ptp"

    def _to_api_model(self) -> api.models.PathJointPTP:
        """Serialize the model to the API model

        Examples:
        >>> JointPTP(target=(1, 2, 3, 4, 5, 6, 7), settings=MotionSettings(tcp_velocity_limit=30))._to_api_model()
        PathJointPTP(target_joint_position=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], path_definition_name='PathJointPTP')
        """
        if not isinstance(self.target, tuple):
            raise ValueError("Target must be a tuple object")
        return api.models.PathJointPTP(
            target_joint_position=list(self.target), path_definition_name="PathJointPTP"
        )

    @pydantic.model_serializer
    def serialize_model(self):
        return self._to_api_model().model_dump()


def jnt(
    target: tuple[float, ...],
    settings: MotionSettings = MotionSettings(),
    collision_scene: wb.models.CollisionScene | None = None,
) -> JointPTP:
    """Convenience function to create a joint PTP motion

    Args:
        target: the target joint configuration
        settings: the motion settings

    Returns: the joint PTP motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert jnt((1, 2, 3, 4, 5, 6), settings=ms) == JointPTP(target=(1, 2, 3, 4, 5, 6), settings=ms)

    """
    return JointPTP(target=target, settings=settings, collision_scene=collision_scene)


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
    def serialize_model(self):
        """Serialize the model to a dictionary

        Examples:
        >>> JointPTP(target=(1, 2, 3, 4, 5, 6, 7), settings=MotionSettings(tcp_velocity_limit=30)).model_dump()
        {'target_joint_position': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], 'path_definition_name': 'PathJointPTP'}
        """
        raise NotImplementedError("Spline motion is not yet implemented")


def spl(
    target: PoseOrVectorTuple,
    settings: MotionSettings = MotionSettings(),
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
    >>> assert spl((1, 2, 3, 4, 5, 6), settings=ms) == Spline(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms)
    >>> assert spl((1, 2, 3)) == spl((1, 2, 3, 0, 0, 0))

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
