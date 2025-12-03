from abc import ABC
from typing import Any, Literal, Sequence

import pydantic

from nova import api, utils
from nova.actions.base import Action
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose

PoseOrSequence = Pose | Sequence[float]


def _convert_to_pose(target: PoseOrSequence) -> Pose:
    if not isinstance(target, Pose):
        t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target

        if len(t) != 6:
            raise ValueError("Target must be a sequence of 6 floats")

        target = Pose(t)
    return target


class Motion(Action, ABC):
    """Base model of a motion

    Args:
        type: the type of the motion
        settings: the settings of the motion

    """

    type: Literal["linear", "cartesian_ptp", "circular", "joint_ptp", "spline", "collision_free"]
    target: Pose | tuple[float, ...]
    settings: MotionSettings = MotionSettings()
    collision_setup: api.models.CollisionSetup | None = None

    @property
    def is_cartesian(self):
        return isinstance(self.target, Pose)

    def is_motion(self) -> bool:
        return True


class Linear(Motion):
    """A linear motion

    Examples:
    >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=10))
    Linear(metas={}, type='linear', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=10.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """

    type: Literal["linear"] = "linear"
    target: Pose

    def to_api_model(self):
        """Serialize the model to the API model

        Examples:
        >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=10)).to_api_model()
        PathLine(target_pose=Pose(position=Vector3d(root=[1.0, 2.0, 3.0]), orientation=RotationVector(root=[4.0, 5.0, 6.0])), path_definition_name='PathLine')
        """
        return api.models.PathLine(
            target_pose=self.target.to_api_model(), path_definition_name="PathLine"
        )


def linear(
    target: PoseOrSequence,
    settings: MotionSettings = MotionSettings(),
    collision_setup: api.models.CollisionSetup | None = None,
    **kwargs: dict[str, Any],
) -> Linear:
    """Convenience function to create a linear motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the linear motion

    Examples:
    >>> ms = MotionSettings(tcp_velocity_limit=10)
    >>> assert linear((1, 2, 3, 4, 5, 6), settings=ms) == Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms, metas={'line_number': 1})
    >>> assert linear((1, 2, 3)) == linear((1, 2, 3, 0, 0, 0))
    >>> assert linear(Pose((1, 2, 3, 4, 5, 6)), settings=ms) == linear((1, 2, 3, 4, 5, 6), settings=ms)
    >>> Action.from_dict(linear((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    Linear(metas={'line_number': 1}, type='linear', target=Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0)), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """
    if not isinstance(target, Pose):
        t = (*target, 0.0, 0.0, 0.0) if len(target) == 3 else target
        target = Pose(t)

    kwargs.update(line_number=utils.get_caller_linenumber())

    return Linear(target=target, settings=settings, collision_setup=collision_setup, metas=kwargs)


lin = linear


class CartesianPTP(Motion):
    """A point-to-point motion

    Examples:
    >>> CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30))
    CartesianPTP(metas={}, type='cartesian_ptp', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """

    type: Literal["cartesian_ptp"] = "cartesian_ptp"

    def to_api_model(self) -> api.models.PathCartesianPTP:
        """Serialize the model to the API model

        Examples:
        >>> CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30)).to_api_model()
        PathCartesianPTP(target_pose=Pose(position=Vector3d(root=[1.0, 2.0, 3.0]), orientation=RotationVector(root=[4.0, 5.0, 6.0])), path_definition_name='PathCartesianPTP')
        """
        if not isinstance(self.target, Pose):
            raise ValueError("Target must be a Pose object")
        return api.models.PathCartesianPTP(
            target_pose=self.target.to_api_model(), path_definition_name="PathCartesianPTP"
        )


def cartesian_ptp(
    target: PoseOrSequence,
    settings: MotionSettings = MotionSettings(),
    collision_setup: api.models.CollisionSetup | None = None,
    **kwargs: dict[str, Any],
) -> CartesianPTP:
    """Convenience function to create a point-to-point motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        settings: the motion settings

    Returns: the point-to-point motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert cartesian_ptp((1, 2, 3, 4, 5, 6), settings=ms) == CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms, metas={'line_number': 1})
    >>> assert cartesian_ptp((1, 2, 3)) == cartesian_ptp((1, 2, 3, 0, 0, 0))
    >>> assert cartesian_ptp(Pose((1, 2, 3, 4, 5, 6)), settings=ms) == cartesian_ptp((1, 2, 3, 4, 5, 6), settings=ms)
    >>> Action.from_dict(cartesian_ptp((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    CartesianPTP(metas={'line_number': 1}, type='cartesian_ptp', target=Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0)), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """
    target = _convert_to_pose(target)
    kwargs.update(line_number=utils.get_caller_linenumber())
    return CartesianPTP(
        target=target, settings=settings, collision_setup=collision_setup, metas=kwargs
    )


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
        PathCircle(via_pose=Pose(position=Vector3d(root=[10.0, 20.0, 30.0]), orientation=RotationVector(root=[40.0, 50.0, 60.0])), target_pose=Pose(position=Vector3d(root=[1.0, 2.0, 3.0]), orientation=RotationVector(root=[4.0, 5.0, 6.0])), path_definition_name='PathCircle')
        """
        if not isinstance(self.target, Pose):
            raise ValueError("Target must be a Pose object")
        if not isinstance(self.intermediate, Pose):
            raise ValueError("Intermediate must be a Pose object")
        return api.models.PathCircle(
            target_pose=self.target.to_api_model(),
            via_pose=self.intermediate.to_api_model(),
            path_definition_name="PathCircle",
        )


def circular(
    target: PoseOrSequence,
    intermediate: PoseOrSequence,
    settings: MotionSettings = MotionSettings(),
    collision_setup: api.models.CollisionSetup | None = None,
    **kwargs: dict[str, Any],
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
    >>> assert circular((1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), settings=ms) == Circular(target=Pose((1, 2, 3, 4, 5, 6)), intermediate=Pose((7, 8, 9, 10, 11, 12)), settings=ms, metas={'line_number': 1})
    >>> assert circular((1, 2, 3), (4, 5, 6)) == circular((1, 2, 3, 0, 0, 0), (4, 5, 6, 0, 0, 0))
    >>> Action.from_dict(circular((1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), MotionSettings()).model_dump())
    Circular(metas={'line_number': 1}, type='circular', target=Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0)), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None, intermediate=Pose(position=Vector3d(x=7.0, y=8.0, z=9.0), orientation=Vector3d(x=10.0, y=11.0, z=12.0)))

    """
    target = _convert_to_pose(target)
    intermediate = _convert_to_pose(intermediate)
    kwargs.update(line_number=utils.get_caller_linenumber())
    return Circular(
        target=target,
        intermediate=intermediate,
        settings=settings,
        collision_setup=collision_setup,
        metas=kwargs,
    )


cir = circular


class JointPTP(Motion):
    """A joint PTP motion

    Examples:
    >>> JointPTP(target=(1, 2, 3, 4, 5, 6), settings=MotionSettings(tcp_velocity_limit=30))
    JointPTP(metas={}, type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """

    type: Literal["joint_ptp"] = "joint_ptp"

    def to_api_model(self) -> api.models.PathJointPTP:
        """Serialize the model to the API model

        Examples:
        >>> JointPTP(target=(1, 2, 3, 4, 5, 6, 7), settings=MotionSettings(tcp_velocity_limit=30)).to_api_model()
        PathJointPTP(target_joint_position=DoubleArray(root=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]), path_definition_name='PathJointPTP')
        """
        if not isinstance(self.target, tuple):
            raise ValueError("Target must be a tuple object")
        return api.models.PathJointPTP(
            target_joint_position=api.models.DoubleArray(list(self.target)),
            path_definition_name="PathJointPTP",
        )


def joint_ptp(
    target: tuple[float, ...],
    settings: MotionSettings = MotionSettings(),
    collision_setup: api.models.CollisionSetup | None = None,
    **kwargs: dict[str, Any],
) -> JointPTP:
    """Convenience function to create a joint PTP motion

    Args:
        target: the target joint configuration
        settings: the motion settings
        collision_setup: the collision setup. If collision_setup is provided, the motion will be a collision free motion.
            If collision_setup is not provided, the motion will be a normal joint PTP motion.

    Returns: the joint PTP motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert joint_ptp((1, 2, 3, 4, 5, 6), settings=ms) == JointPTP(target=(1, 2, 3, 4, 5, 6), settings=ms, metas={'line_number': 1})
    >>> Action.from_dict(joint_ptp((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    JointPTP(metas={'line_number': 1}, type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """

    kwargs.update(line_number=utils.get_caller_linenumber())
    return JointPTP(target=target, settings=settings, collision_setup=collision_setup, metas=kwargs)


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
    target: PoseOrSequence,
    settings: MotionSettings = MotionSettings(),
    path_parameter: float = 1,
    time=None,
    collision_setup: api.models.CollisionSetup | None = None,
    **kwargs: dict[str, Any],
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
    >>> assert spline((1, 2, 3, 4, 5, 6), settings=ms) == Spline(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms, metas={'line_number': 1})
    >>> assert spline((1, 2, 3)) == spline((1, 2, 3, 0, 0, 0))
    >>> Action.from_dict(spline((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    Spline(metas={'line_number': 1}, type='spline', target=Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0)), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None, path_parameter=1.0, time=None)

    """
    target = _convert_to_pose(target)
    kwargs.update(line_number=utils.get_caller_linenumber())
    return Spline(
        target=target,
        settings=settings,
        path_parameter=path_parameter,
        time=time,
        collision_setup=collision_setup,
        metas=kwargs,
    )


spl = spline


class CollisionFreeMotion(Motion):
    """A collision free motion

    Examples:
    >>> CollisionFreeMotion(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30))
    CollisionFreeMotion(metas={}, type='collision_free', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6)), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None, algorithm=CollisionFreeAlgorithm(root=RRTConnectAlgorithm(algorithm_name='RRTConnectAlgorithm', max_iterations=10000, max_step_size=1, adaptive_step_size=True, apply_smoothing=True, apply_blending=True)))
    """

    type: Literal["collision_free"] = "collision_free"
    target: Pose | tuple[float, ...]
    settings: MotionSettings = MotionSettings()
    collision_setup: api.models.CollisionSetup | None = None

    algorithm: api.models.CollisionFreeAlgorithm = api.models.CollisionFreeAlgorithm(
        api.models.RRTConnectAlgorithm()
    )

    def to_api_model(self) -> api.models.PlanCollisionFreeRequest:
        """"""
        # TODO: use data structure and API model are too different.
        # don't return the API model here
        raise NotImplementedError("CollisionFreeMotion.to_api_model is not implemented yet")


def collision_free(
    target: Pose | tuple[float, ...],
    settings: MotionSettings = MotionSettings(),
    collision_setup: api.models.CollisionSetup | None = None,
    algorithm: api.models.CollisionFreeAlgorithm = api.models.CollisionFreeAlgorithm(
        api.models.RRTConnectAlgorithm()
    ),
    **kwargs: dict[str, Any],
) -> CollisionFreeMotion:
    """Convenience function to create a collision free motion

    Args:
        target: the target joint configuration or pose
        settings: the motion settings
        collision_setup: the collision setup
        alogorithm: the collision free algorithm, default is RRTConnectAlgorithm

    Returns: the collision free motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert collision_free((1, 2, 3, 4, 5, 6), settings=ms) == CollisionFreeMotion(target=(1, 2, 3, 4, 5, 6), settings=ms, metas={'line_number': 1})
    >>> Action.from_dict(collision_free((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    CollisionFreeMotion(metas={'line_number': 1}, type='collision_free', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None, algorithm=CollisionFreeAlgorithm(root=RRTConnectAlgorithm(algorithm_name='RRTConnectAlgorithm', max_iterations=10000, max_step_size=1.0, adaptive_step_size=True, apply_smoothing=True, apply_blending=True)))
    """
    kwargs.update(line_number=utils.get_caller_linenumber())
    return CollisionFreeMotion(
        target=target,
        settings=settings,
        collision_setup=collision_setup,
        algorithm=algorithm,
        metas=kwargs,
    )
