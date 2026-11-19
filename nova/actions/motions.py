import itertools
import math
from abc import ABC
from typing import Any, Literal, Sequence

import numpy as np
import pydantic

from nova import api, utils
from nova.actions.base import Action
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose

PoseLike = (
    api.models.ConfiguredPose
    | api.models.DatasetPose
    | api.models.Pose
    | Pose
    | Sequence[float]
    | np.ndarray
)


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
    Linear(metas={}, type='linear', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6), kinematic_configuration=None), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=10.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """

    type: Literal["linear"] = "linear"
    target: Pose

    def to_api_model(self):
        """Serialize the model to the API model

        Examples:
        >>> Linear(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=10)).to_api_model()
        PathLine(target_pose=Pose(position=(1.0, 2.0, 3.0), orientation=(4.0, 5.0, 6.0)), path_definition_name='PathLine')
        """
        return api.models.PathLine(
            target_pose=self.target.to_api_model(), path_definition_name="PathLine"
        )


def linear(
    target: PoseLike,
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
    Linear(metas={'line_number': 1}, type='linear', target=Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0), kinematic_configuration=None), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """
    target = target if isinstance(target, Pose) else Pose(target)

    kwargs.update(utils.get_caller_metas())

    return Linear(target=target, settings=settings, collision_setup=collision_setup, metas=kwargs)


lin = linear


class CartesianPTP(Motion):
    """A point-to-point motion

    Examples:
    >>> CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30))
    CartesianPTP(metas={}, type='cartesian_ptp', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6), kinematic_configuration=None), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """

    type: Literal["cartesian_ptp"] = "cartesian_ptp"

    def to_api_model(self) -> api.models.PathCartesianPTP:
        """Serialize the model to the API model

        If target.kinematic_configuration is set, it is passed to
        PathCartesianPTP.kinematic_configuration to specify the kinematic branch
        and axis ranges for reaching the target pose.

        Examples:
        >>> CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30)).to_api_model()
        PathCartesianPTP(target_pose=Pose(position=(1.0, 2.0, 3.0), orientation=(4.0, 5.0, 6.0)), kinematic_configuration=None, path_definition_name='PathCartesianPTP')
        >>> from nova import api
        >>> kc = api.models.KinematicConfiguration(kinematic_branch=api.models.KinematicBranch(shoulder_branch='FRONT', elbow_branch='UP', wrist_branch='NO_FLIP'))
        >>> CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kc), settings=MotionSettings()).to_api_model().kinematic_configuration == kc
        True
        """
        if not isinstance(self.target, Pose):
            raise ValueError("Target must be a Pose object")
        return api.models.PathCartesianPTP(
            target_pose=self.target.to_api_model(),
            kinematic_configuration=self.target.kinematic_configuration,
            path_definition_name="PathCartesianPTP",
        )


def cartesian_ptp(
    target: PoseLike,
    settings: MotionSettings = MotionSettings(),
    collision_setup: api.models.CollisionSetup | None = None,
    **kwargs: dict[str, Any],
) -> CartesianPTP:
    """Convenience function to create a point-to-point motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
            To specify a kinematic configuration, pass it via the Pose:
            ``Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=KinematicConfiguration(...))``
        settings: the motion settings

    Returns: the point-to-point motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert cartesian_ptp((1, 2, 3, 4, 5, 6), settings=ms) == CartesianPTP(target=Pose((1, 2, 3, 4, 5, 6)), settings=ms, metas={'line_number': 1})
    >>> assert cartesian_ptp((1, 2, 3)) == cartesian_ptp((1, 2, 3, 0, 0, 0))
    >>> assert cartesian_ptp(Pose((1, 2, 3, 4, 5, 6)), settings=ms) == cartesian_ptp((1, 2, 3, 4, 5, 6), settings=ms)
    >>> Action.from_dict(cartesian_ptp((1, 2, 3, 4, 5, 6), MotionSettings()).model_dump())
    CartesianPTP(metas={'line_number': 1}, type='cartesian_ptp', target=Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0), kinematic_configuration=None), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """
    target = target if isinstance(target, Pose) else Pose(target)
    kwargs.update(utils.get_caller_metas())
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
        PathCircle(via_pose=Pose(position=(10.0, 20.0, 30.0), orientation=(40.0, 50.0, 60.0)), target_pose=Pose(position=(1.0, 2.0, 3.0), orientation=(4.0, 5.0, 6.0)), path_definition_name='PathCircle')
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
    target: PoseLike,
    intermediate: PoseLike,
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
    Circular(metas={'line_number': 1}, type='circular', target=Pose(position=Vector3d(x=1.0, y=2.0, z=3.0), orientation=Vector3d(x=4.0, y=5.0, z=6.0), kinematic_configuration=None), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None, intermediate=Pose(position=Vector3d(x=7.0, y=8.0, z=9.0), orientation=Vector3d(x=10.0, y=11.0, z=12.0), kinematic_configuration=None))

    """
    target = target if isinstance(target, Pose) else Pose(target)
    intermediate = intermediate if isinstance(intermediate, Pose) else Pose(intermediate)
    kwargs.update(utils.get_caller_metas())
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
    JointPTP(metas={}, type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

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
    JointPTP(metas={'line_number': 1}, type='joint_ptp', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None)

    """

    kwargs.update(utils.get_caller_metas())
    return JointPTP(target=target, settings=settings, collision_setup=collision_setup, metas=kwargs)


jnt = joint_ptp


class Spline(Motion):
    """A cartesian cubic spline through a list of poses

    The spline starts at ``start``, passes through every pose of ``via`` and ends at ``target``.
    The planner interpolates position and orientation (quaternion coefficients, normalized) with
    a cubic spline through all poses and plans the whole spline as one motion command. With
    uneven pose spacing the curve can overshoot between poses.

    ``start`` is the pose the robot is
    at when the spline begins, i.e. the target of the motion before it; the planner rejects the
    spline otherwise (``FeedbackCubicSplineNotAtStartPose``).

    Args:
        start: the pose the spline starts at
        via: the poses the spline passes through between ``start`` and ``target``
        parameters: the path parameter of every pose (``start``, ``via``, ``target``), finite and
            strictly rising. If ``None``, the chord length along the poses in mm is used;
            identical consecutive poses get a minimal step, and consecutive poses that share a
            position but differ in orientation (a reorientation in place) raise a ``ValueError``,
            so pass explicit parameters for them.

    All pose components must be finite.

    """

    type: Literal["spline"] = "spline"
    target: Pose
    start: Pose
    via: tuple[Pose, ...] = ()
    parameters: tuple[float, ...] | None = None

    @pydantic.model_validator(mode="after")
    def _validate_parameters(self):
        poses = (self.start, *self.via, self.target)
        if not all(math.isfinite(value) for pose in poses for value in pose.to_tuple()):
            raise ValueError("the poses of a spline must be finite")
        if self.parameters is None:
            for (a, b), step in zip(itertools.pairwise(poses), _chord_lengths(poses)):
                if step < _MIN_PARAMETER_STEP and a.orientation != b.orientation:
                    raise ValueError(
                        "consecutive spline poses share a position but differ in orientation; "
                        "pass explicit parameters"
                    )
            return self
        if not all(math.isfinite(parameter) for parameter in self.parameters):
            raise ValueError("the path parameters of a spline must be finite")
        if len(self.parameters) != len(self.via) + 2:
            raise ValueError("a spline needs one path parameter per pose")
        if any(b <= a for a, b in itertools.pairwise(self.parameters)):
            raise ValueError("the path parameters of a spline must be strictly rising")
        return self

    def to_api_model(self) -> api.models.PathCubicSpline:
        """Serialize the model to the API model

        Without ``parameters``, the path parameter is the chord length along the poses in mm.
        Identical consecutive poses are given a minimal step, so the parameter still rises;
        consecutive poses that share a position but differ in orientation (any difference in
        the orientation vector) are rejected at construction unless ``parameters`` is given.

        Examples:
        >>> Spline(start=Pose((0, 0, 0)), via=(Pose((3, 4, 0)),), target=Pose((3, 4, 12))).to_api_model()
        PathCubicSpline(parameters=[CubicSplineParameter(pose=Pose(position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0)), path_parameter=0.0), CubicSplineParameter(pose=Pose(position=(3.0, 4.0, 0.0), orientation=(0.0, 0.0, 0.0)), path_parameter=5.0), CubicSplineParameter(pose=Pose(position=(3.0, 4.0, 12.0), orientation=(0.0, 0.0, 0.0)), path_parameter=17.0)], path_definition_name='PathCubicSpline')
        """
        poses = (self.start, *self.via, self.target)
        parameters = self.parameters or _chord_length_parameters(poses)
        return api.models.PathCubicSpline(
            parameters=[
                api.models.CubicSplineParameter(
                    pose=pose.to_api_model(), path_parameter=float(parameter)
                )
                for pose, parameter in zip(poses, parameters)
            ],
            path_definition_name="PathCubicSpline",
        )


#: Path parameter step between identical poses, so the default parameter still rises.
_MIN_PARAMETER_STEP = 1e-3


def _chord_lengths(poses: Sequence[Pose]) -> np.ndarray:
    positions = np.array([[p.position.x, p.position.y, p.position.z] for p in poses])
    return np.linalg.norm(np.diff(positions, axis=0), axis=1)


def _chord_length_parameters(poses: Sequence[Pose]) -> list[float]:
    steps = np.maximum(_chord_lengths(poses), _MIN_PARAMETER_STEP)
    return [0.0, *np.cumsum(steps).tolist()]


def spline(
    target: PoseLike,
    via: Sequence[PoseLike] = (),
    *,
    start: PoseLike,
    settings: MotionSettings = MotionSettings(),
    parameters: Sequence[float] | None = None,
    collision_setup: api.models.CollisionSetup | None = None,
    **kwargs: dict[str, Any],
) -> Spline:
    """Convenience function to create a spline motion

    Args:
        target: the target pose or vector. If the target is a vector, the orientation is set to (0, 0, 0).
        via: the poses or vectors the spline passes through between start and target
        start: the pose or vector the robot is at when the spline begins, i.e. the target of the
            motion before it
        settings: the motion settings
        parameters: the path parameter of every pose (start, via, target), finite and strictly
            rising. If ``None``, the chord length along the poses in mm is used; identical
            consecutive poses get a minimal step, and a reorientation in place (consecutive poses
            with the same position but a different orientation) raises a ``ValueError``, so pass
            explicit parameters for it.

    Returns: the spline motion

    Examples:
    >>> ms = MotionSettings(tcp_acceleration_limit=10)
    >>> assert spline((1, 2, 3, 4, 5, 6), start=(0, 0, 0), settings=ms) == Spline(target=Pose((1, 2, 3, 4, 5, 6)), start=Pose((0, 0, 0)), settings=ms, metas={'line_number': 1})
    >>> assert spline((1, 2, 3), start=(0, 0, 0)) == spline((1, 2, 3, 0, 0, 0), start=(0, 0, 0, 0, 0, 0))
    >>> Action.from_dict(spline((1, 2, 3), via=[(1, 1, 1)], start=(0, 0, 0)).model_dump()).via
    (Pose(position=Vector3d(x=1.0, y=1.0, z=1.0), orientation=Vector3d(x=0.0, y=0.0, z=0.0), kinematic_configuration=None),)

    """

    def as_pose(value: PoseLike) -> Pose:
        return value if isinstance(value, Pose) else Pose(value)

    kwargs.update(utils.get_caller_metas())
    return Spline(
        target=as_pose(target),
        start=as_pose(start),
        via=tuple(as_pose(pose) for pose in via),
        settings=settings,
        parameters=None if parameters is None else tuple(float(x) for x in parameters),
        collision_setup=collision_setup,
        metas=kwargs,
    )


spl = spline


class CollisionFreeMotion(Motion):
    """A collision free motion

    Examples:
    >>> CollisionFreeMotion(target=Pose((1, 2, 3, 4, 5, 6)), settings=MotionSettings(tcp_velocity_limit=30))
    CollisionFreeMotion(metas={}, type='collision_free', target=Pose(position=Vector3d(x=1, y=2, z=3), orientation=Vector3d(x=4, y=5, z=6), kinematic_configuration=None), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=30.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None, algorithm=RRTConnectAlgorithm(algorithm_name='RRTConnectAlgorithm', max_iterations=10000, max_step_size=0.1, adaptive_step_size=True, step_size=None, apply_smoothing=True, apply_blending=True))
    """

    type: Literal["collision_free"] = "collision_free"
    target: Pose | tuple[float, ...]
    settings: MotionSettings = MotionSettings()
    collision_setup: api.models.CollisionSetup | None = None

    algorithm: api.models.CollisionFreeAlgorithm = api.models.RRTConnectAlgorithm()

    def to_api_model(self) -> api.models.PlanCollisionFreeRequest:
        """"""
        # TODO: use data structure and API model are too different.
        # don't return the API model here
        raise NotImplementedError("CollisionFreeMotion.to_api_model is not implemented yet")


class MultiCollisionFreeMotion(Action):
    """A synchronized collision-free motion across several motion groups.

    The multi-group counterpart of :class:`CollisionFreeMotion`: one action that
    targets every group at once and is planned by the multi-motion-group RRT
    endpoint into a single, time-synchronized trajectory (all groups sharing one
    ``times``/``locations``). Unlike :class:`CollisionFreeMotion` it carries no
    single ``target``; the per-group targets live in :attr:`targets`.

    Attributes:
        targets: The target per motion group, keyed by name — a pose (resolved
            via inverse kinematics) or a joint tuple.
        settings: Optional motion settings per group, folded into that group's
            planning limits. Groups left out plan at their controller limits.
        collision_setups: Optional cross-group and environment collision setup
            per group. ``None`` plans against the groups' own geometry only.
        algorithm: RRT settings shared by the whole search. ``None`` uses the
            endpoint default.
    """

    type: Literal["multi_collision_free"] = "multi_collision_free"
    targets: dict[str, Pose | tuple[float, ...]]
    settings: dict[str, MotionSettings] = {}
    collision_setups: dict[str, api.models.MultiCollisionSetup] | None = None
    algorithm: api.models.RRTConnectAlgorithm | None = None

    def is_motion(self) -> bool:
        return True

    def to_api_model(self) -> api.models.MultiSearchCollisionFreeRequest:
        # The request also needs each group's setup and start joints, which the
        # action does not carry; MultiMotionGroupPlanner assembles it instead.
        raise NotImplementedError("MultiCollisionFreeMotion.to_api_model is not implemented")


def multi_collision_free(
    targets: dict[str, Pose | tuple[float, ...]],
    settings: dict[str, MotionSettings] | None = None,
    collision_setups: dict[str, api.models.MultiCollisionSetup] | None = None,
    algorithm: api.models.RRTConnectAlgorithm | None = None,
    **kwargs: Any,
) -> MultiCollisionFreeMotion:
    """Convenience factory for a synchronized collision-free motion.

    Args:
        targets: The target per motion group (pose or joint tuple), keyed by name.
        settings: Optional per-group motion settings.
        collision_setups: Optional per-group collision setup.
        algorithm: Shared RRT settings; ``None`` uses the endpoint default.

    Returns:
        The multi-motion-group collision-free motion.
    """
    kwargs.update(utils.get_caller_metas())
    return MultiCollisionFreeMotion(
        targets=targets,
        settings=settings or {},
        collision_setups=collision_setups,
        algorithm=algorithm,
        metas=kwargs,
    )


def collision_free(
    target: Pose | tuple[float, ...],
    settings: MotionSettings = MotionSettings(),
    collision_setup: api.models.CollisionSetup | None = None,
    algorithm: api.models.CollisionFreeAlgorithm = api.models.RRTConnectAlgorithm(),
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
    CollisionFreeMotion(metas={'line_number': 1}, type='collision_free', target=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), settings=MotionSettings(blending=None, blending_auto=None, blending_radius=None, joint_velocity_limits=None, joint_acceleration_limits=None, joint_jerk_limits=None, tcp_velocity_limit=50.0, tcp_acceleration_limit=None, tcp_jerk_limit=None, tcp_orientation_velocity_limit=None, tcp_orientation_acceleration_limit=None, tcp_orientation_jerk_limit=None, position_zone_radius=None, min_blending_velocity=None), collision_setup=None, algorithm=RRTConnectAlgorithm(algorithm_name='RRTConnectAlgorithm', max_iterations=10000, max_step_size=0.1, adaptive_step_size=True, step_size=None, apply_smoothing=True, apply_blending=True))
    """
    kwargs.update(utils.get_caller_metas())
    return CollisionFreeMotion(
        target=target,
        settings=settings,
        collision_setup=collision_setup,
        algorithm=algorithm,
        metas=kwargs,
    )
