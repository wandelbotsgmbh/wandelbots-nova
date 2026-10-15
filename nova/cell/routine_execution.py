"""Translate a CommandRoutine into the SDK's action model for execution."""

from nova import api
from nova.actions import Action
from nova.actions.io import WriteAction
from nova.actions.motions import CartesianPTP, Circular, JointPTP, Linear
from nova.datasets import Dataset, DatasetError
from nova.exceptions import PoseResolutionError, UnsupportedRoutineCommand
from nova.types import Pose
from nova.types.motion_settings import MotionSettings


def routine_to_actions(
    routine: api.models.CommandRoutine, dataset: Dataset | None = None
) -> list[Action]:
    """Translate the routine's commands into motion actions, in order.

    Poses taught in ``dataset`` (local pose references and poses given in a dataset frame)
    are resolved to world poses.

    Raises:
        UnsupportedRoutineCommand: If a command cannot be executed yet.
        PoseResolutionError: If a pose cannot be resolved to a world pose.
        Nothing is translated partially, so these are raised before any motion.
    """
    return [
        _to_action(routine, dataset, index, command)
        for index, command in enumerate(routine.commands)
    ]


def _to_action(
    routine: api.models.CommandRoutine,
    dataset: Dataset | None,
    index: int,
    command: api.models.Command,
):
    label = f"Command {index} ({command.type})"
    if isinstance(command, api.models.SetIOCommand):
        return _to_write(command)
    if not isinstance(command, api.models.ExplicitPathMotionCommand):
        raise UnsupportedRoutineCommand(f"{label} is not supported yet")

    settings = _to_settings(command.motion_settings or routine.default_motion_settings)
    path_type = command.path_type

    if isinstance(path_type, api.models.PathTypeJointPTP):
        if not isinstance(command.target, api.models.JointPositionReference):
            raise UnsupportedRoutineCommand(f"{label}: joint PTP needs a joint position target")
        return JointPTP(target=tuple(command.target.joints), settings=settings)

    target = _to_pose(routine, dataset, label, command.target)
    if isinstance(path_type, api.models.PathTypeLine):
        return Linear(target=target, settings=settings)
    if isinstance(path_type, api.models.PathTypeCartesianPTP):
        return CartesianPTP(target=target, settings=settings)
    if isinstance(path_type, api.models.PathTypeCircle):
        return Circular(
            target=target,
            intermediate=_to_pose(routine, dataset, label, path_type.via_pose),
            settings=settings,
        )
    raise UnsupportedRoutineCommand(
        f"{label}: path type {type(path_type).__name__} is not supported"
    )


def _to_write(command: api.models.SetIOCommand) -> WriteAction:
    """Map set_io onto a write; its ``at`` trigger is resolved against the planned path."""
    io = command.io_value
    value = int(io.value) if isinstance(io, api.models.IOIntegerValue) else io.value
    return WriteAction(
        key=io.io, value=value, device_id=None, origin=command.io_origin, at=command.at
    )


def _to_pose(
    routine: api.models.CommandRoutine, dataset: Dataset | None, label: str, ref: api.models.PoseRef
) -> Pose:
    """Resolve a pose reference to a world pose, keeping its kinematic configuration."""
    if isinstance(ref, api.models.LocalPoseReference):
        if dataset is None or ref.pose_id not in dataset.poses:
            where = "no dataset was given" if dataset is None else f"dataset '{dataset.id}'"
            raise PoseResolutionError(
                f"{label}: pose '{ref.pose_id}' cannot be resolved: not found, {where}"
            )
        dataset_pose = dataset.poses[ref.pose_id]
        try:
            world = dataset_pose.as_world()
        except DatasetError as error:
            raise PoseResolutionError(
                f"{label}: pose '{ref.pose_id}' cannot be resolved to world: {error}"
            ) from error
        return world.model_copy(
            update={"kinematic_configuration": dataset_pose.kinematic_configuration}
        )

    if not isinstance(ref, api.models.InlinePoseReference):
        raise UnsupportedRoutineCommand(
            f"{label}: {type(ref).__name__} targets are not supported yet"
        )
    if ref.tcp is not None and ref.tcp != routine.tcp:
        raise UnsupportedRoutineCommand(f"{label}: per-target TCP overrides are not supported")

    pose = Pose.from_api_model(ref.pose, ref.kinematic_configuration)
    if ref.frame is None:
        return pose
    if dataset is None or ref.frame not in dataset.frames:
        raise PoseResolutionError(
            f"{label}: frame '{ref.frame}' cannot be resolved: not found in the given dataset"
        )
    try:
        world = dataset.frames[ref.frame].as_world() @ pose
    except DatasetError as error:
        raise PoseResolutionError(
            f"{label}: frame '{ref.frame}' cannot be resolved to world: {error}"
        ) from error
    return world.model_copy(update={"kinematic_configuration": ref.kinematic_configuration})


def _to_settings(settings: api.models.MotionSettings | None) -> MotionSettings:
    if settings is None:
        return MotionSettings()
    limits = settings.limits_override or api.models.LimitsOverride()
    return MotionSettings(
        blending=settings.blending,
        joint_velocity_limits=_as_tuple(limits.joint_velocity_limits),
        joint_acceleration_limits=_as_tuple(limits.joint_acceleration_limits),
        joint_jerk_limits=_as_tuple(limits.joint_jerk_limits),
        tcp_velocity_limit=limits.tcp_velocity_limit,
        tcp_acceleration_limit=limits.tcp_acceleration_limit,
        tcp_jerk_limit=limits.tcp_jerk_limit,
        tcp_orientation_velocity_limit=limits.tcp_orientation_velocity_limit,
        tcp_orientation_acceleration_limit=limits.tcp_orientation_acceleration_limit,
        tcp_orientation_jerk_limit=limits.tcp_orientation_jerk_limit,
    )


def _as_tuple(values: list[float] | None) -> tuple[float, ...] | None:
    return tuple(values) if values else None
