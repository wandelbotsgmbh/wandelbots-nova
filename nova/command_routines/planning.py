"""Convert a ``CommandRoutine`` into the SDK's ``Action`` list.

There is no NOVA API endpoint that plans or executes a ``CommandRoutine`` directly. Instead,
:func:`actions_from_command_routine` translates its commands into the ``Action`` objects that
``MotionGroup.plan``/``execute``/``plan_and_execute`` already understand, so a routine can be
planned and executed through the existing pipeline.

Only the subset of ``CommandRoutine`` that has an equivalent today is supported:

- ``ExplicitPathMotionCommand`` and ``GeneratedPathMotionCommand`` (without a direction
  ``constraint``, which has no ``Action`` equivalent yet).
- Pose targets that are inline (``InlinePoseReference``), joint-space
  (``JointPositionReference``), a ``DatasetPoseReference`` that already carries a
  ``resolved_pose``, or a ``LocalPoseReference`` when a ``pose_resolver`` is supplied (see
  :func:`nova.command_routines.resolve_dataset_poses` to build one from a dataset export).
- ``SetIOCommand`` and ``WaitForTimeCommand`` placed at the default location (``at=None``),
  i.e. immediately after the preceding motion. Any other ``at`` trigger (``at_path_fraction``,
  ``at_distance``, ``at_time``) has no equivalent in the current motion-boundary-only overlay
  mechanism.
- A joint-space path (``PathTypeJointPTP``) still requires a joint-position target -- planning
  a joint PTP to a *Cartesian* pose has no ``Action`` equivalent yet, even when that pose
  resolves successfully.

``WaitForIOCommand``, ``PauseOnIOCommand`` and ``MarkerCommand`` have no ``Action`` equivalent
at all. Using any of the unsupported features raises :class:`UnsupportedCommandRoutineFeature`.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping

from nova import api
from nova.actions.base import Action
from nova.actions.io import io_write
from nova.actions.mock import wait
from nova.actions.motions import CartesianPTP, Circular, CollisionFreeMotion, JointPTP, Linear
from nova.types import MotionSettings, Pose

PoseResolver = Mapping[str, Pose] | Callable[[str], Pose]


class UnsupportedCommandRoutineFeature(NotImplementedError):
    """A CommandRoutine command or setting has no plan/execute equivalent yet."""


def actions_from_command_routine(
    routine: api.models.CommandRoutine, *, pose_resolver: PoseResolver | None = None
) -> list[Action]:
    """Convert ``routine.commands`` into the ``Action`` list plan/execute expects.

    Motion commands that do not carry their own ``motion_settings`` fall back to
    ``routine.default_motion_settings``, mirroring :func:`nova.command_routines.with_settings`.

    Args:
        routine: The CommandRoutine to convert.
        pose_resolver: A ``pose_id -> Pose`` mapping or callable used to resolve
            ``LocalPoseReference`` targets. Without one, a ``LocalPoseReference`` raises
            :class:`UnsupportedCommandRoutineFeature`. Build one from a dataset export with
            :func:`nova.command_routines.resolve_dataset_poses`.

    Raises:
        UnsupportedCommandRoutineFeature: for a command or placement that has no equivalent in
            the current plan/execute pipeline (see module docstring).
    """
    actions: list[Action] = []
    for command in routine.commands:
        if isinstance(command, api.models.ExplicitPathMotionCommand):
            actions.append(
                _explicit_motion_action(
                    command,
                    default_settings=routine.default_motion_settings,
                    pose_resolver=pose_resolver,
                )
            )
        elif isinstance(command, api.models.GeneratedPathMotionCommand):
            actions.append(
                _generated_motion_action(
                    command,
                    default_settings=routine.default_motion_settings,
                    pose_resolver=pose_resolver,
                )
            )
        else:
            actions.append(_overlay_action(command))
    return actions


def _explicit_motion_action(
    command: api.models.ExplicitPathMotionCommand,
    *,
    default_settings: api.models.MotionSettings | None,
    pose_resolver: PoseResolver | None,
) -> Action:
    settings = _motion_settings_from_api(command.motion_settings or default_settings)
    target = _resolve_pose_ref(command.target, pose_resolver)
    path_type = command.path_type

    if isinstance(path_type, api.models.PathTypeLine):
        return Linear(target=_require_pose(target, "A line motion"), settings=settings)
    if isinstance(path_type, api.models.PathTypeCartesianPTP):
        return CartesianPTP(
            target=_require_pose(target, "A Cartesian PTP motion"), settings=settings
        )
    if isinstance(path_type, api.models.PathTypeJointPTP):
        return JointPTP(target=_require_joints(target, "A joint PTP motion"), settings=settings)
    if isinstance(path_type, api.models.PathTypeCircle):
        via = _resolve_pose_ref(path_type.via_pose, pose_resolver)
        return Circular(
            target=_require_pose(target, "A circular motion"),
            intermediate=_require_pose(via, "A circular motion's via pose"),
            settings=settings,
        )
    raise UnsupportedCommandRoutineFeature(f"Unsupported path type: {type(path_type)!r}")


def _generated_motion_action(
    command: api.models.GeneratedPathMotionCommand,
    *,
    default_settings: api.models.MotionSettings | None,
    pose_resolver: PoseResolver | None,
) -> Action:
    if command.generator.constraint is not None:
        raise UnsupportedCommandRoutineFeature(
            "GeneratedPathMotionCommand.generator.constraint (direction constraints) has no "
            "plan/execute equivalent yet."
        )
    settings = _motion_settings_from_api(command.motion_settings or default_settings)
    target = _resolve_pose_ref(command.target, pose_resolver)
    return CollisionFreeMotion(
        target=target, settings=settings, algorithm=command.generator.algorithm
    )


OverlayCommand = (
    api.models.SetIOCommand
    | api.models.WaitForIOCommand
    | api.models.PauseOnIOCommand
    | api.models.WaitForTimeCommand
    | api.models.MarkerCommand
)


def _overlay_action(command: OverlayCommand) -> Action:
    if command.at is not None:
        raise UnsupportedCommandRoutineFeature(
            f"{type(command).__name__}.at triggers are not yet supported by plan/execute; "
            "only the default placement (at=None, immediately after the preceding motion) is."
        )
    if isinstance(command, api.models.SetIOCommand):
        return io_write(
            key=command.io_value.io,
            value=_io_value_to_python(command.io_value),
            origin=command.io_origin,
        )
    if isinstance(command, api.models.WaitForTimeCommand):
        return wait(command.duration_ms / 1000)
    raise UnsupportedCommandRoutineFeature(
        f"{type(command).__name__} has no plan/execute equivalent yet."
    )


def _io_value_to_python(io_value: api.models.IOValue) -> bool | int | float:
    if isinstance(io_value, api.models.IOBooleanValue):
        return io_value.value
    if isinstance(io_value, api.models.IOIntegerValue):
        return int(io_value.value)
    if isinstance(io_value, api.models.IOFloatValue):
        return io_value.value
    raise UnsupportedCommandRoutineFeature(f"Unsupported I/O value type: {type(io_value)!r}")


def _resolve_pose_ref(
    ref: api.models.PoseRef, pose_resolver: PoseResolver | None
) -> Pose | tuple[float, ...]:
    if isinstance(ref, api.models.JointPositionReference):
        return tuple(ref.joints)
    if isinstance(ref, api.models.InlinePoseReference):
        if ref.coordinate_system is not None:
            raise UnsupportedCommandRoutineFeature(
                "InlinePoseReference.coordinate_system is not yet supported by plan/execute."
            )
        return Pose.from_api_model(ref.pose, kinematic_configuration=ref.kinematic_configuration)
    if isinstance(ref, api.models.DatasetPoseReference):
        if ref.resolved_pose is None:
            raise UnsupportedCommandRoutineFeature(
                f"DatasetPoseReference {ref.dataset_pose!r} has no resolved_pose; resolve it "
                "against its dataset before planning."
            )
        if ref.resolved_pose.coordinate_system is not None:
            raise UnsupportedCommandRoutineFeature(
                "DatasetPoseReference.resolved_pose.coordinate_system is not yet supported "
                "by plan/execute."
            )
        return Pose.from_api_model(
            ref.resolved_pose.pose,
            kinematic_configuration=ref.resolved_pose.kinematic_configuration,
        )
    if isinstance(ref, api.models.LocalPoseReference):
        if pose_resolver is None:
            raise UnsupportedCommandRoutineFeature(
                f"LocalPoseReference {ref.pose_id!r} cannot be resolved: no pose_resolver was "
                "given. Pass one to actions_from_command_routine (see "
                "resolve_dataset_poses() to build one from a dataset export)."
            )
        try:
            pose = (
                pose_resolver[ref.pose_id]
                if isinstance(pose_resolver, Mapping)
                else pose_resolver(ref.pose_id)
            )
        except KeyError:
            raise UnsupportedCommandRoutineFeature(
                f"pose_resolver has no entry for LocalPoseReference {ref.pose_id!r}."
            ) from None
        return pose
    raise UnsupportedCommandRoutineFeature(f"Unsupported pose reference type: {type(ref)!r}")


def _require_pose(target: Pose | tuple[float, ...], what: str) -> Pose:
    if not isinstance(target, Pose):
        raise UnsupportedCommandRoutineFeature(f"{what} requires a Cartesian pose target.")
    return target


def _require_joints(target: Pose | tuple[float, ...], what: str) -> tuple[float, ...]:
    if not isinstance(target, tuple):
        raise UnsupportedCommandRoutineFeature(f"{what} requires a joint-position target.")
    return target


_UNSUPPORTED_BLENDING_ZONE_FIELDS = (
    "position_zone_percentage",
    "orientation_zone_radius",
    "orientation_zone_percentage",
    "joints_zone_radius",
    "joints_zone_percentage",
    "space",
)


def _motion_settings_from_api(settings: api.models.MotionSettings | None) -> MotionSettings:
    if settings is None:
        return MotionSettings()

    kwargs: dict = {}

    if settings.limits_override is not None:
        lo = settings.limits_override
        kwargs.update(
            joint_velocity_limits=tuple(lo.joint_velocity_limits)
            if lo.joint_velocity_limits
            else None,
            joint_acceleration_limits=tuple(lo.joint_acceleration_limits)
            if lo.joint_acceleration_limits
            else None,
            joint_jerk_limits=tuple(lo.joint_jerk_limits) if lo.joint_jerk_limits else None,
            tcp_velocity_limit=lo.tcp_velocity_limit,
            tcp_acceleration_limit=lo.tcp_acceleration_limit,
            tcp_jerk_limit=lo.tcp_jerk_limit,
            tcp_orientation_velocity_limit=lo.tcp_orientation_velocity_limit,
            tcp_orientation_acceleration_limit=lo.tcp_orientation_acceleration_limit,
            tcp_orientation_jerk_limit=lo.tcp_orientation_jerk_limit,
        )

    if settings.blending is not None:
        blending = settings.blending
        if isinstance(blending, api.models.BlendingAuto):
            kwargs["blending_auto"] = blending.min_velocity_in_percent
        elif isinstance(blending, api.models.BlendingPosition):
            if any(
                getattr(blending, field) is not None for field in _UNSUPPORTED_BLENDING_ZONE_FIELDS
            ):
                raise UnsupportedCommandRoutineFeature(
                    "Only BlendingPosition.position_zone_radius is supported by plan/execute; "
                    "the other blending-zone fields are not yet representable."
                )
            kwargs["blending_radius"] = blending.position_zone_radius
        else:
            raise UnsupportedCommandRoutineFeature(f"Unsupported blending type: {type(blending)!r}")

    return MotionSettings(**kwargs)
