"""Ergonomic constructors for the NOVA CommandRoutine command types.

These factories wrap the generated ``api.models.*`` command routine models so a
``CommandRoutineBuilder`` (and its callers) can assemble a routine without importing
the generated classes directly.
"""

from __future__ import annotations

from typing import Any

from nova import api

# --- I/O values ------------------------------------------------------------


def io_value(io: str, value: bool | int | float) -> api.models.IOValue:
    """Build the discriminated ``IOValue`` for a digital, integer or analog output.

    ``bool`` maps to a boolean value, ``int`` to an integer value (transmitted as a
    string to avoid precision loss) and ``float`` to an analog value.
    """
    # bool is a subclass of int, so it must be checked first.
    if isinstance(value, bool):
        return api.models.IOBooleanValue(io=io, value=value)
    if isinstance(value, int):
        return api.models.IOIntegerValue(io=io, value=str(value))
    if isinstance(value, float):
        return api.models.IOFloatValue(io=io, value=value)
    raise TypeError(f"Unsupported I/O value type: {type(value)!r}")


# --- Pose references -------------------------------------------------------


def local_pose(pose_id: str) -> api.models.LocalPoseReference:
    """Reference a pose taught in the enclosing dataset."""
    return api.models.LocalPoseReference(pose_id=pose_id)


def joint_position(joints: list[float]) -> api.models.JointPositionReference:
    """Target a joint configuration (radians)."""
    return api.models.JointPositionReference(joints=joints)


def _as_pose_ref(target: api.models.PoseRef | str) -> api.models.PoseRef:
    """Coerce a bare pose id into a :func:`local_pose` reference."""
    return local_pose(target) if isinstance(target, str) else target


def _as_joint_ref(target: api.models.PoseRef | list[float]) -> api.models.PoseRef:
    """Coerce a bare joint list into a :func:`joint_position` reference."""
    return joint_position(target) if isinstance(target, list) else target


# --- Path types ------------------------------------------------------------


def path_line() -> api.models.PathTypeLine:
    """A straight Cartesian path."""
    return api.models.PathTypeLine()


def path_cartesian_ptp() -> api.models.PathTypeCartesianPTP:
    """A Cartesian point-to-point path."""
    return api.models.PathTypeCartesianPTP()


def path_joint_ptp() -> api.models.PathTypeJointPTP:
    """A joint-space point-to-point path."""
    return api.models.PathTypeJointPTP()


def path_circle(via_pose: api.models.PoseRef) -> api.models.PathTypeCircle:
    """A circular Cartesian path shaped by ``via_pose``."""
    return api.models.PathTypeCircle(via_pose=via_pose)


# --- Triggers --------------------------------------------------------------


def at_path_fraction(value: float) -> api.models.PathFractionTrigger:
    """Place an overlay command at a fraction ``[0, 1)`` of the anchor motion segment."""
    return api.models.PathFractionTrigger(value=value)


def at_distance(
    millimeters: float, reference: api.models.AtReference
) -> api.models.DistanceTrigger:
    """Place an overlay command at a Cartesian distance from a motion boundary."""
    return api.models.DistanceTrigger(millimeters=millimeters, reference=reference)


def at_time(seconds: float, reference: api.models.AtReference) -> api.models.TimeTrigger:
    """Place an overlay command at a duration from a motion boundary."""
    return api.models.TimeTrigger(seconds=seconds, reference=reference)


# --- Commands --------------------------------------------------------------


def motion(
    target: api.models.PoseRef,
    path_type: api.models.PathType,
    *,
    settings: api.models.MotionSettings | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.ExplicitPathMotionCommand:
    """A motion command whose path is authored explicitly."""
    return api.models.ExplicitPathMotionCommand(
        target=target,
        path_type=path_type,
        motion_settings=settings,
        intent=intent,
        metadata=metadata,
    )


def generated_motion(
    target: api.models.PoseRef,
    generator: api.models.MotionGenerator,
    *,
    settings: api.models.MotionSettings | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.GeneratedPathMotionCommand:
    """A motion command whose path the planner generates from ``generator``."""
    return api.models.GeneratedPathMotionCommand(
        target=target,
        generator=generator,
        motion_settings=settings,
        intent=intent,
        metadata=metadata,
    )


def move_linear(
    target: api.models.PoseRef | str,
    *,
    settings: api.models.MotionSettings | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.ExplicitPathMotionCommand:
    """A straight-line motion to ``target`` (a pose id or reference)."""
    return motion(
        _as_pose_ref(target), path_line(), settings=settings, intent=intent, metadata=metadata
    )


def move_cartesian_ptp(
    target: api.models.PoseRef | str,
    *,
    settings: api.models.MotionSettings | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.ExplicitPathMotionCommand:
    """A Cartesian point-to-point motion to ``target`` (a pose id or reference)."""
    return motion(
        _as_pose_ref(target),
        path_cartesian_ptp(),
        settings=settings,
        intent=intent,
        metadata=metadata,
    )


def move_joint_ptp(
    target: api.models.PoseRef | list[float],
    *,
    settings: api.models.MotionSettings | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.ExplicitPathMotionCommand:
    """A joint-space point-to-point motion to ``target`` (joints or a reference)."""
    return motion(
        _as_joint_ref(target), path_joint_ptp(), settings=settings, intent=intent, metadata=metadata
    )


def move_circular(
    target: api.models.PoseRef | str,
    via_pose: api.models.PoseRef,
    *,
    settings: api.models.MotionSettings | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.ExplicitPathMotionCommand:
    """A circular motion to ``target`` (a pose id or reference) shaped by ``via_pose``."""
    return motion(
        _as_pose_ref(target),
        path_circle(via_pose),
        settings=settings,
        intent=intent,
        metadata=metadata,
    )


def set_io(
    io: str,
    value: bool | int | float,
    *,
    io_origin: api.models.IOOrigin = api.models.IOOrigin.CONTROLLER,
    at: api.models.AtTrigger | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.SetIOCommand:
    """A non-blocking output command that writes ``value`` to ``io``."""
    return api.models.SetIOCommand(
        io_value=io_value(io, value), io_origin=io_origin, at=at, intent=intent, metadata=metadata
    )


def wait_for_time(
    duration_ms: int,
    *,
    at: api.models.AtTrigger | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.WaitForTimeCommand:
    """Block at a trajectory location for a fixed duration."""
    return api.models.WaitForTimeCommand(
        duration_ms=duration_ms, at=at, intent=intent, metadata=metadata
    )


def wait_for_io(
    condition: api.models.IOExpression,
    *,
    timeout_ms: int | None = None,
    at: api.models.AtTrigger | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.WaitForIOCommand:
    """Block at a trajectory location until an I/O expression becomes true."""
    return api.models.WaitForIOCommand(
        condition=condition, timeout_ms=timeout_ms, at=at, intent=intent, metadata=metadata
    )


def pause_on_io(
    condition: api.models.IOExpression,
    *,
    at: api.models.AtTrigger | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.PauseOnIOCommand:
    """Pause while an I/O expression is true."""
    return api.models.PauseOnIOCommand(condition=condition, at=at, intent=intent, metadata=metadata)


def marker(
    name: str,
    *,
    payload: dict[str, Any] | None = None,
    at: api.models.AtTrigger | None = None,
    intent: str | None = None,
    metadata: dict[str, str] | None = None,
) -> api.models.MarkerCommand:
    """Emit a named point of interest when execution crosses a trajectory location."""
    return api.models.MarkerCommand(
        name=name, payload=payload, at=at, intent=intent, metadata=metadata
    )
