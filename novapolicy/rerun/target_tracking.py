"""Target-vs-actual tracking time-series logging."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

from novapolicy.rerun.constants import (
    MIN_LINE_STEPS,
    TCP_ERROR_VECTOR_COLOR,
    TCP_TARGET_TRAIL_COLOR,
    TRAIL_WIDTH_UI,
)
import rerun as rr

_TCP_POSITION_DIMS = 3
_TCP_DIMS = 6

if TYPE_CHECKING:
    from nova.types import Pose
    from rerun import RecordingStream


def log_joint_tracking(
    mg_id: str,
    target: list[float],
    actual: list[float],
    step: int,
    *,
    start_time: float,
    recording: RecordingStream | None,
) -> None:
    """Log commanded joint positions, actual positions, and tracking error."""
    if not target or not actual:
        return

    n = min(len(target), len(actual))
    target_joints = target[:n]
    actual_joints = actual[:n]
    joint_error = [target_joints[i] - actual_joints[i] for i in range(n)]

    _set_time(step, start_time, recording)
    for i, value in enumerate(target_joints):
        rr.log(f"policy/{mg_id}/joint_target/j{i}", rr.Scalars(value), recording=recording)
    for i, value in enumerate(actual_joints):
        rr.log(f"policy/{mg_id}/joint_actual/j{i}", rr.Scalars(value), recording=recording)
    for i, value in enumerate(joint_error):
        rr.log(f"policy/{mg_id}/joint_error/j{i}", rr.Scalars(value), recording=recording)
    rr.log(
        f"policy/{mg_id}/joint_error/norm_rad",
        rr.Scalars(math.dist(target_joints, actual_joints)),
        recording=recording,
    )


def log_joint_tcp_tracking(
    mg_id: str,
    target_position: list[float],
    actual: Pose,
    step: int,
    *,
    start_time: float,
    recording: RecordingStream | None,
    target_trail: list[list[float]] | None = None,
    max_trail_points: int = 500,
) -> None:
    """Log TCP position/error derived from a commanded joint target."""
    if len(target_position) < _TCP_POSITION_DIMS:
        return

    target_position = target_position[:_TCP_POSITION_DIMS]
    actual_position = list(actual.position)
    position_error = [target_position[i] - actual_position[i] for i in range(3)]

    _set_time(step, start_time, recording)
    _log_tcp_position_series(
        mg_id,
        target_position=target_position,
        actual_position=actual_position,
        position_error=position_error,
        recording=recording,
    )
    _log_tcp_3d(
        mg_id,
        target_position,
        actual_position,
        recording=recording,
        target_trail=target_trail,
        max_trail_points=max_trail_points,
    )


def log_tcp_tracking(
    mg_id: str,
    target: list[float],
    actual: Pose,
    step: int,
    *,
    start_time: float,
    recording: RecordingStream | None,
    target_trail: list[list[float]] | None = None,
    max_trail_points: int = 500,
) -> None:
    """Log commanded TCP pose, actual TCP pose, and tracking error."""
    if len(target) < _TCP_DIMS:
        return

    target_position = target[:3]
    target_orientation = target[3:6]
    actual_position = list(actual.position)
    actual_orientation = list(actual.orientation)
    position_error = [target_position[i] - actual_position[i] for i in range(3)]
    orientation_error = [target_orientation[i] - actual_orientation[i] for i in range(3)]

    _set_time(step, start_time, recording)
    _log_tcp_scalar_series(
        mg_id,
        target_position=target_position,
        target_orientation=target_orientation,
        actual_position=actual_position,
        actual_orientation=actual_orientation,
        position_error=position_error,
        orientation_error=orientation_error,
        recording=recording,
    )
    _log_tcp_3d(
        mg_id,
        target_position,
        actual_position,
        recording=recording,
        target_trail=target_trail,
        max_trail_points=max_trail_points,
    )


def _log_tcp_scalar_series(
    mg_id: str,
    *,
    target_position: list[float],
    target_orientation: list[float],
    actual_position: list[float],
    actual_orientation: list[float],
    position_error: list[float],
    orientation_error: list[float],
    recording: RecordingStream | None,
) -> None:
    _log_tcp_position_series(
        mg_id,
        target_position=target_position,
        actual_position=actual_position,
        position_error=position_error,
        recording=recording,
    )
    for name, value in zip(("rx", "ry", "rz"), target_orientation, strict=True):
        rr.log(
            f"policy/{mg_id}/tcp_target/orientation/{name}", rr.Scalars(value), recording=recording
        )
    for name, value in zip(("rx", "ry", "rz"), actual_orientation, strict=True):
        rr.log(
            f"policy/{mg_id}/tcp_actual/orientation/{name}", rr.Scalars(value), recording=recording
        )
    for name, value in zip(("drx", "dry", "drz"), orientation_error, strict=True):
        rr.log(
            f"policy/{mg_id}/tcp_error/orientation/{name}", rr.Scalars(value), recording=recording
        )

    rr.log(
        f"policy/{mg_id}/tcp_error/orientation_norm_rad",
        rr.Scalars(math.dist(target_orientation, actual_orientation)),
        recording=recording,
    )


def _log_tcp_position_series(
    mg_id: str,
    *,
    target_position: list[float],
    actual_position: list[float],
    position_error: list[float],
    recording: RecordingStream | None,
) -> None:
    """Log the position components shared by joint- and TCP-target tracking."""
    for name, value in zip(("x", "y", "z"), target_position, strict=True):
        rr.log(f"policy/{mg_id}/tcp_target/position/{name}", rr.Scalars(value), recording=recording)
    for name, value in zip(("x", "y", "z"), actual_position, strict=True):
        rr.log(f"policy/{mg_id}/tcp_actual/position/{name}", rr.Scalars(value), recording=recording)
    for name, value in zip(("dx", "dy", "dz"), position_error, strict=True):
        rr.log(f"policy/{mg_id}/tcp_error/position/{name}", rr.Scalars(value), recording=recording)
    rr.log(
        f"policy/{mg_id}/tcp_error/position_norm_mm",
        rr.Scalars(math.dist(target_position, actual_position)),
        recording=recording,
    )


def _log_tcp_3d(
    mg_id: str,
    target_position: list[float],
    actual_position: list[float],
    *,
    recording: RecordingStream | None,
    target_trail: list[list[float]] | None,
    max_trail_points: int,
) -> None:
    if target_trail is not None:
        target_trail.append(target_position)
        if len(target_trail) > max_trail_points:
            target_trail.pop(0)
        if len(target_trail) >= MIN_LINE_STEPS:
            rr.log(
                f"policy/{mg_id}/tcp_target_trail",
                rr.LineStrips3D(
                    [target_trail],
                    colors=[TCP_TARGET_TRAIL_COLOR],
                    radii=rr.components.Radius.ui_points(TRAIL_WIDTH_UI),
                ),
                recording=recording,
            )
        else:
            rr.log(
                f"policy/{mg_id}/tcp_target_point",
                rr.Points3D(
                    [target_position],
                    colors=[TCP_TARGET_TRAIL_COLOR],
                    radii=rr.components.Radius.ui_points(4.0),
                ),
                recording=recording,
            )

    rr.log(
        f"policy/{mg_id}/tcp_error_vector",
        rr.LineStrips3D(
            [[actual_position, target_position]],
            colors=[TCP_ERROR_VECTOR_COLOR],
            radii=rr.components.Radius.ui_points(TRAIL_WIDTH_UI),
        ),
        recording=recording,
    )


def _set_time(step: int, start_time: float, recording: RecordingStream | None) -> None:
    elapsed = time.monotonic() - start_time
    rr.set_time("policy_time", duration=elapsed, recording=recording)
    rr.set_time("policy_step", sequence=step, recording=recording)
