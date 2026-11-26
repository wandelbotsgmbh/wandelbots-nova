"""Target-vs-actual tracking time-series logging."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from novapolicy.rerun.constants import (
    TCP_ERROR_VECTOR_COLOR,
    TCP_TARGET_TRAIL_COLOR,
    TRAIL_WIDTH_UI,
)
from novapolicy.rerun.context import log_points, log_trail
from novapolicy.rerun.kinematics import to_robot_frame
import rerun as rr

_TCP_POSITION_DIMS = 3
_TCP_DIMS = 6

if TYPE_CHECKING:
    from nova.types import Pose
    from novapolicy.rerun.context import RerunContext
    from rerun import RecordingStream


def log_joint_tracking(
    ctx: RerunContext, mg_id: str, target: list[float], actual: list[float], step: int
) -> None:
    """Log commanded joint positions, actual positions, and tracking error."""
    if not target or not actual:
        return

    n = min(len(target), len(actual))
    target_joints = target[:n]
    actual_joints = actual[:n]
    joint_error = [target_joints[i] - actual_joints[i] for i in range(n)]

    ctx.set_time(step)
    recording = ctx.recording
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
    ctx: RerunContext,
    mg_id: str,
    target_position: list[float],
    actual: Pose,
    step: int,
    *,
    target_trail: list[list[float]] | None = None,
) -> None:
    """Log TCP position/error derived from a commanded joint target, both in the robot's frame
    (the actual pose moved there by the context's pose frame)."""
    if len(target_position) < _TCP_POSITION_DIMS:
        return

    target_position = target_position[:_TCP_POSITION_DIMS]
    actual_position = to_robot_frame(list(actual.position), ctx.pose_frames.get(mg_id))
    position_error = [target_position[i] - actual_position[i] for i in range(3)]

    ctx.set_time(step)
    _log_tcp_position_series(
        mg_id,
        target_position=target_position,
        actual_position=actual_position,
        position_error=position_error,
        recording=ctx.recording,
    )
    _log_tcp_3d(ctx, mg_id, target_position, actual_position, target_trail)


def log_tcp_tracking(
    ctx: RerunContext,
    mg_id: str,
    target: list[float],
    actual: Pose,
    step: int,
    *,
    target_trail: list[list[float]] | None = None,
) -> None:
    """Log commanded TCP pose, actual TCP pose, and tracking error."""
    if len(target) < _TCP_DIMS:
        return

    target_position = target[:3]
    target_orientation = target[3:6]
    actual_position = to_robot_frame(list(actual.position), ctx.pose_frames.get(mg_id))
    actual_orientation = list(actual.orientation)
    position_error = [target_position[i] - actual_position[i] for i in range(3)]
    orientation_error = [target_orientation[i] - actual_orientation[i] for i in range(3)]

    ctx.set_time(step)
    _log_tcp_scalar_series(
        mg_id,
        target_position=target_position,
        target_orientation=target_orientation,
        actual_position=actual_position,
        actual_orientation=actual_orientation,
        position_error=position_error,
        orientation_error=orientation_error,
        recording=ctx.recording,
    )
    _log_tcp_3d(ctx, mg_id, target_position, actual_position, target_trail)


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
    ctx: RerunContext,
    mg_id: str,
    target_position: list[float],
    actual_position: list[float],
    target_trail: list[list[float]] | None,
) -> None:
    """Draw the commanded path (a marker until it is long enough) and the error vector."""
    if target_trail is not None and not log_trail(
        ctx,
        f"policy/{mg_id}/tcp_target_trail",
        target_trail,
        target_position,
        TCP_TARGET_TRAIL_COLOR,
    ):
        log_points(
            ctx, f"policy/{mg_id}/tcp_target_point", [target_position], TCP_TARGET_TRAIL_COLOR
        )
    rr.log(
        f"policy/{mg_id}/tcp_error_vector",
        rr.LineStrips3D(
            [[actual_position, target_position]],
            colors=[TCP_ERROR_VECTOR_COLOR],
            radii=rr.components.Radius.ui_points(TRAIL_WIDTH_UI),
        ),
        recording=ctx.recording,
    )
