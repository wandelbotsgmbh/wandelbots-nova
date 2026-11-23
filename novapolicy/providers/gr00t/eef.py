"""Convert NOVA rotation-vector poses to the TCP formats GR00T models take (quaternion, rot6d)."""

from __future__ import annotations

from enum import StrEnum
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nova.types import Pose

_ANGLE_EPSILON = 1e-8


class TcpFormat(StrEnum):
    """TCP pose representation format."""

    ROTATION_VECTOR = "rotation_vector"
    """[x, y, z, rx, ry, rz] — 6 values."""

    QUATERNION = "quaternion"
    """[x, y, z, qx, qy, qz, qw] — 7 values."""

    ROT6D = "rot6d"
    """[x, y, z, r1x, r1y, r1z, r2x, r2y, r2z] — 9 values. GR00T format."""


def pose_to_eef(  # ruff: ignore[too-many-locals]
    pose: Pose, fmt: TcpFormat | str, *, position_scale: float = 0.001
) -> list[float]:
    """Convert a NOVA pose (mm, rotation vector) to TCP values in the requested format.

    Args:
        pose: The pose.
        fmt: A ``TcpFormat`` or its value.
        position_scale: Factor on the position; ``0.001`` (default) gives metres.

    Returns:
        The position, then 3, 4 or 6 orientation values (see ``TcpFormat``).
    """
    pos = pose.position
    ori = pose.orientation
    x = float(pos.x) * position_scale
    y = float(pos.y) * position_scale
    z = float(pos.z) * position_scale

    fmt_str = str(fmt)

    if fmt_str == TcpFormat.ROTATION_VECTOR:
        return [x, y, z, float(ori.x), float(ori.y), float(ori.z)]

    ax, ay, az = float(ori.x), float(ori.y), float(ori.z)
    angle = math.sqrt(ax * ax + ay * ay + az * az)

    if fmt_str == TcpFormat.QUATERNION:
        if angle < _ANGLE_EPSILON:
            return [x, y, z, 0.0, 0.0, 0.0, 1.0]
        half = angle / 2.0
        s = math.sin(half) / angle
        return [x, y, z, ax * s, ay * s, az * s, math.cos(half)]

    # rot6d (GR00T format): first two columns of rotation matrix
    if fmt_str != TcpFormat.ROT6D:
        msg = f"Unknown TcpFormat: {fmt!r}"
        raise ValueError(msg)
    if angle < _ANGLE_EPSILON:
        return [x, y, z, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]

    kx, ky, kz = ax / angle, ay / angle, az / angle
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1.0 - c

    r00 = kx * kx * v + c
    r10 = ky * kx * v + kz * s
    r20 = kz * kx * v - ky * s

    r01 = kx * ky * v - kz * s
    r11 = ky * ky * v + c
    r21 = kz * ky * v + kx * s

    return [x, y, z, r00, r10, r20, r01, r11, r21]
