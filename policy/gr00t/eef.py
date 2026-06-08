"""End-effector pose conversion for GR00T.

Converts Nova's rotation-vector poses to the representations
expected by GR00T models (quaternion, rot6d).
"""

from __future__ import annotations

from enum import StrEnum
import math

_ANGLE_EPSILON = 1e-8


class TcpFormat(StrEnum):
    """TCP pose representation format."""

    ROTATION_VECTOR = "rotation_vector"
    """[x, y, z, rx, ry, rz] — 6 values."""

    QUATERNION = "quaternion"
    """[x, y, z, qx, qy, qz, qw] — 7 values."""

    ROT6D = "rot6d"
    """[x, y, z, r1x, r1y, r1z, r2x, r2y, r2z] — 9 values. GR00T format."""


def pose_to_eef(
    pose: object, fmt: TcpFormat | str, *, position_scale: float = 0.001
) -> list[float]:
    """Convert a Nova Pose to TCP values in the requested format.

    Parameters
    ----------
    pose:
        A Nova ``Pose`` object with ``.position`` (x, y, z in mm) and
        ``.orientation`` (rotation vector in radians).
    fmt:
        A ``TcpFormat`` enum value or one of
        ``"rotation_vector"``, ``"quaternion"``, ``"rot6d"``.
    position_scale:
        Multiply position values by this factor.
        Default ``0.001`` converts Nova's mm to meters.
        Use ``1.0`` to keep mm.

    Returns
    -------
    list[float]
        - ``rotation_vector``: [x, y, z, rx, ry, rz] — 6 values
        - ``quaternion``: [x, y, z, qx, qy, qz, qw] — 7 values
        - ``rot6d``: [x, y, z, r1x, r1y, r1z, r2x, r2y, r2z] — 9 values
    """
    pos = pose.position  # type: ignore[union-attr]
    ori = pose.orientation  # type: ignore[union-attr]
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

    # Rodrigues rotation formula → rotation matrix
    kx, ky, kz = ax / angle, ay / angle, az / angle
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1.0 - c

    # First column of rotation matrix
    r00 = kx * kx * v + c
    r10 = ky * kx * v + kz * s
    r20 = kz * kx * v - ky * s

    # Second column of rotation matrix
    r01 = kx * ky * v - kz * s
    r11 = ky * ky * v + c
    r21 = kz * ky * v + kx * s

    return [x, y, z, r00, r10, r20, r01, r11, r21]
