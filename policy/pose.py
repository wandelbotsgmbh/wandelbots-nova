"""Pose conversion utilities for policy observations.

Converts Nova's rotation-vector poses to various representations used by
policy models (quaternion, rot6d).
"""

from __future__ import annotations

import math

_ANGLE_EPSILON = 1e-8


def pose_to_tcp(pose: object, fmt: str) -> list[float]:
    """Convert a Nova Pose to TCP values in the requested format.

    Parameters
    ----------
    pose:
        A Nova ``Pose`` object with ``.position`` (x, y, z in mm) and
        ``.orientation`` (rotation vector in radians).
    fmt:
        One of ``"rotation_vector"``, ``"quaternion"``, ``"rot6d"``.

    Returns
    -------
    list[float]
        - ``rotation_vector``: [x, y, z, rx, ry, rz] — 6 values
        - ``quaternion``: [x, y, z, qx, qy, qz, qw] — 7 values
        - ``rot6d``: [x, y, z, r1x, r1y, r1z, r2x, r2y, r2z] — 9 values

    Position is in meters (Nova uses mm internally).
    """
    pos = pose.position  # type: ignore[union-attr]
    ori = pose.orientation  # type: ignore[union-attr]
    x = float(pos.x) / 1000.0
    y = float(pos.y) / 1000.0
    z = float(pos.z) / 1000.0

    if fmt == "rotation_vector":
        return [x, y, z, float(ori.x), float(ori.y), float(ori.z)]

    ax, ay, az = float(ori.x), float(ori.y), float(ori.z)
    angle = math.sqrt(ax * ax + ay * ay + az * az)

    if fmt == "quaternion":
        if angle < _ANGLE_EPSILON:
            return [x, y, z, 0.0, 0.0, 0.0, 1.0]
        half = angle / 2.0
        s = math.sin(half) / angle
        return [x, y, z, ax * s, ay * s, az * s, math.cos(half)]

    # rot6d (GR00T format): first two columns of rotation matrix
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
