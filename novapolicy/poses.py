"""TCP poses as a policy sees them: in its coordinate system and rotation representation.

NOVA reports and commands TCP poses as ``[x, y, z, rx, ry, rz]`` (mm, rotation vector) in
the controller's world coordinate system. A policy may have been trained on another
rotation representation (NOVA's ``OrientationType``: rotation vector, quaternion
``[x, y, z, w]`` or one of the Euler conventions) and on poses in another coordinate
system. Every observation builder and the action path use these functions, so the
conversion is the same for every policy client.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from nova import api

if TYPE_CHECKING:
    from collections.abc import Sequence

OrientationType = api.models.OrientationType
ROTATION_VECTOR = OrientationType.ROTATION_VECTOR
QUATERNION = OrientationType.QUATERNION
_QUATERNION_WIDTH = 4
_ANGLES_WIDTH = 3
_INTRINSIC = "EULER_ANGLES_INTRINSIC_"
_EXTRINSIC = "EULER_ANGLES_EXTRINSIC_"


def orientation_width(orientation_type: OrientationType) -> int:
    """Number of orientation values: 4 for a quaternion, else 3."""
    return _QUATERNION_WIDTH if orientation_type == QUATERNION else _ANGLES_WIDTH


def _euler_sequence(orientation_type: OrientationType) -> str | None:
    """SciPy's axis sequence: upper case intrinsic, lower case extrinsic."""
    name = orientation_type.value
    if name.startswith(_INTRINSIC):
        return name.removeprefix(_INTRINSIC).upper()
    if name.startswith(_EXTRINSIC):
        return name.removeprefix(_EXTRINSIC).lower()
    return None


def from_rotvec(rotvec: Sequence[float], orientation_type: OrientationType) -> list[float]:
    """A rotation vector in ``orientation_type`` (quaternion as ``[x, y, z, w]``)."""
    if orientation_type == ROTATION_VECTOR:
        return [float(v) for v in rotvec]
    rotation = Rotation.from_rotvec(np.asarray(rotvec, dtype=float))
    if orientation_type == QUATERNION:
        return rotation.as_quat().tolist()  # scalar last, as NOVA's API
    sequence = _euler_sequence(orientation_type)
    if sequence is None:
        msg = f"Unsupported orientation type {orientation_type!r}"
        raise ValueError(msg)
    return rotation.as_euler(sequence).tolist()


def to_rotvec(values: Sequence[float], orientation_type: OrientationType) -> list[float]:
    """An orientation in ``orientation_type`` as a rotation vector."""
    if orientation_type == ROTATION_VECTOR:
        return [float(v) for v in values]
    if orientation_type == QUATERNION:
        return Rotation.from_quat(np.asarray(values, dtype=float)).as_rotvec().tolist()
    sequence = _euler_sequence(orientation_type)
    if sequence is None:
        msg = f"Unsupported orientation type {orientation_type!r}"
        raise ValueError(msg)
    return Rotation.from_euler(sequence, np.asarray(values, dtype=float)).as_rotvec().tolist()


def pose_matrix(pose: Sequence[float]) -> np.ndarray:
    """4x4 transform of ``[x, y, z, rx, ry, rz]`` (mm, rotation vector)."""
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(np.asarray(pose[3:6], dtype=float)).as_matrix()
    t[:3, 3] = np.asarray(pose[:3], dtype=float)
    return t


def matrix_pose(t: np.ndarray) -> list[float]:
    """``[x, y, z, rx, ry, rz]`` of a 4x4 transform."""
    return [*t[:3, 3].tolist(), *Rotation.from_matrix(t[:3, :3]).as_rotvec().tolist()]


def world_to_frame(pose: Sequence[float], frame: np.ndarray | None) -> list[float]:
    """A world pose expressed in ``frame`` (the frame's pose in world); identity if None."""
    if frame is None:
        return [float(v) for v in pose]
    return matrix_pose(np.linalg.inv(frame) @ pose_matrix(pose))


def frame_to_world(pose: Sequence[float], frame: np.ndarray | None) -> list[float]:
    """A pose in ``frame`` expressed in world; identity if None."""
    if frame is None:
        return [float(v) for v in pose]
    return matrix_pose(frame @ pose_matrix(pose))
