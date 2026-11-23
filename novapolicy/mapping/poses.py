"""TCP poses as a policy sees them: in its coordinate system and rotation representation.

NOVA poses are ``[x, y, z, rx, ry, rz]`` (mm, rotation vector) in world; quaternions are
``[x, y, z, w]``. Shared by every client's observation and action path.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from nova import api
from novapolicy._sdk import get_controller_id

if TYPE_CHECKING:
    from collections.abc import Sequence

    from nova.cell.motion_group import MotionGroup

OrientationType = api.models.OrientationType
ROTATION_VECTOR = OrientationType.ROTATION_VECTOR
QUATERNION = OrientationType.QUATERNION
_QUATERNION_WIDTH = 4
_ANGLES_WIDTH = 3
_INTRINSIC = "EULER_ANGLES_INTRINSIC_"
_EXTRINSIC = "EULER_ANGLES_EXTRINSIC_"
_ROTVEC_EPSILON = 1e-12


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


async def coordinate_system_in_world(mg: MotionGroup, coordinate_system: str) -> np.ndarray:
    """The pose in world (4x4) of a controller's coordinate system, following its references."""
    api_client = mg._api_client
    controller = get_controller_id(mg)
    frame = pose_matrix([0.0] * 6)
    seen: set[str] = set()
    current: str | None = coordinate_system
    while current and current != "world" and current not in seen:  # '' is world too
        seen.add(current)
        cs = await api_client.controller_api.get_coordinate_system(
            cell=mg._cell, controller=controller, coordinate_system=current
        )
        orientation_type = cs.orientation_type or ROTATION_VECTOR
        rotvec = to_rotvec(list(cs.orientation or [0.0, 0.0, 0.0]), orientation_type)
        frame = pose_matrix([*(cs.position or [0.0, 0.0, 0.0]), *rotvec]) @ frame
        current = cs.reference_coordinate_system
    return frame


def unwrap_tcp_rotations(steps: list[list[float]], reference: list[float]) -> list[list[float]]:
    """Rewrite each TCP step's rotation vector as the equivalent one nearest its predecessor.

    ``angle * axis`` equals ``(angle + 2 pi k) * axis``; near pi measurements flip between ``v``
    and about ``-v``, and interpolating across that would turn the wrist half a turn.
    ``reference`` anchors the first step.
    """
    out: list[list[float]] = []
    previous = list(reference)
    for step in steps:
        rotvec = _equivalent_rotvec_near(step[3:6], previous)
        out.append([*step[:3], *rotvec, *step[6:]])
        previous = rotvec
    return out


def _equivalent_rotvec_near(rotvec: list[float], reference: list[float]) -> list[float]:
    angle = math.sqrt(sum(value * value for value in rotvec))
    if angle < _ROTVEC_EPSILON:
        # The identity has no axis of its own: its equivalents are the full turns about any
        # axis, so take the one along the reference nearest to it.
        reference_angle = math.sqrt(sum(value * value for value in reference))
        if reference_angle < _ROTVEC_EPSILON:
            return list(rotvec)
        turns = round(reference_angle / (2.0 * math.pi))
        scale = 2.0 * math.pi * turns / reference_angle
        return [v + r * scale for v, r in zip(rotvec, reference, strict=True)]
    axis = [value / angle for value in rotvec]
    # Equivalent vectors lie on the axis line at angle + 2 pi k; pick the one
    # nearest the reference's projection onto that line.
    projection = sum(a * r for a, r in zip(axis, reference, strict=True))
    turns = round((projection - angle) / (2.0 * math.pi))
    return [a * (angle + 2.0 * math.pi * turns) for a in axis]
