"""Forward-kinematics and frame helpers for the policy overlays, built on the public ``DHRobot``
API (ADR 0020)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from nova_rerun_bridge.dh_robot import DHRobot

    from nova.types import Pose

_ROTATION_EPS = 1e-12


def flange_matrix(dh_robot: DHRobot, joints: list[float]) -> np.ndarray:
    """4x4 flange (end-of-chain) transform for the given joints: ``calculate_joint_positions``
    with the full orientation kept, so a TCP offset can be applied in the flange frame."""
    matrix = np.asarray(dh_robot.pose_to_matrix(dh_robot.mounting), dtype=float)
    for dh_param, joint in zip(dh_robot.dh_parameters, joints, strict=False):
        matrix @= np.asarray(
            dh_robot.dh_transform(dh_param=dh_param, joint_position=joint), dtype=float
        )
    return matrix


def joint_tcp_position(
    dh_robot: DHRobot, joint_target: list[float], tcp_offset: np.ndarray | None
) -> list[float]:
    """TCP position for a joint target: flange FK plus the TCP offset if known."""
    if tcp_offset is None:
        return dh_robot.calculate_joint_positions(joint_target)[-1]
    flange = flange_matrix(dh_robot, joint_target)
    return (flange @ tcp_offset)[:3, 3].tolist()


def tcp_offset_matrix(offset: Pose) -> np.ndarray:
    """4x4 flange->TCP transform from a TCP offset pose (rotation-vector orientation)."""
    pos = (offset.position.x, offset.position.y, offset.position.z)
    rotvec = (offset.orientation.x, offset.orientation.y, offset.orientation.z)
    theta = float(np.linalg.norm(rotvec))
    if theta < _ROTATION_EPS:
        rotation = np.eye(3)
    else:
        kx, ky, kz = (np.asarray(rotvec) / theta).tolist()
        skew = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
        # Rodrigues' rotation formula
        rotation = np.eye(3) + np.sin(theta) * skew + (1.0 - np.cos(theta)) * (skew @ skew)

    matrix = np.eye(4)
    matrix[0:3, 0:3] = rotation
    matrix[0:3, 3] = pos
    return matrix


def to_robot_frame(position: list[float], frame: np.ndarray | None) -> list[float]:
    """Move a reported TCP position, relative to its chain's mount, into the robot's frame."""
    if frame is None:
        return position
    point = np.asarray(frame, dtype=float) @ np.array([*position[:3], 1.0])
    return [float(value) for value in point[:3]]
