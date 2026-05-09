"""Tests for pose_to_tcp rotation conversions."""

from __future__ import annotations

import math
from unittest.mock import MagicMock

import pytest

from policy.pose import TcpFormat, pose_to_tcp


def _pose(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> MagicMock:
    """Create a mock Nova Pose (position in mm, orientation as rotation vector)."""
    pos = MagicMock()
    pos.x, pos.y, pos.z = x, y, z
    ori = MagicMock()
    ori.x, ori.y, ori.z = rx, ry, rz
    pose = MagicMock()
    pose.position = pos
    pose.orientation = ori
    return pose


def test_rotation_vector_passthrough():
    result = pose_to_tcp(_pose(1000, 2000, 3000, 0.1, 0.2, 0.3), TcpFormat.ROTATION_VECTOR)
    assert len(result) == 6
    # Position converted from mm to meters
    assert result[0] == pytest.approx(1.0)
    assert result[1] == pytest.approx(2.0)
    assert result[2] == pytest.approx(3.0)
    # Orientation passed through
    assert result[3] == pytest.approx(0.1)
    assert result[4] == pytest.approx(0.2)
    assert result[5] == pytest.approx(0.3)


def test_quaternion_identity():
    """Zero rotation vector → identity quaternion (0,0,0,1)."""
    result = pose_to_tcp(_pose(0, 0, 0, 0, 0, 0), TcpFormat.QUATERNION)
    assert len(result) == 7
    assert result[3:] == pytest.approx([0.0, 0.0, 0.0, 1.0], abs=1e-10)


def test_quaternion_90deg_z():
    """90° rotation around Z → qz=sin(45°), qw=cos(45°)."""
    angle = math.pi / 2  # 90 degrees
    result = pose_to_tcp(_pose(0, 0, 0, 0, 0, angle), TcpFormat.QUATERNION)
    expected_s = math.sin(angle / 2)
    expected_c = math.cos(angle / 2)
    assert result[3] == pytest.approx(0.0, abs=1e-10)
    assert result[4] == pytest.approx(0.0, abs=1e-10)
    assert result[5] == pytest.approx(expected_s, rel=1e-6)
    assert result[6] == pytest.approx(expected_c, rel=1e-6)


def test_quaternion_unit_norm():
    """Any quaternion output should have unit norm."""
    result = pose_to_tcp(_pose(100, 200, 300, 0.7, -0.3, 1.2), TcpFormat.QUATERNION)
    qx, qy, qz, qw = result[3:]
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    assert norm == pytest.approx(1.0, rel=1e-8)


def test_rot6d_identity():
    """Zero rotation → first two columns of identity matrix."""
    result = pose_to_tcp(_pose(0, 0, 0, 0, 0, 0), TcpFormat.ROT6D)
    assert len(result) == 9
    # First column of identity
    assert result[3:6] == pytest.approx([1.0, 0.0, 0.0], abs=1e-10)
    # Second column of identity
    assert result[6:9] == pytest.approx([0.0, 1.0, 0.0], abs=1e-10)


def test_rot6d_90deg_z():
    """90° rotation around Z."""
    angle = math.pi / 2
    result = pose_to_tcp(_pose(0, 0, 0, 0, 0, angle), TcpFormat.ROT6D)
    # First column: [cos(90), sin(90), 0] = [0, 1, 0]
    assert result[3] == pytest.approx(0.0, abs=1e-10)
    assert result[4] == pytest.approx(1.0, abs=1e-10)
    assert result[5] == pytest.approx(0.0, abs=1e-10)
    # Second column: [-sin(90), cos(90), 0] = [-1, 0, 0]
    assert result[6] == pytest.approx(-1.0, abs=1e-10)
    assert result[7] == pytest.approx(0.0, abs=1e-10)
    assert result[8] == pytest.approx(0.0, abs=1e-10)


def test_rot6d_orthogonality():
    """Columns of rot6d output must be orthogonal and unit length."""
    result = pose_to_tcp(_pose(500, -200, 800, 1.0, -0.5, 0.8), TcpFormat.ROT6D)
    col1 = result[3:6]
    col2 = result[6:9]
    # Unit length
    norm1 = math.sqrt(sum(v**2 for v in col1))
    norm2 = math.sqrt(sum(v**2 for v in col2))
    assert norm1 == pytest.approx(1.0, rel=1e-8)
    assert norm2 == pytest.approx(1.0, rel=1e-8)
    # Orthogonal (dot product = 0)
    dot = sum(a * b for a, b in zip(col1, col2, strict=True))
    assert dot == pytest.approx(0.0, abs=1e-8)
