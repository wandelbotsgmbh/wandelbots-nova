"""Property-based tests for pose_to_eef rotation conversions.

``pose_to_eef`` is a pure function (its signature is its own driving port), so
these tests state its *invariants* over generated inputs rather than a handful
of pinned examples:

* position is always scaled mm -> m, for every output format;
* the rotation-vector format passes orientation through untouched;
* every quaternion is unit-norm and encodes the original rotation angle;
* every rot6d output is a pair of orthonormal columns.

A couple of exact-boundary cases (zero rotation) are kept as plain examples
because the contract there is a single pinned value, not a property.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

from hypothesis import assume, given, settings, strategies as st
import pytest

from policy.gr00t.eef import TcpFormat, pose_to_eef

# Strategies ---------------------------------------------------------------

_MM = st.floats(min_value=-1e4, max_value=1e4, allow_nan=False, allow_infinity=False)
_RV = st.floats(min_value=-math.pi, max_value=math.pi, allow_nan=False, allow_infinity=False)
_FORMATS = st.sampled_from(list(TcpFormat))


def _pose(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> object:
    """A Nova-Pose-shaped object: position in mm, orientation as a rotation vector."""
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=rx, y=ry, z=rz),
    )


# Position scaling — holds for every format -------------------------------


@given(x=_MM, y=_MM, z=_MM, rx=_RV, ry=_RV, rz=_RV, fmt=_FORMATS)
@settings(max_examples=200, deadline=None)
def test_position_is_always_scaled_from_mm_to_meters(x, y, z, rx, ry, rz, fmt):
    """The first three outputs are the position in metres, whatever the format."""
    result = pose_to_eef(_pose(x, y, z, rx, ry, rz), fmt)
    assert result[0] == pytest.approx(x * 0.001)
    assert result[1] == pytest.approx(y * 0.001)
    assert result[2] == pytest.approx(z * 0.001)


@given(x=_MM, y=_MM, z=_MM)
@settings(max_examples=50, deadline=None)
def test_position_scale_of_one_keeps_millimetres(x, y, z):
    """position_scale=1.0 is the documented escape hatch to keep mm."""
    result = pose_to_eef(_pose(x, y, z, 0, 0, 0), TcpFormat.ROTATION_VECTOR, position_scale=1.0)
    assert result[:3] == pytest.approx([x, y, z])


# rotation_vector — orientation passes through untouched ------------------


@given(x=_MM, y=_MM, z=_MM, rx=_RV, ry=_RV, rz=_RV)
@settings(max_examples=200, deadline=None)
def test_rotation_vector_passes_orientation_through_unchanged(x, y, z, rx, ry, rz):
    """ROTATION_VECTOR returns six values and never touches the orientation."""
    result = pose_to_eef(_pose(x, y, z, rx, ry, rz), TcpFormat.ROTATION_VECTOR)
    assert len(result) == 6
    assert result[3:] == pytest.approx([rx, ry, rz])


# quaternion — unit norm + recovers the rotation it was built from --------


@given(rx=_RV, ry=_RV, rz=_RV)
@settings(max_examples=300, deadline=None)
def test_quaternion_is_always_unit_norm(rx, ry, rz):
    """A quaternion is seven values whose orientation part has unit length."""
    result = pose_to_eef(_pose(0, 0, 0, rx, ry, rz), TcpFormat.QUATERNION)
    assert len(result) == 7
    qx, qy, qz, qw = result[3:]
    assert math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) == pytest.approx(1.0, rel=1e-9)


@given(rx=_RV, ry=_RV, rz=_RV)
@settings(max_examples=300, deadline=None)
def test_quaternion_recovers_the_original_rotation_angle(rx, ry, rz):
    """The quaternion encodes a rotation by |rotation-vector| about its axis.

    For a unit quaternion (v, w) the rotation angle is 2*atan2(|v|, w); since the
    input rotation-vector magnitude *is* that angle, the two must agree. Below the
    function's small-angle cutoff the rotation is treated as identity, so the
    property is stated for non-degenerate rotations only (the zero case is pinned
    in its own example below).
    """
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    assume(angle >= 1e-6)  # above the _ANGLE_EPSILON identity cutoff
    qx, qy, qz, qw = pose_to_eef(_pose(0, 0, 0, rx, ry, rz), TcpFormat.QUATERNION)[3:]
    recovered = 2.0 * math.atan2(math.sqrt(qx * qx + qy * qy + qz * qz), qw)
    assert recovered == pytest.approx(angle, abs=1e-7)


# rot6d — the two columns are always orthonormal --------------------------


@given(rx=_RV, ry=_RV, rz=_RV)
@settings(max_examples=300, deadline=None)
def test_rot6d_columns_are_always_orthonormal(rx, ry, rz):
    """rot6d returns nine values: two unit-length, mutually orthogonal columns."""
    result = pose_to_eef(_pose(0, 0, 0, rx, ry, rz), TcpFormat.ROT6D)
    assert len(result) == 9
    col1, col2 = result[3:6], result[6:9]
    assert math.sqrt(sum(v * v for v in col1)) == pytest.approx(1.0, rel=1e-9)
    assert math.sqrt(sum(v * v for v in col2)) == pytest.approx(1.0, rel=1e-9)
    dot = sum(a * b for a, b in zip(col1, col2, strict=True))
    assert dot == pytest.approx(0.0, abs=1e-9)


# Exact boundaries — zero rotation is a pinned value, not a property ------
# bypass: the contract at angle==0 is a single exact vector, so an example
# states it more clearly than a property would.


def test_zero_rotation_is_the_identity_quaternion():
    result = pose_to_eef(_pose(0, 0, 0, 0, 0, 0), TcpFormat.QUATERNION)
    assert result[3:] == pytest.approx([0.0, 0.0, 0.0, 1.0], abs=1e-12)


def test_zero_rotation_is_the_identity_rot6d_columns():
    result = pose_to_eef(_pose(0, 0, 0, 0, 0, 0), TcpFormat.ROT6D)
    assert result[3:6] == pytest.approx([1.0, 0.0, 0.0], abs=1e-12)  # first identity column
    assert result[6:9] == pytest.approx([0.0, 1.0, 0.0], abs=1e-12)  # second identity column


def test_an_unknown_format_is_rejected():
    with pytest.raises(ValueError, match="TcpFormat"):
        pose_to_eef(_pose(0, 0, 0, 0, 0, 0), "euler_xyz")
