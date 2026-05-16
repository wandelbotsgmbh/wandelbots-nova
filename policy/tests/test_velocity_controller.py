"""Tests for the PID velocity controller."""

from __future__ import annotations

import pytest

from policy.pidjogging.velocity_controller import VelocityController


def test_on_target_returns_zero() -> None:
    vc = VelocityController()
    assert vc.compute([1.0, 2.0], [1.0, 2.0]) == [0.0, 0.0]


def test_within_tolerance_returns_zero() -> None:
    vc = VelocityController(tolerance=0.01)
    assert vc.compute([1.0], [1.005]) == [0.0]


def test_within_tolerance_still_applies_feedforward() -> None:
    """When within tolerance but feedforward is non-zero, don't return zero.

    This is the critical case for small action chunks: the interpolated target
    may be within tolerance of current position, but the chunk still needs to
    drive motion via feedforward velocity.
    """
    vc = VelocityController(tolerance=0.01, ff_gain=1.0)
    vel = vc.compute([1.0], [1.005], feedforward_velocity=[0.5])
    assert vel[0] > 0.0, "Feedforward must drive motion even within tolerance"


def test_error_drives_toward_target() -> None:
    vc = VelocityController(p_gain=2.0, d_gain=0.0)
    assert vc.compute([0.0], [1.0])[0] > 0
    assert vc.compute([1.0], [0.0])[0] < 0


def test_clamps_to_velocity_limit() -> None:
    vc = VelocityController(p_gain=100.0, velocity_limit=1.5)
    assert vc.compute([0.0], [10.0]) == [1.5]
    assert vc.compute([10.0], [0.0]) == [-1.5]


def test_mismatched_lengths_raises() -> None:
    vc = VelocityController()
    with pytest.raises(ValueError, match="mismatch"):
        vc.compute([0.0, 0.0], [1.0])
