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
