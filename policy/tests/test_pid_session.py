"""Tests for PidJoggingSession velocity computation and target advancement."""

from __future__ import annotations

import time
from unittest.mock import MagicMock

from nova.types import Pose
from policy.pid_jogging_session import PidJoggingSession
from policy.types import GuardStopError, PolicyRunnerConfig


def _session(
    *,
    guards: list | None = None,
    velocity_limit: float = 1.5,
    tolerance: float = 0.01,
) -> PidJoggingSession:
    mg = MagicMock()
    mg.id = "0@ur10e"
    mg._controller_id = "ur10e"
    mg._cell = "cell"
    mg._api_client = MagicMock()
    config = PolicyRunnerConfig(velocity_limit=velocity_limit, tolerance=tolerance)
    session = PidJoggingSession(mg, config, safety_guards=guards)
    session._num_joints = 6
    session._current_joints = [0.0] * 6
    session._current_tcp_pose = Pose(0, 0, 500, 0, 0, 0)
    session._current_tcp_name = "Flange"
    return session


def test_no_target_returns_zero():
    s = _session()
    vel = s._compute_velocity_with_safety()
    assert vel == [0.0] * 6


def test_target_drives_velocity():
    s = _session()
    s.update_chunk([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dt_ms=0.0)
    vel = s._compute_velocity_with_safety()
    # Should drive joint 0 positively toward 1.0
    assert vel[0] > 0
    # Other joints at target → zero
    assert all(v == 0.0 for v in vel[1:])


def test_multistep_chunk_advances_with_time():
    s = _session()
    step_0 = [0.1] * 6
    step_1 = [0.2] * 6
    step_2 = [0.3] * 6
    s.update_chunk([step_0, step_1, step_2], dt_ms=100.0)

    # Initially at step 0
    target = s._get_active_target()
    assert target == step_0

    # Simulate 150ms elapsed → should be at step 1
    s._step_start_time = time.monotonic() - 0.15
    target = s._get_active_target()
    assert target == step_1

    # Simulate 250ms elapsed → should be at step 2 (clamped)
    s._step_start_time = time.monotonic() - 0.25
    target = s._get_active_target()
    assert target == step_2


def test_single_step_chunk_stays():
    s = _session()
    s.update_chunk([[0.5] * 6], dt_ms=33.0)
    # Even with time elapsed, single step stays at index 0
    s._step_start_time = time.monotonic() - 10.0
    target = s._get_active_target()
    assert target == [0.5] * 6


def test_safety_guard_raises():
    """A guard returning False triggers GuardStopError."""
    def always_stop(ctx):
        return False

    always_stop.__name__ = "always_stop"

    s = _session(guards=[always_stop])
    s.update_chunk([[1.0] * 6], dt_ms=0.0)

    try:
        s._compute_velocity_with_safety()
        assert False, "Should have raised"  # noqa: B011
    except GuardStopError as e:
        assert e.guard_name == "always_stop"
        assert not s._running


def test_guard_receives_io_values():
    """Guards see IO values passed to session."""
    received = {}

    def check_io(ctx):
        received.update(ctx.io_values or {})
        return True

    s = _session(guards=[check_io])
    s._io_values = {"digital_out[0]": True, "analog_in[1]": 3.14}
    s.update_chunk([[0.5] * 6], dt_ms=0.0)
    s._compute_velocity_with_safety()

    assert received == {"digital_out[0]": True, "analog_in[1]": 3.14}


def test_update_chunk_resets_index():
    s = _session()
    s.update_chunk([[0.1] * 6, [0.2] * 6], dt_ms=100.0)
    s._step_index = 1  # Simulate advancement

    # New chunk resets
    s.update_chunk([[0.9] * 6], dt_ms=0.0)
    assert s._step_index == 0
    assert s._get_active_target() == [0.9] * 6
