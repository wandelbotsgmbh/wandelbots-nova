"""Tests for PidJoggingSession velocity computation and target advancement."""

from __future__ import annotations

import time
from unittest.mock import MagicMock

import pytest

from nova.types import Pose
from policy.pidjogging import PidJoggingSession
from policy.types import GuardStopError, PidConfig


def _session(
    *,
    guards: list | None = None,
    velocity_limit: float = 1.5,
    tolerance: float = 0.01,
    lookahead_ms: float = 0.0,
) -> PidJoggingSession:
    mg = MagicMock()
    mg.id = "0@ur10e"
    mg._controller_id = "ur10e"
    mg._cell = "cell"
    mg._api_client = MagicMock()
    config = PidConfig(velocity_limit=velocity_limit, tolerance=tolerance, lookahead_ms=lookahead_ms)
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


def test_current_state_includes_optional_joint_signals():
    s = _session()
    s._current_joint_torques = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    s._current_joint_currents = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]

    state = s.current_state

    assert state is not None
    assert state.joint_torques == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert state.joint_currents == (1.0, 1.1, 1.2, 1.3, 1.4, 1.5)


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

    # At t=0: spline starts at step 0 value (0.1)
    target = s._get_active_target()
    assert all(abs(t - 0.1) < 0.01 for t in target)

    # Simulate 150ms elapsed -> interpolated between step 1 and step 2
    s._queue._start_time = time.monotonic() - 0.15
    target = s._get_active_target()
    assert target is not None
    # With 0 lookahead: 150ms at 100ms spacing = midway step 1-2 \u2192 0.25
    assert all(abs(t - 0.25) < 0.02 for t in target)

    # Simulate 250ms elapsed -> past end -> clamped at step 2 (0.3)
    s._queue._start_time = time.monotonic() - 0.25
    target = s._get_active_target()
    assert all(abs(t - 0.3) < 0.001 for t in target)


def test_single_step_chunk_stays():
    s = _session()
    s.update_chunk([[0.5] * 6], dt_ms=33.0)
    # Single step -> holds position
    s._queue._start_time = time.monotonic() - 10.0
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


@pytest.mark.asyncio
async def test_callback_policy_through_schema_to_pid():
    """End-to-end: callback policy adds 1.0 to each joint → schema parses → PID drives toward target."""
    from policy.schema import Observation, PolicySchema

    mg = MagicMock()
    mg.id = "0@ur10e"
    mg._controller_id = "ur10e"
    mg._cell = "cell"

    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
    ])

    # Policy: increase every joint by 1.0
    async def add_one_policy(obs, _schema, images=None, io_values=None):
        return {k: v + 1.0 for k, v in obs.items()}

    # Simulate current robot state
    current_joints = [0.1, -1.5, 0.0, 0.5, -0.3, 1.2]
    state = MagicMock()
    state.joints = tuple(current_joints)
    state.pose = None
    state.tcp = None
    state.joint_torques = None
    state.joint_currents = None

    # Build observation via schema
    obs = await schema.build_observation({"0@ur10e": state})
    assert obs == {
        "joints_1": 0.1, "joints_2": -1.5, "joints_3": 0.0,
        "joints_4": 0.5, "joints_5": -0.3, "joints_6": 1.2,
    }

    # Run through policy callback
    action_dict = await add_one_policy(obs, schema)
    expected_targets = [j + 1.0 for j in current_joints]
    for i, expected in enumerate(expected_targets, 1):
        assert action_dict[f"joints_{i}"] == pytest.approx(expected)

    # Parse action via schema → ActionChunk-style joints
    joints, _tcp, ios = await schema.parse_action(action_dict)
    assert "0@ur10e" in joints
    assert joints["0@ur10e"] == [expected_targets]
    assert ios is None

    # Feed into PID session
    s = _session()
    s._current_joints = list(current_joints)
    s.update_chunk(joints["0@ur10e"], dt_ms=0.0)

    vel = s._compute_velocity_with_safety()

    # Every joint should drive positively (target = current + 1.0)
    for i, v in enumerate(vel):
        assert v > 0, f"Joint {i} velocity should be positive, got {v}"
