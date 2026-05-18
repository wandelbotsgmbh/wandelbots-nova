"""Tests for ActionQueue interpolation, feedforward, and exhaustion behavior."""

from __future__ import annotations

import time

from policy.jogging.action_queue import ActionQueue


def _queue_with_ramp(n: int = 4, dt_ms: float = 100.0) -> ActionQueue:
    """Create a queue with a simple ramp: step i = [0.1 * (i+1)] per joint."""
    q = ActionQueue()
    steps = [[0.1 * (i + 1)] for i in range(n)]  # [[0.1], [0.2], [0.3], [0.4]]
    q.update(steps, dt_ms)
    return q


# ---------------------------------------------------------------------------
# Interpolation
# ---------------------------------------------------------------------------


def test_interpolation_at_start():
    q = _queue_with_ramp()
    target = q.get_target()
    assert target is not None
    assert abs(target[0] - 0.1) < 0.01  # step 0


def test_interpolation_midway():
    q = _queue_with_ramp(dt_ms=100.0)
    # Simulate 150ms elapsed → halfway between step 1 and step 2
    q._start_time = time.monotonic() - 0.15
    target = q.get_target()
    assert target is not None
    assert abs(target[0] - 0.25) < 0.01  # lerp(0.2, 0.3, 0.5)


def test_interpolation_clamps_at_end():
    q = _queue_with_ramp(dt_ms=100.0)
    # Simulate 500ms elapsed → past end (only 4 steps = 300ms)
    q._start_time = time.monotonic() - 0.5
    target = q.get_target()
    assert target is not None
    assert abs(target[0] - 0.4) < 0.001  # last step


# ---------------------------------------------------------------------------
# Feedforward
# ---------------------------------------------------------------------------


def test_feedforward_mid_chunk():
    q = _queue_with_ramp(dt_ms=100.0)
    # Move to index 1 (middle of chunk)
    q._start_time = time.monotonic() - 0.1
    q.get_target()  # updates _index
    ff = q.get_feedforward_velocity()
    assert ff is not None
    # Central difference around index 1: (step2 - step0) / (2 * dt)
    expected = (0.3 - 0.1) / (2 * 0.1)  # 1.0 rad/s
    assert abs(ff[0] - expected) < 0.01


def test_feedforward_none_for_single_step():
    q = ActionQueue()
    q.update([[1.0, 2.0]], dt_ms=0.0)
    assert q.get_feedforward_velocity() is None


def test_feedforward_none_when_empty():
    q = ActionQueue()
    assert q.get_feedforward_velocity() is None


# ---------------------------------------------------------------------------
# Exhaustion: chunk finished, waiting for next chunk
# ---------------------------------------------------------------------------


def test_feedforward_zero_when_chunk_exhausted():
    """After the chunk is fully consumed, feedforward must return None (hold position)."""
    q = _queue_with_ramp(n=4, dt_ms=100.0)

    # Simulate enough time for the chunk to be exhausted (400ms for 4 steps at 100ms)
    q._start_time = time.monotonic() - 0.5

    # get_target should clamp to last step
    target = q.get_target()
    assert target is not None
    assert abs(target[0] - 0.4) < 0.001

    # feedforward must be None — robot should hold, not drift
    ff = q.get_feedforward_velocity()
    assert ff is None


def test_target_holds_after_exhaustion():
    """After exhaustion, repeated get_target calls return the same last position."""
    q = _queue_with_ramp(n=4, dt_ms=100.0)
    q._start_time = time.monotonic() - 1.0  # well past end

    t1 = q.get_target()
    t2 = q.get_target()
    assert t1 == t2 == [0.4]


def test_feedforward_resumes_after_new_chunk():
    """After exhaustion, a new chunk should restore feedforward."""
    q = _queue_with_ramp(n=4, dt_ms=100.0)
    q._start_time = time.monotonic() - 1.0  # exhaust it
    q.get_target()
    assert q.get_feedforward_velocity() is None

    # Load a new chunk
    q.update([[0.0], [0.5], [1.0], [1.5]], dt_ms=100.0)
    q._start_time = time.monotonic() - 0.1  # move to index 1
    q.get_target()  # updates _index

    ff = q.get_feedforward_velocity()
    assert ff is not None
    assert abs(ff[0]) > 0.1  # non-zero feedforward


def test_hold_position_chunk_no_overshoot():
    """A chunk where all steps are identical should produce zero feedforward always.

    This simulates the 'hold position' case where the policy outputs the same
    target for every step. The robot should stay perfectly still.
    """
    q = ActionQueue()
    static_pos = [1.5, -0.3, 0.0]
    q.update([list(static_pos) for _ in range(16)], dt_ms=50.0)

    # Check at multiple time points
    for elapsed_ms in [0, 25, 100, 400, 1000]:
        q._start_time = time.monotonic() - elapsed_ms / 1000.0
        target = q.get_target()
        assert target is not None
        for j in range(3):
            assert abs(target[j] - static_pos[j]) < 0.001

        ff = q.get_feedforward_velocity()
        if ff is not None:
            # Even mid-chunk, identical steps → zero velocity
            for v in ff:
                assert abs(v) < 0.001


# ---------------------------------------------------------------------------
# Lookahead
# ---------------------------------------------------------------------------


def test_lookahead_shifts_target_forward():
    q = _queue_with_ramp(n=4, dt_ms=100.0)
    t_no_la = q.get_target(lookahead_ms=0.0)
    t_with_la = q.get_target(lookahead_ms=50.0)
    assert t_no_la is not None and t_with_la is not None
    # With 50ms lookahead at t=0: should be halfway between step 0 and step 1
    assert t_with_la[0] > t_no_la[0]
    assert abs(t_with_la[0] - 0.15) < 0.01


# ---------------------------------------------------------------------------
# Full PID loop through chunk gap: no overshoot
# ---------------------------------------------------------------------------


def test_update_clears_exhaustion():
    """A new chunk after exhaustion resets state and starts tracking."""
    q = _queue_with_ramp(n=2, dt_ms=50.0)
    q._start_time = time.monotonic() - 1.0  # exhaust
    q.get_target()
    assert q._exhausted is True

    q.update([[0.0], [1.0]], dt_ms=100.0)
    assert q._exhausted is False
    target = q.get_target()
    assert target is not None
    # With blending, first step is a mix of old (0.2) and new (0.0)
    # Just verify it's tracking and not exhausted
    assert target[0] < 0.5


# ---------------------------------------------------------------------------
# Observation-time alignment: slow inference with direction changes
# ---------------------------------------------------------------------------


def test_observation_time_skips_stale_steps():
    """With observation_time in the past, get_target skips consumed steps.

    Simulates: observation captured 300ms ago, inference took 300ms,
    chunk of 16 steps at 33ms arrives. step[0] is 300ms stale.
    Without observation_time, target would snap back to step[0].
    With it, target starts ~9 steps in.
    """
    import unittest.mock

    q = ActionQueue()
    now = 500000.0
    obs_time = now - 0.3  # observation was 300ms ago

    steps = [[0.1 * (i + 1)] for i in range(16)]  # ramp 0.1 → 1.6

    with unittest.mock.patch("time.monotonic", return_value=now):
        q.update(steps, dt_ms=33.0, observation_time=obs_time)
        target = q.get_target()

    assert target is not None
    # 300ms / 33ms ≈ 9 steps in → target should be around step[9] = 1.0
    assert target[0] > 0.8, f"Target {target[0]:.2f} too low — didn't skip stale steps"
    assert target[0] < 1.2, f"Target {target[0]:.2f} too high"


def test_observation_time_none_starts_at_step0():
    """Without observation_time, update starts from step[0] as before."""
    import unittest.mock

    q = ActionQueue()
    now = 500000.0

    steps = [[0.1 * (i + 1)] for i in range(16)]

    with unittest.mock.patch("time.monotonic", return_value=now):
        q.update(steps, dt_ms=33.0)  # no observation_time
        target = q.get_target()

    assert target is not None
    assert abs(target[0] - 0.1) < 0.02, f"Target {target[0]:.2f} should be near step[0]"


