"""Tests for ActionQueue interpolation, feedforward, and exhaustion behavior."""

from __future__ import annotations

import time

from policy.pidjogging.action_queue import ActionQueue


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


def test_pid_holds_position_after_chunk_exhausted():
    """Simulate the full PID loop: chunk executes, exhausts, waits 2s.

    The robot must converge to the last waypoint and stay there with
    zero velocity — no overshoot from stale feedforward.
    """
    from policy.pidjogging.velocity_controller import VelocityController

    q = ActionQueue()
    pid = VelocityController(
        velocity_limit=2.0, tolerance=0.01,
        p_gain=3.0, d_gain=0.15, ff_gain=1.0,
    )

    # Ramp from 0.0 to 0.4 over 4 steps at 50ms
    dt = 0.008  # 125Hz sim
    base_t = 100000.0

    import unittest.mock
    with unittest.mock.patch("time.monotonic", return_value=base_t):
        q.update([[0.1], [0.2], [0.3], [0.4]], dt_ms=50.0)

    pos = [0.0]
    max_overshoot = 0.0

    for tick in range(500):  # 4 seconds
        t = tick * dt
        fake_now = base_t + t

        # Monkey-patch time for deterministic sim
        import unittest.mock
        with unittest.mock.patch("time.monotonic", return_value=fake_now):
            target = q.get_target(lookahead_ms=50.0)
            ff = q.get_feedforward_velocity()

        if target is None:
            continue

        ff_scaled = [v * 1.0 for v in ff] if ff else None
        vel = pid.compute(pos, target, feedforward_velocity=ff_scaled, timestamp=fake_now)
        pos = [pos[0] + vel[0] * dt]

        # After the chunk is done (>200ms), check for overshoot past 0.4
        if t > 0.3:
            overshoot = pos[0] - 0.4
            if overshoot > 0:
                max_overshoot = max(max_overshoot, overshoot)

    # Robot should be at 0.4 (±tolerance), no overshoot
    assert abs(pos[0] - 0.4) < 0.02, f"Final position {pos[0]:.4f}, expected 0.4"
    assert max_overshoot < 0.01, f"Overshoot past target: {max_overshoot:.4f} rad"


def test_pid_smooth_transition_after_gap():
    """Chunk 1 executes and exhausts. 1s gap. Chunk 2 arrives.

    Verify: no velocity spike at chunk 2 start.
    """
    from policy.pidjogging.velocity_controller import VelocityController

    q = ActionQueue()
    pid = VelocityController(
        velocity_limit=2.0, tolerance=0.01,
        p_gain=3.0, d_gain=0.15, ff_gain=1.0,
    )

    dt = 0.008
    base_t = 200000.0
    pos = [0.0]
    velocities: list[float] = []

    # Phase 1: ramp to 0.3 (3 steps at 50ms = 100ms chunk)
    import unittest.mock
    with unittest.mock.patch("time.monotonic", return_value=base_t):
        q.update([[0.1], [0.2], [0.3]], dt_ms=50.0)

    for tick in range(250):  # 2 seconds (chunk + 1.8s gap)
        t = tick * dt
        fake_now = base_t + t
        with unittest.mock.patch("time.monotonic", return_value=fake_now):
            target = q.get_target(lookahead_ms=50.0)
            ff = q.get_feedforward_velocity()
        if target is None:
            continue
        ff_scaled = [v * 1.0 for v in ff] if ff else None
        vel = pid.compute(pos, target, feedforward_velocity=ff_scaled, timestamp=fake_now)
        pos = [pos[0] + vel[0] * dt]
        velocities.append(vel[0])

    # Robot should be settled at 0.3
    assert abs(pos[0] - 0.3) < 0.02
    # Last 50 velocities should be near zero (settled)
    assert all(abs(v) < 0.1 for v in velocities[-50:])

    # Phase 2: new chunk arrives — ramp from 0.3 to 0.6
    gap_end_t = base_t + 2.0
    with unittest.mock.patch("time.monotonic", return_value=gap_end_t):
        q.update([[0.3], [0.4], [0.5], [0.6]], dt_ms=50.0)

    max_vel = 0.0
    for tick in range(50):  # 400ms after new chunk
        t = 2.0 + tick * dt
        fake_now = base_t + t
        with unittest.mock.patch("time.monotonic", return_value=fake_now):
            target = q.get_target(lookahead_ms=50.0)
            ff = q.get_feedforward_velocity()
        if target is None:
            continue
        ff_scaled = [v * 1.0 for v in ff] if ff else None
        vel = pid.compute(pos, target, feedforward_velocity=ff_scaled, timestamp=fake_now)
        pos = [pos[0] + vel[0] * dt]
        max_vel = max(max_vel, abs(vel[0]))

    # Velocity should stay within limit — no spike
    assert max_vel <= 2.0, f"Velocity spike: {max_vel:.2f} rad/s"
    # Robot should be tracking toward 0.6
    assert pos[0] > 0.35, f"Robot didn't start moving: pos={pos[0]:.3f}"


# ---------------------------------------------------------------------------
# Update replaces cleanly
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
