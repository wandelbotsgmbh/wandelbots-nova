"""Tests for PID velocity controller tracking accuracy with single-step targets.

No chunks, no feedforward, no blending — just one target per tick.
Simulates the PID control loop at 125Hz with deterministic timestamps.
Verifies the controller handles all movement patterns without overshoot.
"""

from __future__ import annotations

import math

from policy.pidjogging.velocity_controller import VelocityController

# Simulation parameters
SIM_HZ = 125
SIM_DT = 1.0 / SIM_HZ
BASE_T = 500000.0  # fake monotonic base for deterministic tests


def _simulate(
    target_fn,
    *,
    duration: float = 4.0,
    warmup: float = 0.5,
    p_gain: float = 3.0,
    d_gain: float = 0.15,
    velocity_limit: float = 2.0,
) -> tuple[float, float, int]:
    """Run PID simulation and return (max_error_deg, mean_error_deg, overshoot_count).

    Args:
        target_fn: (t: float) -> float, returns the desired position at time t.
        duration: Total simulation time in seconds.
        warmup: Skip this many seconds at the start when computing errors.
    """
    pid = VelocityController(
        velocity_limit=velocity_limit,
        tolerance=0.01,
        p_gain=p_gain,
        d_gain=d_gain,
    )

    pos = [target_fn(0.0)]  # start at the initial target
    errors: list[float] = []
    prev_sign: int | None = None
    overshoots = 0

    for tick in range(int(duration / SIM_DT)):
        t = tick * SIM_DT
        now = BASE_T + t
        target = [target_fn(t)]

        vel = pid.compute(pos, target, timestamp=now)
        pos = [pos[0] + vel[0] * SIM_DT]

        if t > warmup:
            err = pos[0] - target[0]
            errors.append(abs(err))

            sign = 1 if err > 0.005 else (-1 if err < -0.005 else 0)
            if sign != 0 and prev_sign is not None and prev_sign not in (0, sign):
                overshoots += 1
            if sign != 0:
                prev_sign = sign

    max_err = math.degrees(max(errors)) if errors else 0
    mean_err = math.degrees(sum(errors) / len(errors)) if errors else 0
    return max_err, mean_err, overshoots


# ---------------------------------------------------------------------------
# Movement patterns (single-step, no chunks)
# ---------------------------------------------------------------------------


class TestSingleStepTracking:
    """PID tracking with one target per tick — no chunks, no feedforward."""

    def test_hold_position(self):
        """Constant target. Robot should be perfectly still after warmup."""
        _max, mean_err, overshoots = _simulate(lambda t: 0.5, duration=2.0)
        assert mean_err < 0.1, f"mean={mean_err:.3f}°"
        assert overshoots == 0

    def test_step_to_new_position(self):
        """Instant jump from 0 to 0.3 rad. Should converge without overshoot."""
        _max, mean_err, overshoots = _simulate(
            lambda t: 0.3 if t > 0.1 else 0.0,
            warmup=1.0,  # measure after settling
        )
        assert mean_err < 1.0, f"mean={mean_err:.3f}°"
        assert overshoots <= 1, f"overshoots={overshoots}"

    def test_linear_ramp(self):
        """Constant-velocity ramp at 0.1 rad/s. PID must track with small lag."""
        speed = 0.1
        _max, mean_err, _ = _simulate(lambda t: speed * t)
        # Steady-state error for P-only tracking a ramp = speed / p_gain
        expected_ss = math.degrees(speed / 3.0)
        assert mean_err < expected_ss * 2, f"mean={mean_err:.3f}° expected<{expected_ss*2:.3f}°"

    def test_sinusoidal(self):
        """Smooth sinusoidal oscillation. Tests phase lag and amplitude tracking."""
        amplitude, freq = 0.3, 0.5
        max_err, mean_err, _os = _simulate(
            lambda t: amplitude * math.sin(2 * math.pi * freq * t),
        )
        # Without feedforward, PID has phase lag. Mean error should still be bounded.
        assert mean_err < 8.0, f"mean={mean_err:.3f}°"
        assert max_err < 15.0, f"max={max_err:.3f}°"

    def test_triangle_wave(self):
        """Seesaw / triangle wave. Linear segments with sharp reversals."""
        amplitude, period = 0.3, 2.0

        def triangle(t: float) -> float:
            phase = (t % period) / period
            return amplitude * (4 * phase - 1) if phase < 0.5 else amplitude * (3 - 4 * phase)

        _max, mean_err, _os = _simulate(triangle)
        assert mean_err < 10.0, f"mean={mean_err:.3f}°"

    def test_square_wave_no_overshoot(self):
        """Step between two positions every 1s. Must not overshoot either target."""
        def square(t: float) -> float:
            return 0.2 if int(t) % 2 == 0 else -0.2

        _max, _mean, overshoots = _simulate(square, warmup=0.3, duration=6.0)
        assert overshoots <= 6, f"overshoots={overshoots}"

    def test_fast_sinusoidal(self):
        """Higher frequency sin (1Hz). Robot can't fully track — verify no instability."""
        amplitude, freq = 0.2, 1.0
        max_err, mean_err, _os = _simulate(
            lambda t: amplitude * math.sin(2 * math.pi * freq * t),
        )
        # Higher freq = larger tracking error, but should not diverge
        assert max_err < 25.0, f"max={max_err:.3f}° — PID may be unstable"
        assert mean_err < 15.0, f"mean={mean_err:.3f}°"

    def test_slow_sinusoidal_tracks_closely(self):
        """Very slow sin (0.1Hz). PID should track almost perfectly."""
        amplitude, freq = 0.3, 0.1
        _max, mean_err, overshoots = _simulate(
            lambda t: amplitude * math.sin(2 * math.pi * freq * t),
            duration=10.0,
        )
        assert mean_err < 3.0, f"mean={mean_err:.3f}°"
        assert overshoots <= 4, f"overshoots={overshoots}"

    def test_return_to_zero(self):
        """Move out, return to zero. Final position must be within tolerance."""
        def out_and_back(t: float) -> float:
            if t < 1.0:
                return 0.5 * t  # ramp to 0.5
            if t < 2.0:
                return 0.5  # hold
            if t < 3.0:
                return 0.5 * (3.0 - t)  # ramp back to 0
            return 0.0  # hold at zero

        _max, mean_err, _os = _simulate(out_and_back, warmup=0.2, duration=5.0)
        assert mean_err < 5.0, f"mean={mean_err:.3f}°"

    def test_velocity_limit_not_exceeded(self):
        """Large instant jump. Velocity must be clamped, no overshoot once settled."""
        pid = VelocityController(velocity_limit=2.0, p_gain=3.0, d_gain=0.15)
        pos = [0.0]

        for tick in range(500):
            now = BASE_T + tick * SIM_DT
            vel = pid.compute(pos, [1.0], timestamp=now)  # jump to 1.0 rad
            assert abs(vel[0]) <= 2.0 + 0.001, f"velocity {vel[0]} exceeds limit"
            pos = [pos[0] + vel[0] * SIM_DT]

        # Should have converged
        assert abs(pos[0] - 1.0) < 0.02, f"final pos={pos[0]:.4f}"

    def test_zero_target_produces_zero_velocity(self):
        """Robot at target. Velocity must be exactly zero (within tolerance)."""
        pid = VelocityController(velocity_limit=2.0, p_gain=3.0, d_gain=0.15, tolerance=0.01)
        vel = pid.compute([0.5], [0.5], timestamp=BASE_T)
        assert vel == [0.0]

    def test_multiple_joints_independent(self):
        """Each joint tracks its own target independently."""
        pid = VelocityController(velocity_limit=2.0, p_gain=3.0, d_gain=0.15)
        pos = [0.0, 0.0, 0.0]

        for tick in range(200):
            now = BASE_T + tick * SIM_DT
            t = tick * SIM_DT
            targets = [
                0.3 * math.sin(2 * math.pi * 0.5 * t),  # joint 0: slow sin
                0.1 * t,                                   # joint 1: ramp
                0.2,                                        # joint 2: constant
            ]
            vel = pid.compute(pos, targets, timestamp=now)
            pos = [pos[j] + vel[j] * SIM_DT for j in range(3)]

        # Joint 2 should be at 0.2 (constant target)
        assert abs(pos[2] - 0.2) < 0.02, f"joint 2 pos={pos[2]:.4f}"
        # Joint 1 should be tracking the ramp
        expected_j1 = 0.1 * 200 * SIM_DT
        assert abs(pos[1] - expected_j1) < 0.1, f"joint 1 pos={pos[1]:.4f}"
