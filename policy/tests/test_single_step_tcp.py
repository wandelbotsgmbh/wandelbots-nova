"""Tests for TCP (Cartesian) PID tracking with single-step targets.

Simulates PID control of a 6-DOF Cartesian space (x,y,z,rx,ry,rz)
using the same VelocityController as joint mode. Verifies tracking
across different movement patterns without real hardware.
"""

from __future__ import annotations

import math

from policy.pidjogging.velocity_controller import VelocityController

SIM_HZ = 125
SIM_DT = 1.0 / SIM_HZ
BASE_T = 600000.0

# TCP velocity limits: [tx, ty, tz, rx, ry, rz]
TCP_VEL_LIMITS = [500.0, 500.0, 500.0, 1.5, 1.5, 1.5]

# Home TCP pose (mm, rad): x, y, z, rx, ry, rz
HOME_TCP = [400.0, 0.0, 500.0, 0.0, 3.14159, 0.0]


def _simulate_tcp(
    target_fn,
    *,
    duration: float = 4.0,
    warmup: float = 0.5,
    p_gain: float = 3.0,
    d_gain: float = 0.15,
) -> tuple[float, float]:
    """Run TCP PID simulation. Returns (max_error_mm, mean_error_mm).

    Only measures position error (first 3 axes), not orientation.
    """
    pid = VelocityController(
        velocity_limit=TCP_VEL_LIMITS,
        tolerance=0.5,  # 0.5mm tolerance for TCP
        p_gain=p_gain,
        d_gain=d_gain,
    )

    pos = list(HOME_TCP)
    errors: list[float] = []

    for tick in range(int(duration / SIM_DT)):
        t = tick * SIM_DT
        now = BASE_T + t
        target = target_fn(t)

        vel = pid.compute(pos, target, timestamp=now)
        pos = [pos[j] + vel[j] * SIM_DT for j in range(6)]

        if t > warmup:
            err = math.sqrt(sum((pos[j] - target[j]) ** 2 for j in range(3)))
            errors.append(err)

    max_err = max(errors) if errors else 0
    mean_err = sum(errors) / len(errors) if errors else 0
    return max_err, mean_err


class TestTcpSingleStep:
    """TCP PID tracking with single-step targets in Cartesian space."""

    def test_hold_position(self):
        """Static target. Error should be zero after warmup."""
        _max, mean_err = _simulate_tcp(lambda _t: list(HOME_TCP), duration=2.0)
        assert mean_err < 0.1, f"mean={mean_err:.3f}mm"

    def test_linear_x(self):
        """Constant velocity along X (20 mm/s)."""
        speed = 20.0

        def target(t: float) -> list[float]:
            p = list(HOME_TCP)
            p[0] = HOME_TCP[0] + speed * t
            return p

        _max, mean_err = _simulate_tcp(target)
        # Steady-state lag = speed / p_gain
        expected_ss = speed / 3.0
        assert mean_err < expected_ss * 2, f"mean={mean_err:.3f}mm"

    def test_circle_xy(self):
        """Circular motion in XY plane."""
        radius, freq = 20.0, 0.3

        def target(t: float) -> list[float]:
            angle = 2 * math.pi * freq * t
            p = list(HOME_TCP)
            p[0] = HOME_TCP[0] + radius * math.cos(angle)
            p[1] = HOME_TCP[1] + radius * math.sin(angle)
            return p

        _max, mean_err = _simulate_tcp(target)
        assert mean_err < 12.0, f"mean={mean_err:.3f}mm"  # circle has phase lag

    def test_sinusoidal_z(self):
        """Sinusoidal oscillation along Z."""
        amplitude, freq = 15.0, 0.5

        def target(t: float) -> list[float]:
            p = list(HOME_TCP)
            p[2] = HOME_TCP[2] + amplitude * math.sin(2 * math.pi * freq * t)
            return p

        _max, mean_err = _simulate_tcp(target)
        assert mean_err < 12.0, f"mean={mean_err:.3f}mm"

    def test_step_to_new_position(self):
        """Instant 30mm jump along X. Should converge."""

        def target(t: float) -> list[float]:
            p = list(HOME_TCP)
            if t > 0.1:
                p[0] += 30.0
            return p

        _max, mean_err = _simulate_tcp(target, warmup=1.5)
        assert mean_err < 1.0, f"mean={mean_err:.3f}mm"

    def test_figure_eight(self):
        """Lissajous curve in XY."""
        size, freq = 15.0, 0.25

        def target(t: float) -> list[float]:
            angle = 2 * math.pi * freq * t
            p = list(HOME_TCP)
            p[0] = HOME_TCP[0] + size * math.sin(angle)
            p[1] = HOME_TCP[1] + size * math.sin(2 * angle)
            return p

        _max, mean_err = _simulate_tcp(target)
        assert mean_err < 10.0, f"mean={mean_err:.3f}mm"

    def test_diagonal_ramp(self):
        """Simultaneous motion in XYZ."""
        speed = 15.0

        def target(t: float) -> list[float]:
            p = list(HOME_TCP)
            p[0] = HOME_TCP[0] + speed * t
            p[1] = HOME_TCP[1] + speed * t * 0.7
            p[2] = HOME_TCP[2] + speed * t * 0.3
            return p

        _max, mean_err = _simulate_tcp(target)
        expected_ss = speed * math.sqrt(1 + 0.7**2 + 0.3**2) / 3.0
        assert mean_err < expected_ss * 2, f"mean={mean_err:.3f}mm"

    def test_velocity_limit_clamped(self):
        """Large jump. Velocity must stay within limits."""
        pid = VelocityController(
            velocity_limit=TCP_VEL_LIMITS,
            tolerance=0.5,
            p_gain=3.0,
            d_gain=0.15,
        )
        pos = list(HOME_TCP)
        target = list(HOME_TCP)
        target[0] += 200.0  # 200mm jump

        for tick in range(500):
            now = BASE_T + tick * SIM_DT
            vel = pid.compute(pos, target, timestamp=now)
            for j in range(6):
                assert abs(vel[j]) <= TCP_VEL_LIMITS[j] + 0.001, (
                    f"axis {j}: vel={vel[j]} exceeds limit {TCP_VEL_LIMITS[j]}"
                )
            pos = [pos[j] + vel[j] * SIM_DT for j in range(6)]

        assert abs(pos[0] - target[0]) < 1.0, f"didn't converge: pos[0]={pos[0]}"

    def test_orientation_tracking(self):
        """Rotate around Z axis. Orientation should track."""
        pid = VelocityController(
            velocity_limit=TCP_VEL_LIMITS,
            tolerance=0.5,
            p_gain=3.0,
            d_gain=0.15,
        )
        pos = list(HOME_TCP)
        errors: list[float] = []

        for tick in range(int(4.0 / SIM_DT)):
            t = tick * SIM_DT
            now = BASE_T + t
            target = list(HOME_TCP)
            # Rotate rz slowly
            target[5] = HOME_TCP[5] + 0.3 * math.sin(2 * math.pi * 0.2 * t)

            vel = pid.compute(pos, target, timestamp=now)
            pos = [pos[j] + vel[j] * SIM_DT for j in range(6)]

            if t > 0.5:
                errors.append(abs(pos[5] - target[5]))

        mean_err = sum(errors) / len(errors)
        assert mean_err < 0.25, f"orientation mean error={mean_err:.4f} rad"

    def test_slow_motion_tracks_closely(self):
        """Very slow circle (0.1Hz). TCP should track within 2mm."""
        radius, freq = 10.0, 0.1

        def target(t: float) -> list[float]:
            angle = 2 * math.pi * freq * t
            p = list(HOME_TCP)
            p[0] = HOME_TCP[0] + radius * math.cos(angle)
            p[1] = HOME_TCP[1] + radius * math.sin(angle)
            return p

        _max, mean_err = _simulate_tcp(target, duration=10.0)
        assert mean_err < 3.0, f"mean={mean_err:.3f}mm"
