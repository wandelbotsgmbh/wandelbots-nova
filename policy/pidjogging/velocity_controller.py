"""PID velocity controller for joint position tracking.

Pure computation — no I/O or async. Converts position error into
clamped velocity commands using proportional-integral-derivative control.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time

# Minimum time delta to prevent division-by-zero in derivative calculation.
_MIN_DT: float = 0.001
_FF_ZERO_THRESHOLD: float = 1e-6


@dataclass
class VelocityController:
    """PID velocity controller for one motion group.

    Computes joint velocities that drive current positions toward target positions.
    Maintains per-joint integral/derivative state and resets automatically when
    the target changes or positions are within tolerance.
    """

    velocity_limit: float | list[float] = 1.5
    tolerance: float = 0.01
    p_gain: float = 3.0
    i_gain: float = 0.0
    d_gain: float = 0.1
    ff_gain: float = 0.0
    integral_limit: float = 2.0

    # Internal PID state
    _prev_joints: list[float] | None = field(default=None, init=False, repr=False)
    _prev_target: list[float] | None = field(default=None, init=False, repr=False)
    _prev_time: float | None = field(default=None, init=False, repr=False)
    _integral: list[float] | None = field(default=None, init=False, repr=False)

    def _limit(self, index: int) -> float:
        """Get velocity limit for a given axis."""
        if isinstance(self.velocity_limit, list):
            return self.velocity_limit[index]
        return self.velocity_limit

    def reset(self) -> None:
        """Reset all internal PID state."""
        self._prev_joints = None
        self._prev_target = None
        self._prev_time = None
        self._integral = None

    def compute(
        self,
        current: list[float],
        target: list[float],
        *,
        feedforward_velocity: list[float] | None = None,
        timestamp: float | None = None,
    ) -> list[float]:
        """Compute joint velocities to move from current toward target.

        Args:
            current: Current joint positions (radians), length N.
            target: Target joint positions (radians), length N.
            feedforward_velocity: Optional desired velocity at the current target
                (rad/s per joint). When provided (e.g. from a spline derivative),
                added directly to the PID output.
            timestamp: Monotonic timestamp in seconds. If None, uses
                ``time.monotonic()``. Inject for deterministic testing.

        Returns:
            Joint velocities (rad/s), length N, clamped to [-velocity_limit, velocity_limit].

        Raises:
            ValueError: If current and target have different lengths.
        """
        if len(current) != len(target):
            msg = f"Joint count mismatch: current={len(current)}, target={len(target)}"
            raise ValueError(msg)

        n = len(current)
        now = timestamp if timestamp is not None else time.monotonic()

        # If all joints within tolerance, output zero and reset
        within_tolerance = all(
            abs(c - t) <= self.tolerance for c, t in zip(current, target, strict=True)
        )

        # Feedforward: use provided velocity or fall back to historical estimate
        if feedforward_velocity is not None:
            ff = feedforward_velocity
        else:
            ff = self._feedforward(target, now, n)

        # If within tolerance AND no feedforward driving motion, output zero
        if within_tolerance and all(abs(v) < _FF_ZERO_THRESHOLD for v in ff):
            self.reset()
            return [0.0] * n

        # Detect target change → reset derivative/integral state
        if self._target_changed(target):
            self._prev_joints = None
            self._integral = None

        # Initialize integral if needed
        if self._integral is None:
            self._integral = [0.0] * n

        # Compute derivative (velocity estimate from joint position history)
        deriv, dt = self._derivative(current, now, n)

        # Accumulate integral with anti-windup
        if dt > _MIN_DT:
            for i in range(n):
                self._integral[i] += (target[i] - current[i]) * dt
                self._integral[i] = max(
                    -self.integral_limit, min(self.integral_limit, self._integral[i])
                )

        # Update state
        self._prev_joints = list(current)
        self._prev_target = list(target)
        self._prev_time = now

        # Compute PID output per joint
        velocities: list[float] = []
        for i in range(n):
            error = target[i] - current[i]
            # D term acts on error derivative (target_vel - current_vel), not raw velocity.
            # This prevents D from opposing normal feedforward-driven motion.
            # When ff is active, target is moving → d(error)/dt ≈ 0 during tracking.
            # When overshooting, current moves faster than target → D opposes.
            d_term = self.d_gain * deriv[i] if ff[i] == 0.0 else self.d_gain * (deriv[i] - ff[i])
            vel = (
                self.p_gain * error
                + self.i_gain * self._integral[i]
                - d_term
                + ff[i]
            )
            vel = max(-self._limit(i), min(self._limit(i), vel))
            velocities.append(vel)

        return velocities

    def _feedforward(self, target: list[float], now: float, n: int) -> list[float]:
        """Estimate target velocity for feedforward term (historical fallback)."""
        if self._prev_target is None or self._prev_time is None or self.ff_gain == 0.0:
            return [0.0] * n
        dt = now - self._prev_time
        if dt < _MIN_DT:
            return [0.0] * n
        return [
            (t - pt) / dt * self.ff_gain for t, pt in zip(target, self._prev_target, strict=True)
        ]

    def _target_changed(self, target: list[float]) -> bool:
        """Detect if target changed significantly (triggers state reset)."""
        if self._prev_target is None:
            return False
        return any(
            abs(t - pt) > self.tolerance for t, pt in zip(target, self._prev_target, strict=True)
        )

    def _derivative(self, current: list[float], now: float, n: int) -> tuple[list[float], float]:
        """Compute derivative (velocity) from position history."""
        if self._prev_joints is None or self._prev_time is None:
            return [0.0] * n, 0.0
        dt = now - self._prev_time
        if dt < _MIN_DT:
            return [0.0] * n, 0.0
        deriv = [(c - p) / dt for c, p in zip(current, self._prev_joints, strict=True)]
        return deriv, dt
