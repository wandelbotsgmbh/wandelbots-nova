"""Trapezoidal velocity profile for time-based motion control.

Computes velocities from position waypoints and plays them back time-based
with P-correction for tracking error.

This is a temporary client-side implementation. It will be replaced by
NOVA's native waypoint jogging API once available.
"""

from __future__ import annotations

import time

_VEL_ZERO_THRESHOLD = 0.001
_VEL_DECAY = 0.8


class VelocityProfile:
    """Trapezoidal velocity profile with P-correction.

    Control law each tick:
        velocity = feedforward_from_profile + p_gain * (expected_pos - actual_pos)

    The profile advances by elapsed time. When all steps are consumed,
    velocity goes to zero and `done` becomes True.
    """

    def __init__(
        self,
        n_joints: int,
        vel_limit: float | list[float],
        ramp_steps: int,
        p_gain: float,
    ) -> None:
        self._n = n_joints
        self._vel_limit = vel_limit
        self._ramp_steps = ramp_steps
        self._p_gain = p_gain
        self._profile: list[list[float]] | None = None
        self._steps: list[list[float]] | None = None
        self._target: list[float] | None = None
        self._current_vel: list[float] = [0.0] * n_joints
        self._done: bool = True
        self._start_time: float = 0.0
        self._dt_s: float = 0.0

    @property
    def done(self) -> bool:
        """True when the profile has been fully traversed or no chunk loaded."""
        return self._done

    def _limit(self, idx: int) -> float:
        if isinstance(self._vel_limit, list):
            return self._vel_limit[idx] if idx < len(self._vel_limit) else 2.0
        return self._vel_limit

    def set_chunk(self, steps: list[list[float]], dt_ms: float) -> None:
        """Set a new action chunk.

        Args:
            steps: Waypoint positions.
            dt_ms: Time spacing between steps in milliseconds.
        """
        if not steps:
            self._profile = None
            self._steps = None
            self._target = None
            self._done = True
            return

        # Single-step: P-controller mode (immediate target, no trajectory)
        # Done after one control cycle so the executor paces inference
        if len(steps) == 1 or dt_ms <= 0:
            self._profile = None
            self._steps = None
            self._target = steps[-1]
            self._done = False
            self._start_time = time.monotonic()
            self._dt_s = 0.033  # ~30Hz inference rate for single-step
            return

        # Multi-step: compute trapezoidal profile
        self._target = None
        self._steps = steps
        self._done = False
        self._start_time = time.monotonic()
        n_steps = len(steps)
        n = self._n

        dt_s = self._compute_dt(steps, dt_ms / 1000.0, n_steps, n)
        self._dt_s = dt_s
        self._profile = self._build_profile(steps, dt_s, n_steps, n)

    def compute(self, current: list[float], now: float) -> list[float]:
        """Compute velocity command: feedforward + P-correction.

        Args:
            current: Robot's actual position.
            now: Current monotonic time.
        """
        n = self._n

        # Single-position mode: P-controller
        if self._target is not None:
            if now - self._start_time >= self._dt_s:
                self._done = True
            vel = []
            for j in range(n):
                error = self._target[j] - current[j]
                v = self._p_gain * error
                v = max(-self._limit(j), min(self._limit(j), v))
                vel.append(v)
            self._current_vel = vel
            return vel

        # No profile loaded: decay to zero
        if self._profile is None or self._steps is None:
            self._current_vel = [v * _VEL_DECAY for v in self._current_vel]
            if all(abs(v) < _VEL_ZERO_THRESHOLD for v in self._current_vel):
                self._current_vel = [0.0] * n
            return list(self._current_vel)

        # Time-based profile playback
        max_idx = len(self._profile) - 1
        elapsed = now - self._start_time
        frac_idx = elapsed / self._dt_s if self._dt_s > 0 else 0.0

        if frac_idx >= max_idx:
            self._current_vel = [0.0] * n
            self._done = True
            return [0.0] * n

        if frac_idx < 0:
            frac_idx = 0.0

        # Interpolate feedforward velocity from profile
        idx = int(frac_idx)
        alpha = frac_idx - idx
        next_idx = min(idx + 1, max_idx)
        a = self._profile[idx]
        b = self._profile[next_idx]
        ff_vel = [a[j] + alpha * (b[j] - a[j]) for j in range(n)]

        # Interpolate expected position for P-correction
        sa = self._steps[idx]
        sb = self._steps[next_idx]
        expected = [sa[j] + alpha * (sb[j] - sa[j]) for j in range(n)]

        # velocity = feedforward + P * tracking_error
        vel = []
        for j in range(n):
            error = expected[j] - current[j]
            v = ff_vel[j] + self._p_gain * error
            v = max(-self._limit(j), min(self._limit(j), v))
            vel.append(v)
        self._current_vel = vel
        return vel

    # -------------------------------------------------------------------------
    # Internal
    # -------------------------------------------------------------------------

    def _compute_dt(
        self, steps: list[list[float]], dt_s: float, n_steps: int, n: int,
    ) -> float:
        """Stretch dt_s if any velocity exceeds the limit."""
        max_ratio = 1.0
        for i in range(n_steps):
            if i == 0:
                vel = [(steps[1][j] - steps[0][j]) / dt_s for j in range(n)]
            elif i == n_steps - 1:
                continue
            else:
                vel = [(steps[i + 1][j] - steps[i - 1][j]) / (2 * dt_s) for j in range(n)]
            for j in range(n):
                limit = self._limit(j)
                if limit > 0:
                    max_ratio = max(max_ratio, abs(vel[j]) / limit)
        return dt_s * max_ratio

    def _build_profile(
        self, steps: list[list[float]], dt_s: float, n_steps: int, n: int,
    ) -> list[list[float]]:
        """Compute velocities with trapezoidal ramp envelope."""
        raw_vels: list[list[float]] = []
        for i in range(n_steps):
            if i == 0:
                vel = [(steps[1][j] - steps[0][j]) / dt_s for j in range(n)]
            elif i == n_steps - 1:
                vel = [0.0] * n
            else:
                vel = [(steps[i + 1][j] - steps[i - 1][j]) / (2 * dt_s) for j in range(n)]
            raw_vels.append(vel)

        ramp = self._ramp_steps
        profile: list[list[float]] = []
        for i in range(n_steps):
            ramp_up = min(1.0, (i + 1) / ramp) if ramp > 0 else 1.0
            ramp_down = min(1.0, (n_steps - i) / ramp) if ramp > 0 else 1.0
            envelope = min(ramp_up, ramp_down)
            vel = [raw_vels[i][j] * envelope for j in range(n)]
            profile.append(vel)
        return profile
