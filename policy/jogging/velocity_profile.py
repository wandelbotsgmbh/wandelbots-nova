"""Trapezoidal velocity profile with ROS2-style trajectory control.

Applies the same control law as ROS2's joint_trajectory_controller:
- Track desired state internally (position + velocity)
- On chunk arrival: start from last desired state, not actual state
- Each tick: sample trajectory at current time, compute feedforward + PID
- Advance time deterministically

This prevents snap-back because the trajectory always starts from where
we last COMMANDED (desired state), not from a stale observation position.
The PID correction handles the gap between desired and actual.
"""

from __future__ import annotations

import time

_VEL_ZERO_THRESHOLD = 0.001
_VEL_DECAY = 0.8


class VelocityProfile:
    """Trapezoidal velocity profile with internal desired-state tracking.

    Control law (same as ROS2 joint_trajectory_controller with velocity interface):

        velocity_cmd = feedforward_velocity * ff_scale + p_gain * (desired_pos - actual_pos)

    On chunk transition:
    - New trajectory starts from `last_desired_position` (not actual)
    - Trajectory is: [last_desired, future_steps...]
    - PID correction handles the actual-vs-desired gap
    - No snap-back because desired is always consistent/forward

    This unified approach works for both:
    - Policy inference (observation_time in the past → skips stale steps)
    - Jogger streaming (observation_time = now → starts from last desired)
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

        # Internal desired state (ROS2: last_commanded_state_)
        # Tracks where the profile says the robot SHOULD be.
        # Used as starting point for new chunks.
        self._desired_position: list[float] | None = None

    @property
    def done(self) -> bool:
        """True when the profile has been fully traversed or no chunk loaded."""
        return self._done

    def _limit(self, idx: int) -> float:
        if isinstance(self._vel_limit, list):
            return self._vel_limit[idx] if idx < len(self._vel_limit) else 2.0
        return self._vel_limit

    def set_chunk(
        self,
        steps: list[list[float]],
        dt_ms: float,
        *,
        current: list[float] | None = None,
        observation_time: float | None = None,
        final: bool = True,
    ) -> None:
        """Set a new action chunk with ROS2-style trajectory splicing.

        Args:
            steps: Waypoint positions from the policy/user.
            dt_ms: Time spacing between steps in milliseconds.
            current: Robot's actual current position. Used as fallback for the
                first chunk, and for PID correction during execution.
            observation_time: When the observation was captured (monotonic time).
                Steps before (now - observation_time) are considered stale and
                discarded. Defaults to now (no discard).
            final: Whether this chunk will be executed to completion. When True,
                applies trapezoidal ramp envelope (robot stops at end). When
                False (continuous mode), skips the ramp to avoid repeated
                slow-downs when chunks are replaced mid-execution.
        """
        if not steps:
            self._profile = None
            self._steps = None
            self._target = None
            self._done = True
            return

        # Single-step: P-controller mode
        if len(steps) == 1 or dt_ms <= 0:
            self._profile = None
            self._steps = None
            self._target = steps[-1]
            self._done = False
            return

        # --- Trajectory splice (ROS2-style) ---
        now = time.monotonic()
        dt_s_raw = dt_ms / 1000.0

        # 1. Discard steps that are in the past
        obs_time = observation_time if observation_time is not None else now
        past_steps = int((now - obs_time) / dt_s_raw) if dt_s_raw > 0 else 0
        past_steps = max(0, min(past_steps, len(steps) - 1))
        future_steps = steps[past_steps:]

        # 2. Determine starting point for the new trajectory.
        # ROS2 rule: start from last desired state (interpolate_from_desired_state).
        # Only prepend when steps were actually skipped (inference delay case).
        # When no steps are skipped (jogger), the chunk already starts from
        # roughly where desired is — prepending would add a redundant point.
        if past_steps > 0:
            start_point = self._desired_position or current
            trajectory = [start_point, *future_steps] if start_point is not None else list(future_steps)
        else:
            trajectory = list(future_steps)

        # Need at least 2 points for a profile
        if len(trajectory) < 2:  # noqa: PLR2004
            self._target = trajectory[-1] if trajectory else None
            self._profile = None
            self._steps = None
            self._done = not trajectory
            return

        # 3. Compute profile on spliced trajectory
        self._target = None
        self._steps = trajectory
        self._done = False
        self._start_time = now
        n_steps = len(trajectory)
        n = self._n

        dt_s = self._compute_dt(trajectory, dt_s_raw, n_steps, n)
        self._dt_s = dt_s
        self._profile = self._build_profile(trajectory, dt_s, n_steps, n, ramp=final)

        # Smooth velocity transition: blend first steps with current velocity
        blend_count = min(3, n_steps)
        for i in range(blend_count):
            alpha = (i + 1) / (blend_count + 1)
            for j in range(n):
                self._profile[i][j] = (
                    (1 - alpha) * self._current_vel[j] + alpha * self._profile[i][j]
                )

    def compute(self, current: list[float], now: float) -> list[float]:
        """Compute velocity command: feedforward + PID correction.

        ROS2 control law:
            velocity = feedforward_velocity + p_gain * (desired_pos - actual_pos)

        Also updates internal desired_position (last_commanded_state).

        Args:
            current: Robot's actual joint/TCP position.
            now: Current monotonic time.
        """
        n = self._n

        # Single-position mode: P-controller
        if self._target is not None:
            vel = []
            for j in range(n):
                error = self._target[j] - current[j]
                v = self._p_gain * error
                v = max(-self._limit(j), min(self._limit(j), v))
                vel.append(v)
            self._current_vel = vel
            self._desired_position = list(self._target)
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
            # Reached end: hold last position
            self._current_vel = [0.0] * n
            self._done = True
            self._desired_position = list(self._steps[max_idx])
            return [0.0] * n

        if frac_idx < 0:
            frac_idx = 0.0

        # Interpolate desired state from trajectory (position + velocity)
        idx = int(frac_idx)
        alpha = frac_idx - idx
        next_idx = min(idx + 1, max_idx)

        # Desired position at current time
        sa = self._steps[idx]
        sb = self._steps[next_idx]
        desired_pos = [sa[j] + alpha * (sb[j] - sa[j]) for j in range(n)]

        # Feedforward velocity from profile at current time
        a = self._profile[idx]
        b = self._profile[next_idx]
        ff_vel = [a[j] + alpha * (b[j] - a[j]) for j in range(n)]

        # ROS2 control law: velocity = ff + P * (desired - actual)
        vel = []
        for j in range(n):
            error = desired_pos[j] - current[j]
            v = ff_vel[j] + self._p_gain * error
            v = max(-self._limit(j), min(self._limit(j), v))
            vel.append(v)

        # Update internal desired state (ROS2: last_commanded_state)
        self._current_vel = vel
        self._desired_position = desired_pos
        return vel

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------

    def _compute_dt(
        self, steps: list[list[float]], dt_s: float, n_steps: int, n: int,
    ) -> float:
        """Stretch dt_s if any raw velocity exceeds the limit."""
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
        self, steps: list[list[float]], dt_s: float, n_steps: int, n: int, *, ramp: bool = True,
    ) -> list[list[float]]:
        """Compute raw velocities and apply envelope.

        When ramp=True: full trapezoidal envelope (ramp-up + ramp-down).
        When ramp=False: ramp-down only (brake at end, no slow start).
        """
        raw_vels: list[list[float]] = []
        for i in range(n_steps):
            if i == 0:
                vel = [(steps[1][j] - steps[0][j]) / dt_s for j in range(n)]
            elif i == n_steps - 1:
                vel = [0.0] * n
            else:
                vel = [(steps[i + 1][j] - steps[i - 1][j]) / (2 * dt_s) for j in range(n)]
            raw_vels.append(vel)

        ramp_steps = self._ramp_steps
        profile: list[list[float]] = []
        for i in range(n_steps):
            ramp_up = min(1.0, (i + 1) / ramp_steps) if (ramp and ramp_steps > 0) else 1.0
            ramp_down = min(1.0, (n_steps - i) / ramp_steps) if ramp_steps > 0 else 1.0
            envelope = min(ramp_up, ramp_down)
            vel = [raw_vels[i][j] * envelope for j in range(n)]
            profile.append(vel)
        return profile
