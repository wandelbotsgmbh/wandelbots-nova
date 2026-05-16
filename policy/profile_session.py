"""ProfileSession — precomputed velocity profile motion via NOVA Jogging API.

Instead of PID control, computes the full velocity trajectory for each chunk
upfront. Guarantees zero overshoot by forcing velocity to zero at the last step.

Algorithm:
1. Multi-step chunks: compute velocity at each step from position differences,
   apply trapezoidal ramp envelope, interpolate at runtime.
2. Single-step targets: P-controller drives toward the target position.
3. Chunk transitions: blend current velocity into new profile for smoothness.
4. Exhaustion: velocity decays to zero (smooth stop).
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from policy.pidjogging.session import PidJoggingSession
from policy.types import PidConfig

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from policy.types import JoggingMode, ProfileConfig, SafetyGuard


_VEL_ZERO_THRESHOLD = 0.001
_VEL_DECAY = 0.8


class _VelocityProfile:
    """Computes and interpolates a velocity profile for an action chunk."""

    def __init__(self, n_joints: int, vel_limit: float | list[float], ramp_steps: int, p_gain: float) -> None:
        self._n = n_joints
        self._vel_limit = vel_limit
        self._ramp_steps = ramp_steps
        self._p_gain = p_gain
        self._profile: list[list[float]] | None = None
        self._target: list[float] | None = None
        self._dt_s: float = 0.0
        self._start_time: float = 0.0
        self._current_vel: list[float] = [0.0] * n_joints

    def _limit(self, idx: int) -> float:
        if isinstance(self._vel_limit, list):
            return self._vel_limit[idx] if idx < len(self._vel_limit) else 2.0
        return self._vel_limit

    def set_chunk(self, steps: list[list[float]], dt_ms: float) -> None:
        """Precompute velocity profile for this chunk."""
        if not steps:
            self._profile = None
            self._target = None
            return

        # Single position: P-controller mode
        if len(steps) == 1 or dt_ms <= 0:
            self._profile = None
            self._target = steps[-1]
            return

        # Multi-step: build velocity profile
        self._target = None
        self._dt_s = dt_ms / 1000.0
        self._start_time = time.monotonic()
        n_steps = len(steps)
        n = self._n

        # Compute raw velocities from position differences
        raw_vels: list[list[float]] = []
        for i in range(n_steps):
            if i == 0:
                vel = [(steps[1][j] - steps[0][j]) / self._dt_s for j in range(n)]
            elif i == n_steps - 1:
                vel = [0.0] * n  # Zero at end → no overshoot
            else:
                vel = [(steps[i + 1][j] - steps[i - 1][j]) / (2 * self._dt_s) for j in range(n)]
            raw_vels.append(vel)

        # Apply trapezoidal envelope
        ramp = self._ramp_steps
        self._profile = []
        for i in range(n_steps):
            ramp_up = min(1.0, (i + 1) / ramp) if ramp > 0 else 1.0
            ramp_down = min(1.0, (n_steps - i) / ramp) if ramp > 0 else 1.0
            envelope = min(ramp_up, ramp_down)
            vel = [raw_vels[i][j] * envelope for j in range(n)]
            # Clamp per-axis
            vel = [max(-self._limit(j), min(self._limit(j), vel[j])) for j in range(n)]
            self._profile.append(vel)

        # Blend first steps with current velocity for smooth transition
        blend_count = min(3, n_steps)
        for i in range(blend_count):
            alpha = (i + 1) / (blend_count + 1)
            for j in range(n):
                self._profile[i][j] = (1 - alpha) * self._current_vel[j] + alpha * self._profile[i][j]

    def compute(self, current: list[float], now: float) -> list[float]:
        """Get velocity command for current time."""
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
            return vel

        # No profile: decay to zero
        if self._profile is None:
            self._current_vel = [v * _VEL_DECAY for v in self._current_vel]
            if all(abs(v) < _VEL_ZERO_THRESHOLD for v in self._current_vel):
                self._current_vel = [0.0] * n
            return list(self._current_vel)

        # Profile playback: interpolate
        elapsed = now - self._start_time
        frac_idx = elapsed / self._dt_s
        max_idx = len(self._profile) - 1

        if frac_idx >= max_idx:
            # Profile exhausted
            self._current_vel = [0.0] * n
            return [0.0] * n

        idx = int(frac_idx)
        alpha = frac_idx - idx
        a = self._profile[idx]
        b = self._profile[min(idx + 1, max_idx)]
        vel = [a[j] + alpha * (b[j] - a[j]) for j in range(n)]
        self._current_vel = vel
        return vel


class ProfileSession(PidJoggingSession):
    """Motion session using precomputed velocity profiles.

    Inherits PidJoggingSession for the jogging WebSocket lifecycle, state
    streaming, IO writing, and safety guard infrastructure. Overrides only
    the velocity computation to use the profile approach instead of PID.
    """

    def __init__(
        self,
        motion_group: MotionGroup,
        config: ProfileConfig,
        *,
        tcp: str | None = None,
        safety_guards: list[SafetyGuard] | None = None,
        mode: JoggingMode = "joint",
    ) -> None:
        # Create a PidConfig with matching state_rate for the base class
        pid_config = PidConfig(
            velocity_limit=config.velocity_limit,
            state_rate_ms=config.state_rate_ms,
        )
        super().__init__(
            motion_group=motion_group,
            config=pid_config,
            tcp=tcp,
            safety_guards=safety_guards,
            mode=mode,
        )
        self._profile_config = config
        self._velocity_profile = _VelocityProfile(
            n_joints=6,  # updated on start when we know DOF
            vel_limit=config.velocity_limit,
            ramp_steps=config.ramp_steps,
            p_gain=config.p_gain,
        )

    async def start(self) -> None:
        """Start session and initialize profile with correct DOF."""
        await super().start()
        if self._num_joints:
            self._velocity_profile = _VelocityProfile(
                n_joints=self._num_joints,
                vel_limit=self._profile_config.velocity_limit,
                ramp_steps=self._profile_config.ramp_steps,
                p_gain=self._profile_config.p_gain,
            )

    def update_chunk(self, steps: list[list[float]], dt_ms: float, *, observation_time: float | None = None) -> None:
        """Update the velocity profile with new chunk steps."""
        self._velocity_profile.set_chunk(steps, dt_ms)
        # Also update the base class queue (for state tracking / exhaustion detection)
        super().update_chunk(steps, dt_ms, observation_time=observation_time)

    def _compute_velocity_with_safety(self) -> list[float]:
        """Compute velocity from the precomputed profile."""
        current = self._get_current_for_pid()
        if current is None:
            return self._get_zero_velocity()

        # Run safety guards
        current_robot_state = self._build_robot_state()
        if self._safety_guards and current_robot_state is not None:
            self._check_safety_guards(current_robot_state)
        if current_robot_state is not None:
            self._prev_state = current_robot_state

        # Use the velocity profile instead of PID
        return self._velocity_profile.compute(current, time.monotonic())
