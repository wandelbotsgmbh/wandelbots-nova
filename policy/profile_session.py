"""ProfileSession — precomputed velocity profile motion via NOVA Jogging API.

Computes the velocity trajectory for each chunk upfront and advances through
it based on the robot's actual position, not wall-clock time. This guarantees:
- Zero overshoot (velocity is zero at the last step, and that step is only
  reached when the robot physically arrives there)
- Correct timing under latency (the profile slows down if the robot lags)

Algorithm:
1. Multi-step chunks: compute velocity at each step from position differences,
   apply trapezoidal ramp envelope, advance based on robot position at runtime.
2. Single-step targets: P-controller drives toward the target position.
3. Chunk transitions: blend current velocity into new profile for smoothness.
4. Exhaustion: velocity is zero (last step reached).
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

from policy.pidjogging.session import PidJoggingSession
from policy.types import PidConfig

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from policy.types import JoggingMode, ProfileConfig, SafetyGuard


_VEL_ZERO_THRESHOLD = 0.001
_VEL_DECAY = 0.8
_SEGMENT_LENGTH_SQ_MIN = 1e-12


class _VelocityProfile:
    """Computes a velocity profile and advances through it based on robot position."""

    def __init__(self, n_joints: int, vel_limit: float | list[float], ramp_steps: int, p_gain: float) -> None:
        self._n = n_joints
        self._vel_limit = vel_limit
        self._ramp_steps = ramp_steps
        self._p_gain = p_gain
        self._profile: list[list[float]] | None = None
        self._steps: list[list[float]] | None = None  # waypoint positions
        self._target: list[float] | None = None  # single-position target
        self._current_idx: float = 0.0  # fractional index into profile
        self._current_vel: list[float] = [0.0] * n_joints

    def _limit(self, idx: int) -> float:
        if isinstance(self._vel_limit, list):
            return self._vel_limit[idx] if idx < len(self._vel_limit) else 2.0
        return self._vel_limit

    def set_chunk(self, steps: list[list[float]], dt_ms: float, current: list[float] | None = None) -> None:
        """Precompute velocity profile for this chunk.

        Args:
            steps: Waypoint positions.
            dt_ms: Time spacing between steps.
            current: Robot's current position. Used to find the starting
                index in the chunk (skip steps the robot has already passed).
        """
        if not steps:
            self._profile = None
            self._steps = None
            self._target = None
            return

        # Single position: P-controller mode
        if len(steps) == 1 or dt_ms <= 0:
            self._profile = None
            self._steps = None
            self._target = steps[-1]
            return

        # Multi-step: build velocity profile
        self._target = None
        self._steps = steps
        n_steps = len(steps)
        n = self._n

        # Find where the robot currently is in this chunk
        if current is not None:
            self._current_idx = self._find_closest_global(current, steps, n)
        else:
            self._current_idx = 0.0

        # Compute time step, stretching if velocities exceed limits
        dt_s = self._compute_dt(steps, dt_ms / 1000.0, n_steps, n)

        # Build profile: raw velocities + trapezoidal envelope
        self._profile = self._build_profile(steps, dt_s, n_steps, n)

        # Blend first steps with current velocity for smooth transition
        blend_count = min(3, n_steps)
        for i in range(blend_count):
            alpha = (i + 1) / (blend_count + 1)
            for j in range(n):
                self._profile[i][j] = (1 - alpha) * self._current_vel[j] + alpha * self._profile[i][j]

    def _compute_dt(self, steps: list[list[float]], dt_s: float, n_steps: int, n: int) -> float:
        """Compute dt_s, stretching if any raw velocity exceeds the limit."""
        max_ratio = 1.0
        for i in range(n_steps):
            if i == 0:
                vel = [(steps[1][j] - steps[0][j]) / dt_s for j in range(n)]
            elif i == n_steps - 1:
                continue  # last step is forced zero
            else:
                vel = [(steps[i + 1][j] - steps[i - 1][j]) / (2 * dt_s) for j in range(n)]
            for j in range(n):
                limit = self._limit(j)
                if limit > 0:
                    max_ratio = max(max_ratio, abs(vel[j]) / limit)
        return dt_s * max_ratio

    def _build_profile(self, steps: list[list[float]], dt_s: float, n_steps: int, n: int) -> list[list[float]]:
        """Compute raw velocities and apply trapezoidal envelope."""
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

    @staticmethod
    def _find_closest_global(
        current: list[float], steps: list[list[float]], n: int,
    ) -> float:
        """Find the step closest to current position (global search).

        Used on chunk arrival to determine the starting index. Searches all
        steps and returns a fractional index with sub-step interpolation.
        """
        best_idx = 0
        best_dist = sum((current[j] - steps[0][j]) ** 2 for j in range(n))

        for i in range(1, len(steps)):
            dist = sum((current[j] - steps[i][j]) ** 2 for j in range(n))
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        # Sub-step interpolation between best_idx and its neighbor
        if best_idx < len(steps) - 1:
            a, b = steps[best_idx], steps[best_idx + 1]
            ab_sq = sum((b[j] - a[j]) ** 2 for j in range(n))
            if ab_sq > _SEGMENT_LENGTH_SQ_MIN:
                t = sum((current[j] - a[j]) * (b[j] - a[j]) for j in range(n)) / ab_sq
                t = max(0.0, min(1.0, t))
                return best_idx + t

        return float(best_idx)

    def _find_progress(self, current: list[float]) -> float:
        """Find how far along the trajectory the robot actually is.

        Searches forward from current_idx to find the step closest to
        the robot's actual position. Only searches forward to handle
        direction changes within a chunk correctly.
        """
        steps = self._steps
        if not steps:
            return 0.0

        n = self._n
        max_idx = len(steps) - 1
        search_start = int(self._current_idx)

        best_idx = search_start
        best_dist = math.inf

        # Search forward from current position (+ look 1 step back for interpolation)
        start = max(0, search_start - 1)
        for i in range(start, len(steps)):
            dist = sum((current[j] - steps[i][j]) ** 2 for j in range(n))
            if dist < best_dist:
                best_dist = dist
                best_idx = i
            elif i > search_start + 3:
                # Stop searching once distance starts increasing (past the closest)
                break

        # Refine: interpolate between best_idx and best_idx+1 for sub-step precision
        if best_idx < max_idx:
            a = steps[best_idx]
            b = steps[best_idx + 1]
            # Project current position onto the line segment a→b
            ab_sq = sum((b[j] - a[j]) ** 2 for j in range(n))
            if ab_sq > _SEGMENT_LENGTH_SQ_MIN:
                t = sum((current[j] - a[j]) * (b[j] - a[j]) for j in range(n)) / ab_sq
                t = max(0.0, min(1.0, t))
                return best_idx + t

        return float(best_idx)

    def compute(self, current: list[float], _now: float) -> list[float]:
        """Get velocity command based on robot's actual position."""
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
        if self._profile is None or self._steps is None:
            self._current_vel = [v * _VEL_DECAY for v in self._current_vel]
            if all(abs(v) < _VEL_ZERO_THRESHOLD for v in self._current_vel):
                self._current_vel = [0.0] * n
            return list(self._current_vel)

        # Position-based profile playback
        max_idx = len(self._profile) - 1

        # Find where the robot actually is along the trajectory
        progress = self._find_progress(current)

        # Only advance forward (never go back along the profile)
        self._current_idx = max(self._current_idx, progress)
        frac_idx = self._current_idx

        # If robot hasn't reached step[0] yet, drive toward it
        if frac_idx < 0.1:  # noqa: PLR2004
            first_step = self._steps[0]
            dist_sq = sum((current[j] - first_step[j]) ** 2 for j in range(n))
            if dist_sq > _SEGMENT_LENGTH_SQ_MIN:
                vel = []
                for j in range(n):
                    error = first_step[j] - current[j]
                    # Use profile[0] velocity as feedforward + P correction
                    v = self._profile[0][j] + self._p_gain * error
                    v = max(-self._limit(j), min(self._limit(j), v))
                    vel.append(v)
                self._current_vel = vel
                return vel

        if frac_idx >= max_idx:
            # Reached last step → zero velocity
            self._current_vel = [0.0] * n
            return [0.0] * n

        # Interpolate velocity from profile at current progress
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
        # Pass current robot position so the profile can find the right starting index
        current = self._get_current_for_pid()
        self._velocity_profile.set_chunk(steps, dt_ms, current=current)
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
