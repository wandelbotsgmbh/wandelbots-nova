"""Action queue with interpolation and feedforward for PID jogging.

Manages a sequence of joint-position waypoints and provides:
- Linear interpolation between waypoints at arbitrary tick rate
- Central-difference feedforward velocity using the surrounding trajectory
- Inference-delay-aware chunk merging (skips stale leading steps)
"""

from __future__ import annotations

import time


class ActionQueue:
    """Interpolating action queue for smooth trajectory following.

    Stores a chunk of future joint targets and produces interpolated positions
    and feedforward velocities at any query rate (typically ~100Hz PID ticks).

    When a new chunk arrives via :meth:`update`, the queue accounts for the time
    elapsed since the previous chunk started — it skips steps that correspond to
    time already passed (inference delay), so the robot doesn't replay stale targets.
    """

    def __init__(self) -> None:
        self._steps: list[list[float]] = []
        self._dt_ms: float = 0.0
        self._start_time: float = 0.0
        self._index: int = 0

    @property
    def active(self) -> bool:
        """True if the queue has targets to follow."""
        return len(self._steps) > 0

    def update(self, steps: list[list[float]], dt_ms: float) -> None:
        """Replace the queue with a new action chunk.

        If ``dt_ms > 0`` and a previous chunk was actively being consumed,
        the elapsed time since the last update is used to skip leading steps
        that the robot has already passed (inference delay compensation).

        Args:
            steps: List of joint-position waypoints.
            dt_ms: Time spacing between consecutive steps (milliseconds).
                   0 means single-step mode (no interpolation).
        """
        now = time.monotonic()

        # Only skip if we had an active timed chunk being consumed
        skip = 0
        if dt_ms > 0 and self._dt_ms > 0 and self._steps:
            elapsed_ms = (now - self._start_time) * 1000.0
            skip = int(elapsed_ms / dt_ms)
            skip = min(skip, max(0, len(steps) - 1))

        self._steps = steps[skip:] if skip > 0 else steps
        self._dt_ms = dt_ms
        self._start_time = now
        self._index = 0

    def get_target(self) -> list[float] | None:
        """Get the interpolated target position at the current time.

        Returns None if the queue is empty. For single-step mode (dt_ms=0),
        returns the first step without interpolation.
        """
        if not self._steps:
            return None

        if self._dt_ms <= 0.0 or len(self._steps) == 1:
            return self._steps[0]

        elapsed = (time.monotonic() - self._start_time) * 1000.0
        frac_index = elapsed / self._dt_ms
        max_index = len(self._steps) - 1

        if frac_index >= max_index:
            self._index = max_index
            return self._steps[max_index]

        idx = int(frac_index)
        alpha = frac_index - idx
        self._index = idx

        a = self._steps[idx]
        b = self._steps[idx + 1]
        return [a[j] + alpha * (b[j] - a[j]) for j in range(len(a))]

    def get_feedforward_velocity(self) -> list[float] | None:
        """Get feedforward velocity at the current position using central difference.

        Uses the widest symmetric window available around the current index:
        ``velocity = (steps[i+k] - steps[i-k]) / (2k * dt_step)``

        Returns None if no meaningful velocity can be computed (single step,
        or dt_ms=0).
        """
        if not self._steps or self._dt_ms <= 0.0 or len(self._steps) == 1:
            return None

        idx = self._index
        max_index = len(self._steps) - 1
        dt_s = self._dt_ms / 1000.0

        # Widest symmetric window (up to 3 steps each side)
        k = min(idx, max_index - idx, 3)
        if k == 0:
            # Boundary: forward or backward difference
            if idx < max_index:
                ahead, here = self._steps[idx + 1], self._steps[idx]
            else:
                ahead, here = self._steps[idx], self._steps[idx - 1]
            return [(a - h) / dt_s for a, h in zip(ahead, here, strict=False)]

        behind = self._steps[idx - k]
        ahead = self._steps[idx + k]
        span_dt = 2 * k * dt_s
        return [(a - b) / span_dt for a, b in zip(ahead, behind, strict=False)]
