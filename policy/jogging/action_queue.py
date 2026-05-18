"""Action queue with interpolation and feedforward.

Manages a sequence of joint-position waypoints and provides:
- Linear interpolation between waypoints at arbitrary tick rate
- Central-difference feedforward velocity
- Position-matched chunk replacement (no temporal ensembling)
"""

from __future__ import annotations

import time


class ActionQueue:
    """Interpolating action queue for smooth chunk execution.

    When a new chunk arrives via :meth:`update`, the queue finds the step
    closest to the robot's current position and starts execution from there.
    No temporal ensembling — the fresh prediction is strictly better than
    stale remaining steps from the previous chunk.
    """

    def __init__(self) -> None:
        self._steps: list[list[float]] = []
        self._dt_ms: float = 0.0
        self._start_time: float = 0.0
        self._index: int = 0
        self._exhausted: bool = False

    @property
    def active(self) -> bool:
        """True if the queue has targets to follow."""
        return len(self._steps) > 0

    def update(
        self,
        steps: list[list[float]],
        dt_ms: float,
        *,
        observation_time: float | None = None,
        current_position: list[float] | None = None,
    ) -> None:
        """Replace the queue with a new action chunk.

        Finds where in the new chunk the robot currently is (via position
        matching or time-based skip) and starts execution from that point.
        No blending with old data — the fresh prediction is always better.

        Args:
            steps: List of joint-position waypoints.
            dt_ms: Time spacing between consecutive steps (milliseconds).
                   0 means single-step mode (no interpolation).
            observation_time: When the observation for this chunk was captured
                   (monotonic seconds). Used for time-based skip as fallback
                   when current_position is not provided.
            current_position: Robot's actual current joint positions. When
                   provided, finds the closest step in the new chunk and
                   starts execution from there (most accurate).
        """
        now = time.monotonic()
        chunk_start = observation_time if observation_time is not None else now

        self._dt_ms = dt_ms
        self._index = 0
        self._exhausted = False

        if not steps:
            self._steps = []
            self._start_time = now
            return

        # Find where the robot is in this chunk:
        # - Position-match (most accurate): find closest step
        # - Time-based skip (fallback): estimate from inference delay
        if current_position is not None and dt_ms > 0 and len(steps) > 1:
            skip = self._find_closest(current_position, steps)
        elif dt_ms > 0:
            skip = int((now - chunk_start) * 1000.0 / dt_ms)
        else:
            skip = 0
        skip = max(0, min(skip, len(steps) - 1))

        # Start from the matched position forward
        self._steps = steps[skip:]
        self._start_time = now

    @staticmethod
    def _find_closest(current: list[float], steps: list[list[float]]) -> int:
        """Find the step closest to the robot's current position."""
        best_idx = 0
        n = len(current)
        best_dist = sum((current[j] - steps[0][j]) ** 2 for j in range(n))
        for i in range(1, len(steps)):
            dist = sum((current[j] - steps[i][j]) ** 2 for j in range(n))
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        return best_idx

    def get_target(self, lookahead_ms: float = 0.0) -> list[float] | None:
        """Get the interpolated target position at the current time + lookahead.

        Args:
            lookahead_ms: Look ahead this many milliseconds into the trajectory.
                Compensates for phase lag from network + controller latency.

        Returns None if the queue is empty. For single-step mode (dt_ms=0),
        returns the first step without interpolation.
        """
        if not self._steps:
            return None

        if self._dt_ms <= 0.0 or len(self._steps) == 1:
            return self._steps[0]

        elapsed = (time.monotonic() - self._start_time) * 1000.0 + lookahead_ms
        frac_index = elapsed / self._dt_ms
        max_index = len(self._steps) - 1

        if frac_index >= max_index:
            self._index = max_index
            self._exhausted = True
            return self._steps[max_index]

        idx = int(frac_index)
        alpha = frac_index - idx
        self._index = idx
        self._exhausted = False

        a = self._steps[idx]
        b = self._steps[idx + 1]
        return [a[j] + alpha * (b[j] - a[j]) for j in range(len(a))]

    def get_feedforward_velocity(self) -> list[float] | None:
        """Get feedforward velocity at the current position using central difference.

        Returns None if no meaningful velocity can be computed.
        """
        if not self._steps or self._dt_ms <= 0.0 or len(self._steps) == 1:
            return None

        # Chunk fully consumed — hold position, no feedforward
        if self._exhausted:
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
