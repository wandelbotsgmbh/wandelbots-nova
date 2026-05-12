"""Action queue with interpolation, feedforward, and temporal ensembling.

Manages a sequence of joint-position waypoints and provides:
- Linear interpolation between waypoints at arbitrary tick rate
- Central-difference feedforward velocity
- Temporal ensembling: blends overlapping old/new chunks with exponential weights
"""

from __future__ import annotations

import math
import time


class ActionQueue:
    """Interpolating action queue with temporal ensembling for smooth chunk transitions.

    When a new chunk arrives via :meth:`update`, the overlapping region between
    the old and new chunk is blended using exponential weights (ACT-style temporal
    ensembling). This eliminates the velocity discontinuity at chunk boundaries.

    The blend weight for step ``i`` in the overlap region is::

        w_new = 1 - exp(-i / temperature)
        w_old = exp(-i / temperature)
        target[i] = w_old * old[i] + w_new * new[i]

    At ``i = 0`` (chunk boundary), old chunk dominates.
    At ``i = temperature * 3`` (~95% new), new chunk dominates.
    """

    def __init__(self, *, blend_temperature: float = 3.0) -> None:
        self._steps: list[list[float]] = []
        self._dt_ms: float = 0.0
        self._start_time: float = 0.0
        self._index: int = 0
        self._exhausted: bool = False
        self._blend_temperature = blend_temperature

    @property
    def active(self) -> bool:
        """True if the queue has targets to follow."""
        return len(self._steps) > 0

    def update(self, steps: list[list[float]], dt_ms: float) -> None:
        """Replace the queue with a new action chunk, blending the overlap.

        If a previous chunk is still active, the overlapping region (old chunk's
        remaining steps vs new chunk's leading steps) is blended using exponential
        weights for smooth transitions.

        Args:
            steps: List of joint-position waypoints.
            dt_ms: Time spacing between consecutive steps (milliseconds).
                   0 means single-step mode (no interpolation).
        """
        now = time.monotonic()

        # Compute how many old steps remain (for blending)
        old_remaining = self._get_remaining_steps()

        self._dt_ms = dt_ms
        self._start_time = now
        self._index = 0
        self._exhausted = False

        if not old_remaining or dt_ms <= 0.0 or not steps:
            self._steps = list(steps)
            return

        # Blend overlap region
        overlap = min(len(old_remaining), len(steps))
        n_joints = len(steps[0])
        blended: list[list[float]] = []

        for i in range(overlap):
            # Exponential blend: i=0 → mostly old, i→∞ → mostly new
            w_new = 1.0 - math.exp(-i / self._blend_temperature) if self._blend_temperature > 0 else 1.0
            w_old = 1.0 - w_new
            blended.append([
                w_old * old_remaining[i][j] + w_new * steps[i][j]
                for j in range(n_joints)
            ])

        # Append non-overlapping tail of new chunk
        blended.extend(steps[overlap:])
        self._steps = blended

    def _get_remaining_steps(self) -> list[list[float]]:
        """Get the unconsumed steps from the current chunk."""
        if not self._steps or self._dt_ms <= 0.0:
            return []

        elapsed = (time.monotonic() - self._start_time) * 1000.0
        current_idx = int(elapsed / self._dt_ms)
        current_idx = min(current_idx, len(self._steps) - 1)
        return self._steps[current_idx:]

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
