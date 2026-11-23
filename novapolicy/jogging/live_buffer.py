"""Rolling buffer of measured live targets, replayed as a horizon on a fixed grid (ADR 0013).

The robot trails the live target by about ``window_ms``; a target keeps its absolute timestamp
however often the window is re-sent.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import logging
import math
from typing import TYPE_CHECKING, ClassVar, TypeAlias

from novapolicy.jogging.clock import ServerMs

if TYPE_CHECKING:
    from novapolicy.jogging.pipeline import ModuleContext
    from novapolicy.jogging.waypoints import PendingChunk

logger = logging.getLogger(__name__)

TimedPoint: TypeAlias = tuple[float, list[float]]
"""One measured target and the trajectory time it was produced at."""

LIVE_LEAD_MS = 100.0
"""How far ahead of current motion a replacement horizon starts, in ms."""

TIMELINE_GRID_MS = 10.0
"""Quantisation of the session timeline, in ms: every push lands on one absolute grid."""

MIN_BUFFER_SAMPLES = 2
"""A rolling buffer needs at least two samples to define a spacing."""

MAX_LIVE_SAMPLES = 256
"""Hard cap on retained samples, on top of the time window, for a clock that does not advance
(a session not started yet or a stalled state stream)."""


def trajectory_elapsed_ms(now_ms: float, anchor_ms: float, previous_ms: float) -> float:
    """Trajectory time on the **server** clock: quantised, never backwards, forward jumps kept.

    ``now_ms`` must come from the server jogger clock, not wall time.
    """
    elapsed_ms = math.floor(max(0.0, now_ms - anchor_ms) / TIMELINE_GRID_MS) * TIMELINE_GRID_MS
    return max(previous_ms, elapsed_ms)


def resample_evenly(
    samples: list[TimedPoint], step_s: float, *, start: float | None = None
) -> list[list[float]]:
    """Linearly interpolate irregularly timed samples onto an evenly spaced grid.

    ``start`` pins the grid to an absolute phase; without it the grid begins at the oldest
    sample. Grid points past the newest sample hold its value.
    """
    start = samples[0][0] if start is None else start
    end = samples[-1][0]
    if end < start:
        return []
    # Round the span up so the grid reaches the newest sample; the epsilon absorbs float
    # division (two steps can come out as 1.9999999999999996).
    span_steps = math.ceil((end - start) / step_s - 1e-9)
    count = max(MIN_BUFFER_SAMPLES, span_steps + 1)
    grid: list[list[float]] = []
    index = 0
    for i in range(count):
        moment = start + i * step_s
        while index + 2 < len(samples) and samples[index + 1][0] <= moment:
            index += 1
        before_t, before = samples[index]
        after_t, after = samples[min(index + 1, len(samples) - 1)]
        span = after_t - before_t
        fraction = 0.0 if span <= 0 else min(1.0, max(0.0, (moment - before_t) / span))
        grid.append([before[k] + fraction * (after[k] - before[k]) for k in range(len(before))])
    return grid


def append_and_prune(
    buffer: list[TimedPoint], moment: float, values: list[float], *, window_ms: float
) -> None:
    """Add a target and drop samples older than ``window_ms``."""
    buffer.append((moment, list(values)))
    cutoff = moment - window_ms / 1000.0
    while len(buffer) > MIN_BUFFER_SAMPLES and (
        buffer[1][0] < cutoff or len(buffer) > MAX_LIVE_SAMPLES
    ):
        del buffer[0]


def horizon(
    buffer: list[TimedPoint], *, single_step_dt_ms: float
) -> tuple[list[list[float]], float, float] | None:
    """The buffer as ``(steps, grid_start_s, step_dt_ms)``, or ``None`` if too short.

    On ``None`` the caller sends the bare target instead of waiting for the window to fill.
    """
    span_s = buffer[-1][0] - buffer[0][0]
    if len(buffer) < MIN_BUFFER_SAMPLES or span_s <= 0:
        return None

    # One absolute grid, not one starting at the oldest sample (that changes with every
    # prune); the spacing is never coarser than half the window.
    step_dt_s = min(single_step_dt_ms / 1000.0, span_s / 2.0)
    grid_start = math.ceil(buffer[0][0] / step_dt_s) * step_dt_s
    steps = resample_evenly(buffer, step_dt_s, start=grid_start)
    if not steps:
        return None
    return steps, grid_start, step_dt_s * 1000.0


def timeline_timestamp_ms(
    anchor_ms: float, trajectory_time_s: float, *, lead_ms: float = LIVE_LEAD_MS
) -> int:
    """Absolute server timestamp ``anchor + T + lead`` for content generated at time ``T``."""
    return int(anchor_ms + trajectory_time_s * 1000.0 + lead_ms)


def horizon_end_time_s(grid_start_s: float, window_ms: float) -> float:
    """Trajectory time the replayed window reaches up to: ``window_ms`` is the whole horizon."""
    return grid_start_s + window_ms / 1000.0


@dataclass(frozen=True, slots=True)
class Replay:
    """The held live targets as a horizon on an even grid."""

    steps: list[list[float]]
    step_ms: float
    """Spacing of ``steps``."""
    end_time_s: float
    """Trajectory time the horizon reaches up to; mapped (plus lead) to its first timestamp."""


@dataclass
class LiveBuffer:
    """Chunk module: replay the last ``window_ms`` of live targets as a horizon on an even grid.

    For dense live targets (jogging), not policy steps (ADR 0004). Its own timeline (T2); a
    chunk of several steps passes through and clears the buffer.
    """

    stage: ClassVar[str] = "input"  # its place among the chunk modules (CHUNK_STAGES)

    window_ms: float = 500.0
    _points: list[TimedPoint] = field(default_factory=list, init=False, repr=False)
    _anchor_ms: float | None = field(default=None, init=False, repr=False)
    _timeline_ms: float = field(default=0.0, init=False, repr=False)
    _warned: bool = field(default=False, init=False, repr=False)

    def __post_init__(self) -> None:
        if not self.window_ms > 0:
            raise ValueError("LiveBuffer needs a positive window_ms")

    def on_chunk(self, chunk: PendingChunk, context: ModuleContext) -> PendingChunk:
        if len(chunk.steps) != 1:
            if self.clear() and not self._warned:
                self._warned = True
                logger.warning(
                    "A chunk replaced the buffered live targets; they go out alone until the "
                    "buffer refills. Prefer one target form per motion group."
                )
            return chunk
        # A live target is timed by observed_at_ms (the jogger's loop time); a lead chunk and
        # one without it by the session clock now (contracts T13).
        produced = chunk.observed_at_ms if not chunk.lead else None
        produced_ms = float(produced if produced is not None else context.now_ms)
        if self._anchor_ms is None:
            self._anchor_ms = produced_ms
        self._timeline_ms = trajectory_elapsed_ms(produced_ms, self._anchor_ms, self._timeline_ms)
        replay = self.push(self._timeline_ms / 1000.0, chunk.steps[0], step_ms=context.step_ms)
        if replay is None:
            return chunk  # too little history yet: the target goes out alone
        first_ms = timeline_timestamp_ms(self._anchor_ms, replay.end_time_s, lead_ms=LIVE_LEAD_MS)
        return replace(
            chunk,
            steps=replay.steps,
            dt_ms=replay.step_ms,
            first_timestamp_ms=ServerMs(first_ms),
            timestamp_offset_steps=0,
            lead=False,
            observed_at_ms=None,
            extend_buffer=False,
            caller_step_count=len(replay.steps),
            # The newest target is the last step: its IO and hooks go with it.
            when_due=None if chunk.when_due is None else (len(replay.steps) - 1, chunk.when_due[1]),
        )

    def push(self, time_s: float, target: list[float], *, step_ms: float) -> Replay | None:
        """Hold ``target`` (produced at trajectory time ``time_s``) and return the horizon, or
        ``None`` while there is too little history to define a spacing."""
        append_and_prune(self._points, time_s, target, window_ms=self.window_ms)
        replay = horizon(self._points, single_step_dt_ms=step_ms)
        if replay is None:
            return None
        steps, grid_start_s, grid_step_ms = replay
        return Replay(
            steps=steps,
            step_ms=grid_step_ms,
            end_time_s=horizon_end_time_s(grid_start_s, self.window_ms),
        )

    def clear(self) -> int:
        """Forget the held targets; how many there were."""
        held = len(self._points)
        self._points.clear()
        return held
