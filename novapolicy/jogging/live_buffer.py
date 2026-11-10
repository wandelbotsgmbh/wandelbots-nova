"""Rolling buffer of measured live targets, replayed as a continuous horizon.

A live target says only where the target is **now**, and a lone waypoint is a
*terminal* target the server decelerates to a standstill at. So a caller that
replaces one target per tick makes the robot stop and restart continuously. This
buffer removes that: it holds the last ``window_ms`` of targets and replays them
as a waypoint horizon, so the server always has somewhere to be going next.

Nothing here is invented. Every waypoint replayed is a target that was really
measured, handed over late rather than extrapolated forward, which is why the
cost is latency: the robot trails the live target by about the window.

Shared by the joggers' live-target path and :class:`~novapolicy.executor.PolicyExecutor`'s
buffered mode, so both get the same timing behaviour. The three properties below
are the whole point, and each was measured rather than reasoned:

* the replay grid is phase-locked to absolute multiples of the step, not to
  whichever sample happens to be oldest,
* the trajectory-time to server-time mapping uses a **constant** lead,
* resampling interpolates each sample at the instant it was really taken.

Together they mean a given measured target maps to the same absolute timestamp
however often the window is re-sent. Re-deriving the anchor from server "now" at
send time instead moves previously commanded points to new absolute times, and
the robot executes each shift as a lurch.
"""

from __future__ import annotations

import math
from typing import TypeAlias

TimedPoint: TypeAlias = tuple[float, list[float]]
"""One measured target and the trajectory time it was produced at."""

LIVE_LEAD_MS = 100.0
"""How far ahead of current motion a replacement horizon starts.

This is the room the controller has to blend onto the replacement, and the
buffer is re-sent every tick, so the controller performs that join constantly.
Give it too little and it occasionally cannot do so smoothly, which comes out as
a brief stall and catch-up. Every waypoint in the buffer was measured rather
than projected, so nothing here trades accuracy against distance. On the
reference rig, evenness improved monotonically from 30ms to 100ms, where stalls
disappeared; this is not about running out of waypoints, so lengthening the
horizon instead does not help.
"""

TIMELINE_GRID_MS = 10.0
"""Quantisation of the session timeline.

Anchoring every push at a multiple of this keeps successive pushes on ONE
absolute grid; without it each replacement lands at a new phase, which on the
reference rig was the difference between grossly uneven motion and none
measurable at all.
"""

MIN_BUFFER_SAMPLES = 2
"""A rolling buffer needs at least two samples to define a spacing."""

MAX_LIVE_SAMPLES = 256
"""Hard cap on retained samples, on top of the time window.

The window is measured against a trajectory clock that holds at zero until the
robot reports it is executing motion, so while a caller pushes targets into a
session that has not started (or whose state stream has stalled), the oldest
sample's age is always 0 and nothing is ever evicted. The cap is far above what
any real rate needs: a 100Hz caller fills a 500ms buffer with ~50 samples.
"""


def trajectory_elapsed_ms(now_ms: float, anchor_ms: float, previous_ms: float) -> float:
    """Trajectory time on the **server** clock: quantised, and never backwards.

    ``now_ms`` must come from the server jogger clock, not a monotonic clock.
    The robot executes against the jogger session timer and wall time drifts
    from it; driving this from monotonic time measured far worse in two separate
    experiments (10mm+ tracking error and repeated stalls), even though it is the
    smoother of the two.

    Never runs backwards: server "now" is extrapolated between state samples, so
    a late sample can resolve below an earlier estimate, and the content sent is
    a function of this value — a backward step commands the robot to reverse.
    Forward jumps are deliberately *not* limited: they are the clock correcting
    after a late read of the state stream, and rate-limiting them measured far
    worse, with the timeline falling behind until every waypoint landed in the
    past and the robot starved.
    """
    elapsed_ms = max(0.0, now_ms - anchor_ms)
    if TIMELINE_GRID_MS > 0:
        elapsed_ms = math.floor(elapsed_ms / TIMELINE_GRID_MS) * TIMELINE_GRID_MS
    return max(previous_ms, elapsed_ms)


def resample_evenly(
    samples: list[TimedPoint], step_s: float, *, start: float | None = None
) -> list[list[float]]:
    """Put irregularly-timed samples on an evenly spaced grid.

    Linear interpolation at each grid instant, using the time every sample was
    actually taken. The values stay the caller's own — only the instants they
    are read at change — so this resamples a recording rather than predicting
    anything.

    ``start`` pins the grid to an absolute phase; without it the grid begins at
    the oldest sample.
    """
    start = samples[0][0] if start is None else start
    end = samples[-1][0]
    if end < start:
        return []
    # Round the span UP so the grid reaches the newest sample. Truncating drops
    # the last partial interval, and that interval is the leading edge of the
    # horizon — the freshest thing the buffer knows. A grid point landing past
    # the newest sample holds its value (``fraction`` is clamped below), which
    # states the target has stopped rather than guessing where it went next.
    #
    # The epsilon absorbs float division: a span of exactly two steps comes out
    # as 1.9999999999999996 often enough to matter, and truncating that loses a
    # whole waypoint.
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
    """Add a target and drop what has aged out of the window.

    A sample is dropped once it is older than ``window_ms``, so the window's
    length is bounded by time rather than by call rate.
    """
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

    ``None`` means there is not enough history yet to define a spacing; the
    caller should send the bare target instead rather than wait for the window
    to fill, which would otherwise never move at all.
    """
    span_s = buffer[-1][0] - buffer[0][0]
    if len(buffer) < MIN_BUFFER_SAMPLES or span_s <= 0:
        return None

    # Waypoints are laid out at a uniform spacing, so the recorded samples have
    # to be put on a uniform grid *by interpolating at their own timestamps*.
    # Handing them over as-is with an averaged dt replays each one at the wrong
    # moment -- the caller's ticks jitter, and a quantised timeline collapses
    # some of them -- which time-warps the trajectory and injects velocity noise
    # into what is otherwise exact recorded data.
    #
    # The grid is ONE absolute grid, not one starting wherever the oldest
    # surviving sample happens to sit: which sample that is changes with every
    # prune, so anchoring to it lands the same recorded moment on a slightly
    # different timestamp in each push -- the same re-anchoring that turns an
    # otherwise clean motion into a vibrating one.
    # The spacing the controller executes best is a per-controller property, so
    # it comes from config, but never coarser than half the window, so a short
    # buffer is still represented by more than its endpoints.
    step_dt_s = min(single_step_dt_ms / 1000.0, span_s / 2.0)
    grid_start = math.ceil(buffer[0][0] / step_dt_s) * step_dt_s
    steps = resample_evenly(buffer, step_dt_s, start=grid_start)
    if not steps:
        return None
    return steps, grid_start, step_dt_s * 1000.0


def timeline_timestamp_ms(
    anchor_ms: float, trajectory_time_s: float, *, lead_ms: float = LIVE_LEAD_MS
) -> int:
    """Absolute server timestamp for content generated at ``trajectory_time_s``.

    One fixed timeline: content generated at trajectory time ``T`` always maps to
    ``anchor + T + lead``, whenever it is sent. The lead is deliberately
    **constant** — anything time-varying in this mapping (an adaptive lead, a
    drifting clock offset) moves previously commanded points to new absolute
    times, and the robot executes each such shift as a lurch.
    """
    return int(anchor_ms + trajectory_time_s * 1000.0 + lead_ms)


def horizon_end_time_s(grid_start_s: float, window_ms: float) -> float:
    """Trajectory time the replayed window reaches up to, and no further.

    The window is the whole horizon: it ends at "now" because nothing beyond it
    has been measured yet. That is why ``window_ms`` sets the horizon the server
    profiles against, and why it has to be a few hundred milliseconds — the
    server caps its speed at whatever it can still brake to a stop within.
    """
    return grid_start_s + window_ms / 1000.0
