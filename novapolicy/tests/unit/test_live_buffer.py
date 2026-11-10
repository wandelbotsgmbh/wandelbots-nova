"""The rolling replay buffer, and the property the whole mode exists for.

No robot and no broker: the buffer is pure functions over timed samples, so the
timing behaviour that matters can be asserted directly.
"""

from __future__ import annotations

import math

import pytest

from novapolicy.jogging.live_buffer import (
    LIVE_LEAD_MS,
    MAX_LIVE_SAMPLES,
    MIN_BUFFER_SAMPLES,
    append_and_prune,
    horizon,
    horizon_end_time_s,
    resample_evenly,
    timeline_timestamp_ms,
)

WINDOW_MS = 500.0
STEP_DT_MS = 50.0


def _fill(rate_hz: float, seconds: float, *, start: float = 0.0, window_ms: float = WINDOW_MS):
    """A buffer fed at a steady rate with a ramp, one value per axis."""
    buffer: list[tuple[float, list[float]]] = []
    n = int(rate_hz * seconds)
    for i in range(n):
        t = start + i / rate_hz
        append_and_prune(buffer, t, [t, -t], window_ms=window_ms)
    return buffer


# -- the point of the mode --------------------------------------------------


def test_a_sample_keeps_its_timestamp_across_pushes() -> None:
    """The property the buffered mode exists for.

    A target measured at trajectory time T must map to ONE absolute timestamp,
    however many later pushes replay it. If it moves, the robot executes the
    shift as a lurch, which is the vibration the chunk path produces by
    re-deriving its anchor from server "now" on every send.
    """
    anchor_ms = 12_345.0
    seen: dict[float, list[int]] = {}
    for pushes in range(1, 8):
        buffer = _fill(50.0, 0.2 + 0.02 * pushes)
        replay = horizon(buffer, single_step_dt_ms=STEP_DT_MS)
        assert replay is not None
        steps, grid_start, step_dt_ms = replay
        base = timeline_timestamp_ms(
            anchor_ms, horizon_end_time_s(grid_start, WINDOW_MS), lead_ms=LIVE_LEAD_MS
        )
        for index in range(len(steps)):
            moment = grid_start + index * step_dt_ms / 1000.0
            seen.setdefault(round(moment, 6), []).append(base + int(index * step_dt_ms))

    repeated = {m: ts for m, ts in seen.items() if len(ts) > 1}
    assert repeated, "the test needs moments that appear in more than one push"
    for moment, stamps in repeated.items():
        assert len(set(stamps)) == 1, f"trajectory time {moment} moved between pushes: {stamps}"


def test_the_grid_is_phase_locked_not_anchored_to_the_oldest_sample() -> None:
    """Pruning changes which sample is oldest; the grid must not follow it.

    Anchoring on the oldest surviving sample lands the same recorded moment on a
    different timestamp after every prune, which is the same re-anchoring fault
    by another route.
    """
    step_s = STEP_DT_MS / 1000.0
    for offset in (0.0, 0.003, 0.017, 0.049):
        buffer = _fill(50.0, 0.4, start=offset)
        replay = horizon(buffer, single_step_dt_ms=STEP_DT_MS)
        assert replay is not None
        _, grid_start, _ = replay
        assert grid_start / step_s == pytest.approx(round(grid_start / step_s), abs=1e-9)


def test_the_lead_is_constant() -> None:
    """A time-varying lead moves already-commanded points; keep it fixed."""
    a = timeline_timestamp_ms(1000.0, 0.5)
    b = timeline_timestamp_ms(1000.0, 1.5)
    assert b - a == 1000
    assert timeline_timestamp_ms(1000.0, 0.0) == int(1000.0 + LIVE_LEAD_MS)


# -- buffer maintenance -----------------------------------------------------


def test_samples_age_out_of_the_window() -> None:
    buffer = _fill(50.0, 3.0)
    span = buffer[-1][0] - buffer[0][0]
    assert span <= WINDOW_MS / 1000.0 + 1 / 50.0


def test_the_window_actually_covers_its_duration() -> None:
    """The window is the horizon, so it has to reach back the full duration."""
    buffer = _fill(50.0, 3.0)
    span_ms = (buffer[-1][0] - buffer[0][0]) * 1000.0
    assert span_ms > WINDOW_MS * 0.8


def test_the_sample_cap_holds_before_the_timeline_starts() -> None:
    """A clock stuck at zero evicts nothing by age, so the cap has to bite."""
    buffer: list[tuple[float, list[float]]] = []
    for _ in range(MAX_LIVE_SAMPLES * 4):
        append_and_prune(buffer, 0.0, [1.0, 2.0], window_ms=WINDOW_MS)
    assert len(buffer) <= MAX_LIVE_SAMPLES


def test_a_buffer_that_cannot_define_a_spacing_has_no_horizon() -> None:
    buffer: list[tuple[float, list[float]]] = []
    append_and_prune(buffer, 0.0, [0.0, 0.0], window_ms=WINDOW_MS)
    assert horizon(buffer, single_step_dt_ms=STEP_DT_MS) is None
    append_and_prune(buffer, 0.0, [1.0, 1.0], window_ms=WINDOW_MS)
    assert horizon(buffer, single_step_dt_ms=STEP_DT_MS) is None, "zero span is not a spacing"


def test_a_short_buffer_is_represented_by_more_than_its_endpoints() -> None:
    """Spacing is capped at half the span, so a filling window still has shape."""
    buffer = _fill(200.0, 0.06)
    replay = horizon(buffer, single_step_dt_ms=STEP_DT_MS)
    assert replay is not None
    steps, _, step_dt_ms = replay
    assert step_dt_ms <= STEP_DT_MS
    assert len(steps) >= MIN_BUFFER_SAMPLES + 1


# -- replay fidelity --------------------------------------------------------


def test_nothing_replayed_is_beyond_the_newest_measured_target() -> None:
    """Nothing is extrapolated: the horizon ends at what was actually measured."""
    buffer = _fill(50.0, 1.0)
    newest = buffer[-1][1][0]
    replay = horizon(buffer, single_step_dt_ms=STEP_DT_MS)
    assert replay is not None
    steps, _, _ = replay
    assert max(step[0] for step in steps) <= newest + 1e-9


def test_resampling_reads_each_sample_at_its_own_instant() -> None:
    """Irregular ticks must not time-warp the replay."""
    samples = [(0.0, [0.0]), (0.013, [13.0]), (0.031, [31.0]), (0.05, [50.0])]
    grid = resample_evenly(samples, 0.025, start=0.0)
    assert grid[0][0] == pytest.approx(0.0)
    assert grid[1][0] == pytest.approx(25.0, abs=0.5)
    assert grid[2][0] == pytest.approx(50.0, abs=0.5)


def test_a_grid_point_past_the_newest_sample_holds_rather_than_guesses() -> None:
    samples = [(0.0, [0.0]), (0.01, [10.0])]
    grid = resample_evenly(samples, 0.05, start=0.0)
    assert grid[-1][0] == pytest.approx(10.0)


def test_the_horizon_reaches_the_newest_sample() -> None:
    """Truncating the span would drop the freshest thing the buffer knows."""
    samples = [(0.0, [0.0]), (0.04, [4.0]), (0.0999, [9.99])]
    grid = resample_evenly(samples, 0.05, start=0.0)
    assert len(grid) == math.ceil(0.0999 / 0.05 - 1e-9) + 1


def test_horizon_end_is_the_window_after_the_grid_start() -> None:
    assert horizon_end_time_s(1.0, 500.0) == pytest.approx(1.5)
