"""Tests for JoggingTimeClock — pins the server-time-sync heuristic behavior.

The clock keys off ``jogger_session_timestamp_ms`` from the state stream:
  * while that field stays 0 (never wired up / not yet advancing) the clock
    never syncs and ``speed_ratio`` stays 1.0 (scaling is a no-op);
  * clock rate is measured from deltas between server and monotonic-clock
    samples, without assuming that both clocks started together;
  * the latest server sample is extrapolated to estimate server "now".
"""

from __future__ import annotations

import time

from novapolicy.jogging.clock import JoggingTimeClock


def test_unsynced_clock_is_identity():
    """Before any server timestamp arrives, scaling is the identity transform."""
    clock = JoggingTimeClock()
    assert clock.synced is False
    assert clock.speed_ratio == 1.0
    assert clock.scale_timestamp(500) == 500
    assert clock.scale_dt(33.0) == 33.0


def test_zero_timestamp_never_syncs():
    """A jogger_session_timestamp_ms stuck at 0 must not flip the clock to synced."""
    clock = JoggingTimeClock()
    clock.start()
    for _ in range(5):
        clock.update(0)
    assert clock.synced is False
    assert clock.speed_ratio == 1.0


def test_advancing_timestamp_syncs_and_scales_from_clock_deltas():
    """Clock rate comes from sample deltas, independent of the clocks' origins."""
    clock = JoggingTimeClock()
    clock.start()
    clock.update(1_000)
    # Simulate 100 ms of wall time between samples while the server advances 200 ms.
    clock._rate_reference_wall = time.monotonic() - 0.100
    clock.update(1_200)
    assert clock.synced is True
    assert clock.speed_ratio >= 1.5
    assert clock.scale_timestamp(100) == int(100 * clock.speed_ratio)
    assert clock.scale_dt(50.0) == 50.0 * clock.speed_ratio


def test_ratio_clamped_to_at_least_one():
    """The server is never slower than wall-clock; ratio is clamped >= 1.0."""
    clock = JoggingTimeClock()
    clock.start()
    clock.update(1_000)
    # Server advances 50 ms over 200 ms of wall time → raw ratio 0.25.
    clock._rate_reference_wall = time.monotonic() - 0.200
    clock.update(1_050)
    assert clock.speed_ratio == 1.0


def test_estimated_server_now_extrapolates_an_aged_state_sample():
    clock = JoggingTimeClock(speed_ratio=2.0, synced=True, max_lookahead_ms=500.0)
    clock._last_server_ts_ms = 500
    clock._last_server_wall = time.monotonic() - 0.100

    assert 680 <= clock.estimated_server_timestamp_ms <= 730


def test_extract_from_state_walks_execute_details():
    """extract_from_state reads execute.details.jogger_session_timestamp_ms."""

    class _Details:
        jogger_session_timestamp_ms = 123

    class _Execute:
        details = _Details()

    class _State:
        execute = _Execute()

    assert JoggingTimeClock.extract_from_state(_State()) == 123


def test_extract_from_state_handles_missing_fields():
    """Missing execute/details yields None rather than raising."""

    class _Empty:
        execute = None

    assert JoggingTimeClock.extract_from_state(_Empty()) is None
    assert JoggingTimeClock.extract_from_state(object()) is None


# ===========================================================================
# acknowledged_elapsed_ms — "now" must follow the server, not a free-running
# wall clock, so a stalled connection can't make targets race ahead.
# ===========================================================================


def test_unsynced_acknowledged_elapsed_falls_back_to_wall_clock():
    """Before any server timestamp arrives, 'now' is just wall-clock elapsed."""
    clock = JoggingTimeClock()
    clock.start()
    clock._client_start_time = time.monotonic() - 0.200  # ~200ms elapsed
    assert clock.synced is False
    assert 150 <= clock.acknowledged_elapsed_ms <= 260


def test_acknowledged_elapsed_tracks_wall_clock_on_a_healthy_link():
    """With fresh server acks, 'now' follows wall-clock to within a tick — the
    capping is invisible on a healthy connection (no behaviour change)."""
    clock = JoggingTimeClock(max_lookahead_ms=300.0)
    clock.start()
    clock._client_start_time = time.monotonic() - 0.500  # ~500ms wall elapsed
    clock.update(500)  # server agrees it is 500ms in, just now
    # ratio ~= 1.0; acknowledged 'now' ≈ 500ms with only a sub-tick of drift.
    assert 480 <= clock.acknowledged_elapsed_ms <= 540


def test_acknowledged_elapsed_freezes_when_the_stream_stalls():
    """If no new server timestamp arrives, 'now' may drift at most one lookahead
    window past the last ack, then freezes — so it can't run away while the
    connection is down (the fix for catch-up jumps)."""
    clock = JoggingTimeClock(max_lookahead_ms=250.0)
    clock.start()
    clock._client_start_time = time.monotonic() - 0.300
    clock.update(300)  # last ack: server at 300ms, ratio ~= 1.0
    # Simulate the stream having stalled 5 seconds ago.
    clock._last_server_wall = time.monotonic() - 5.0
    now = clock.acknowledged_elapsed_ms
    # 300ms ack + at most a 250ms lookahead, NOT 300 + 5000.
    assert now <= 300 / clock.speed_ratio + 250 + 5
    assert now < 600


def test_acknowledged_elapsed_resumes_from_the_new_ack_after_a_stall():
    """Once a fresh server timestamp lands, 'now' snaps to acknowledged progress
    rather than to the wall time that elapsed during the stall."""
    clock = JoggingTimeClock(max_lookahead_ms=250.0)
    clock.start()
    clock._client_start_time = time.monotonic() - 0.300
    clock.update(300)
    # ... a long stall ... then a fresh ack arrives saying the server only
    # advanced to 360ms (it was paused too).
    clock._last_server_ts_ms = 360
    clock._last_server_wall = time.monotonic()
    assert 350 <= clock.acknowledged_elapsed_ms <= 380


def test_stall_logs_a_warning_once_and_recovers(caplog):
    """A frozen server timer warns exactly once, then logs recovery on resume."""
    import logging

    clock = JoggingTimeClock(max_lookahead_ms=250.0)
    clock.start()
    clock._client_start_time = time.monotonic() - 0.300
    clock.update(300)
    # Stream stalled well beyond the lookahead window.
    clock._last_server_wall = time.monotonic() - 5.0

    with caplog.at_level(logging.WARNING, logger="novapolicy.jogging.clock"):
        # Read "now" many times — the operator should see ONE warning, not a flood.
        for _ in range(10):
            _ = clock.acknowledged_elapsed_ms
    stall_warnings = [r for r in caplog.records if "stalled" in r.message]
    assert len(stall_warnings) == 1

    # A fresh server timestamp resumes the timeline and logs recovery once.
    caplog.clear()
    with caplog.at_level(logging.INFO, logger="novapolicy.jogging.clock"):
        clock.update(360)
    assert any("recovered" in r.message for r in caplog.records)

    # After recovery, a renewed stall warns again (edge-triggered, not latched).
    caplog.clear()
    clock._last_server_wall = time.monotonic() - 5.0
    with caplog.at_level(logging.WARNING, logger="novapolicy.jogging.clock"):
        _ = clock.acknowledged_elapsed_ms
    assert any("stalled" in r.message for r in caplog.records)


def test_healthy_link_does_not_warn(caplog):
    """Fresh acks within the lookahead window must never trip the stall warning."""
    import logging

    clock = JoggingTimeClock(max_lookahead_ms=300.0)
    clock.start()
    clock._client_start_time = time.monotonic() - 0.500
    clock.update(500)
    with caplog.at_level(logging.WARNING, logger="novapolicy.jogging.clock"):
        for _ in range(10):
            _ = clock.acknowledged_elapsed_ms
    assert not [r for r in caplog.records if "stalled" in r.message]


# ===========================================================================
# Property-based invariants for the clock's scaling + clamp behaviour.
# ===========================================================================

from hypothesis import (  # noqa: E402
    given,
    settings,
    strategies as st,
)

_RATIO = st.floats(min_value=1.0, max_value=50.0, allow_nan=False, allow_infinity=False)
_TS = st.integers(min_value=0, max_value=1_000_000)


@given(ratio=_RATIO, t=_TS)
@settings(max_examples=200, deadline=None)
def test_scaling_with_any_ratio_never_shrinks_a_timestamp(ratio, t):
    """Since the server is never slower than wall-clock (ratio >= 1), scaling
    a non-negative timestamp/dt can only grow it, and equals int(t * ratio)."""
    clock = JoggingTimeClock(speed_ratio=ratio)
    assert clock.scale_timestamp(t) == int(t * ratio)
    assert clock.scale_timestamp(t) >= t
    assert clock.scale_dt(float(t)) == t * ratio
    assert clock.scale_dt(float(t)) >= t


@given(ratio=_RATIO, a=_TS, b=_TS)
@settings(max_examples=200, deadline=None)
def test_scaling_is_monotonic_in_the_input(ratio, a, b):
    """Ordering is preserved: a <= b implies scale(a) <= scale(b)."""
    clock = JoggingTimeClock(speed_ratio=ratio)
    lo, hi = sorted((a, b))
    assert clock.scale_timestamp(lo) <= clock.scale_timestamp(hi)


@given(
    readings=st.lists(st.integers(min_value=-1000, max_value=1_000_000), min_size=0, max_size=20)
)
@settings(max_examples=200, deadline=None)
def test_speed_ratio_stays_at_least_one_through_any_reading_sequence(readings):
    """Whatever the state stream reports, speed_ratio is clamped >= 1.0."""
    clock = JoggingTimeClock()
    clock.start()
    for ts in readings:
        clock.update(ts)
        assert clock.speed_ratio >= 1.0


@given(readings=st.lists(st.integers(min_value=-1000, max_value=0), min_size=1, max_size=20))
@settings(max_examples=100, deadline=None)
def test_non_positive_readings_never_sync_the_clock(readings):
    """A jogger clock stuck at <= 0 must never flip to synced or scale != 1.0."""
    clock = JoggingTimeClock()
    clock.start()
    for ts in readings:
        clock.update(ts)
    assert clock.synced is False
    assert clock.speed_ratio == 1.0
