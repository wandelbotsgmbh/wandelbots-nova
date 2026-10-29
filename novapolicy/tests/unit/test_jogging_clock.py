"""Tests for JoggingTimeClock — pins server-time tracking behavior.

The clock keys off ``jogger_session_timestamp_ms`` from the state stream:
  * while that field stays 0 (never wired up / not yet advancing) the clock
    never syncs and falls back to wall-clock elapsed;
  * server milliseconds are real milliseconds — no clock-rate scaling;
  * the latest server sample is extrapolated to estimate server "now", and that
    extrapolation is deliberately uncapped — a stalled stream warns and keeps
    advancing rather than freezing, because capping it deadlocks startup.
"""

from __future__ import annotations

from datetime import UTC, datetime
import time

from novapolicy.jogging.clock import (
    _MAX_DELIVERY_DELAY_MS,
    _STATE_CLOCK_CALIBRATION_SAMPLES,
    JoggingTimeClock,
)


def test_unsynced_clock_reports_wall_clock_elapsed(manual_time):
    """Before any server timestamp arrives, 'now' is plain wall-clock elapsed."""
    clock = JoggingTimeClock()
    clock.start()
    manual_time.advance(0.5)
    assert clock.synced is False
    assert clock.estimated_server_timestamp_ms in {499, 500}


def test_advancing_timestamp_syncs_the_clock(manual_time):
    clock = JoggingTimeClock()
    clock.start()
    clock.update(1_000)
    manual_time.advance(0.1)
    clock.update(1_200)
    assert clock.synced is True
    assert clock.last_server_timestamp_ms == 1_200


def test_server_time_is_never_rescaled_by_a_derived_rate(manual_time):
    """A server timer running fast must NOT stretch the estimated timeline.

    Deriving a server/client rate ratio and scaling by it is the tempting
    alternative. On a real UR10e that ratio settles near 1.09 and slows all motion
    by the same proportion, where an unscaled timeline runs at exactly the
    commanded speed.
    """
    clock = JoggingTimeClock(max_lookahead_ms=500.0)
    clock.start()
    clock.update(1_000)
    manual_time.advance(0.1)
    clock.update(1_200)  # server advanced 200ms in 100ms of wall time

    manual_time.advance(0.1)
    # Extrapolation adds real elapsed milliseconds, not ratio-scaled ones.
    assert clock.estimated_server_timestamp_ms in {1_299, 1_300}


def test_estimated_server_now_extrapolates_an_aged_state_sample(manual_time):
    clock = JoggingTimeClock(max_lookahead_ms=500.0)
    clock.update(500)
    manual_time.advance(0.1)

    assert clock.estimated_server_timestamp_ms in {599, 600}


def test_delayed_state_uses_generation_time_not_receipt_time(manual_time):
    """A late-delivered state must not drag estimated server time backwards.

    Feeds packets the way the state stream does — every one through
    ``observe_state_timestamp`` first, then ``update``. The skew estimate lives
    in the former, and ``update`` cannot backdate anything without it.
    """
    clock = JoggingTimeClock(max_lookahead_ms=500.0)
    first = datetime.fromtimestamp(manual_time.now, UTC)
    clock.observe_state_timestamp(first)
    clock.update(1_000, first)

    # The next state was generated after 100ms but delivered after 200ms.
    generated_at = manual_time.now + 0.1
    manual_time.advance(0.2)
    before = clock.estimated_server_timestamp_ms
    second = datetime.fromtimestamp(generated_at, UTC)
    clock.observe_state_timestamp(second)
    clock.update(1_100, second)

    assert before in {1_199, 1_200}
    assert clock.estimated_server_timestamp_ms in {1_199, 1_200}


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


def test_unsynced_acknowledged_elapsed_falls_back_to_wall_clock(manual_time):
    """Before any server timestamp arrives, 'now' is just wall-clock elapsed."""
    clock = JoggingTimeClock()
    clock.start()
    manual_time.advance(0.2)
    assert clock.synced is False
    assert clock.acknowledged_elapsed_ms in {199, 200}


def test_acknowledged_elapsed_tracks_wall_clock_on_a_healthy_link(manual_time):
    """With fresh server acks, acknowledged time follows the latest server sample."""
    clock = JoggingTimeClock(max_lookahead_ms=300.0)
    clock.start()
    manual_time.advance(0.5)
    clock.update(500)
    assert clock.acknowledged_elapsed_ms == 500


def test_estimated_time_keeps_advancing_through_a_stall(manual_time):
    """A stalled stream must not stop the estimate.

    Bounding it to the newest sample deadlocks the session: the jogger clock
    only advances while waypoints execute, so a robot that has not started yet
    could never be commanded to start.
    """
    clock = JoggingTimeClock(max_lookahead_ms=250.0)
    clock.start()
    manual_time.advance(0.3)
    clock.update(300)
    manual_time.advance(5.0)

    assert clock.acknowledged_elapsed_ms == 5_300


def test_stall_logs_a_warning_once_and_recovers(caplog, manual_time):
    """A frozen server timer warns exactly once, then logs recovery on resume."""
    import logging

    clock = JoggingTimeClock(max_lookahead_ms=250.0)
    clock.start()
    manual_time.advance(0.3)
    clock.update(300)
    manual_time.advance(5.0)

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
    manual_time.advance(5.0)
    with caplog.at_level(logging.WARNING, logger="novapolicy.jogging.clock"):
        _ = clock.acknowledged_elapsed_ms
    assert any("stalled" in r.message for r in caplog.records)


def test_healthy_link_does_not_warn(caplog, manual_time):
    """Fresh acks within the lookahead window must never trip the stall warning."""
    import logging

    clock = JoggingTimeClock(max_lookahead_ms=300.0)
    clock.start()
    manual_time.advance(0.5)
    clock.update(500)
    with caplog.at_level(logging.WARNING, logger="novapolicy.jogging.clock"):
        for _ in range(10):
            _ = clock.acknowledged_elapsed_ms
    assert not [r for r in caplog.records if "stalled" in r.message]


# ===========================================================================
# Property-based invariants for the clock's scaling + clamp behaviour.
# ===========================================================================

from hypothesis import (  # ruff: ignore[module-import-not-at-top-of-file]
    given,
    settings,
    strategies as st,
)

_TS = st.integers(min_value=0, max_value=1_000_000)


@given(
    readings=st.lists(st.integers(min_value=-1000, max_value=1_000_000), min_size=0, max_size=20)
)
@settings(max_examples=200, deadline=None)
def test_last_server_timestamp_never_goes_backwards(readings):
    """Out-of-order or repeated samples must not rewind the acknowledged clock."""
    clock = JoggingTimeClock()
    clock.start()
    previous = 0
    for ts in readings:
        clock.update(ts)
        assert clock.last_server_timestamp_ms >= previous
        previous = clock.last_server_timestamp_ms


@given(readings=st.lists(st.integers(min_value=-1000, max_value=0), min_size=1, max_size=20))
@settings(max_examples=100, deadline=None)
def test_non_positive_readings_never_sync_the_clock(readings):
    """A jogger clock stuck at <= 0 must never flip to synced."""
    clock = JoggingTimeClock()
    clock.start()
    for ts in readings:
        clock.update(ts)
    assert clock.synced is False


def test_a_state_packet_counts_as_one_calibration_sample(manual_time) -> None:
    """One packet, one sample — however many methods the caller routes it through.

    The session observes every packet and additionally calls ``update`` for the
    ones carrying a jogger timestamp. Counting the sample in both places declares
    the skew estimate frozen after half the packets it is meant to see, which is
    what the sample count exists to prevent: a single late first packet then
    biases every waypoint timestamp in the session.
    """
    clock = JoggingTimeClock()
    clock.start()

    for i in range(1, 20):
        stamp = datetime.fromtimestamp(manual_time.now, UTC)
        clock.observe_state_timestamp(stamp)
        clock.update(i * 10, stamp)
        manual_time.advance(0.01)
        assert clock.state_clock_calibrated is False, i

    stamp = datetime.fromtimestamp(manual_time.now, UTC)
    clock.observe_state_timestamp(stamp)
    clock.update(200, stamp)

    assert clock.state_clock_calibrated is True


def test_sample_wall_is_none_before_the_first_server_timestamp() -> None:
    """Never hand out the zero-initialised instant.

    Zero in the monotonic domain is machine boot, so using the raw field as a
    timestamp stamps entries roughly a week before the session started.
    """
    clock = JoggingTimeClock()

    assert clock.last_sample_wall is None

    clock.update(8, None)

    wall = clock.last_sample_wall
    assert wall is not None
    assert abs(wall - time.monotonic()) < 1.0


def test_a_slow_state_rate_still_finishes_calibrating(manual_time) -> None:
    """Calibration must not be able to outlast the readiness timeout.

    The sample count is the real criterion, but the rate those samples arrive at
    belongs to the caller (``WaypointConfig.state_rate_ms``). At 500ms per packet
    twenty of them take ten seconds — ``wait_ready``'s whole default budget — so a
    healthy session would fail to start. Past the window the estimate is frozen
    with whatever it has: a rougher skew estimate, not a broken one.
    """
    clock = JoggingTimeClock()
    clock.start()

    clock.observe_state_timestamp(datetime.fromtimestamp(manual_time.now, UTC))
    assert clock.state_clock_calibrated is False  # one sample is not enough on its own

    manual_time.advance(0.5)
    assert clock.state_clock_calibrated is False  # ...and neither is half the window

    manual_time.advance(0.5)
    assert clock.state_clock_calibrated is True


def test_calibration_needs_at_least_one_sample_however_long_it_waits(manual_time) -> None:
    """The window is a fallback for slow packets, not for absent ones."""
    clock = JoggingTimeClock()
    clock.start()
    manual_time.advance(60.0)
    assert clock.state_clock_calibrated is False


def test_a_wall_clock_step_cannot_shunt_the_estimate_into_the_future(manual_time) -> None:
    """Backdating from a bad state timestamp is bounded.

    The delivery delay is a difference of ``time.time()`` readings against a skew
    baseline frozen at calibration, so it is only as trustworthy as the wall
    clock. An NTP step — or one oddly stamped packet — inflates it, and it is
    subtracted from the monotonic reference, pushing estimated server time
    forward. The jogger's trajectory clock never runs backwards, so it latches
    that estimate and the robot holds still until the real server clock catches
    up. Bounding the correction bounds the stall.
    """
    clock = JoggingTimeClock()
    clock.start()
    for i in range(_STATE_CLOCK_CALIBRATION_SAMPLES):
        stamp = datetime.fromtimestamp(manual_time.now, UTC)
        clock.observe_state_timestamp(stamp)
        clock.update(1_000 + i, stamp)
    assert clock.state_clock_calibrated is True

    # A packet claiming to have been generated 30 seconds ago.
    clock.update(2_000, datetime.fromtimestamp(manual_time.now - 30.0, UTC))

    # The reference is backdated by at most the clamp, so "now" is the jogger
    # timestamp plus that much. Unbounded, it would read ~32_000 and hold the
    # trajectory clock there for thirty seconds.
    capped = 2_000 + int(_MAX_DELIVERY_DELAY_MS)
    assert clock.estimated_server_timestamp_ms in {capped - 1, capped}
