"""Tests for JoggingTimeClock — pins the server-time-sync heuristic behavior.

The clock keys off ``jogger_session_timestamp_ms`` from the state stream:
  * while that field stays 0 (never wired up / not yet advancing) the clock
    never syncs and ``speed_ratio`` stays 1.0 (scaling is a no-op);
  * once it advances, ``speed_ratio`` becomes an active multiplier applied to
    every outgoing waypoint timestamp.
"""

from __future__ import annotations

import time

from policy.jogging.clock import JoggingTimeClock


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


def test_advancing_timestamp_syncs_and_scales():
    """Once the server clock advances, speed_ratio becomes an active multiplier."""
    clock = JoggingTimeClock()
    clock.start()
    # Simulate ~100ms of client wall-clock having elapsed.
    clock._client_start_time = time.monotonic() - 0.100
    # Server reports it is 200ms into the session → ratio ~= 2.0.
    clock.update(200)
    assert clock.synced is True
    assert clock.speed_ratio >= 1.5
    # Scaling now multiplies outgoing timestamps by the ratio.
    assert clock.scale_timestamp(100) == int(100 * clock.speed_ratio)
    assert clock.scale_dt(50.0) == 50.0 * clock.speed_ratio


def test_ratio_clamped_to_at_least_one():
    """The server is never slower than wall-clock; ratio is clamped >= 1.0."""
    clock = JoggingTimeClock()
    clock.start()
    # Client elapsed 200ms but server only reports 50ms → raw ratio 0.25.
    clock._client_start_time = time.monotonic() - 0.200
    clock.update(50)
    assert clock.speed_ratio == 1.0


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
