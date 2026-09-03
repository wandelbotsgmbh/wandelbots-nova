"""Tests for make_waypoints_request — the documented jogging timestamp protocol.

This pure function turns raw steps and timing into a NOVA ``ActionChunkRequest``
carrying joint- or pose-flavoured waypoints. Every waypoint is ``base + i*dt``:
an exact raw NOVA timestamp may provide ``base``; otherwise server "now" is
resolved at call time. ``timestamp_offset_steps`` shifts that base by whole
intervals.
"""

from __future__ import annotations

import itertools

from hypothesis import given, settings, strategies as st

from nova import api
from novapolicy.jogging.clock import JoggingTimeClock
from novapolicy.jogging.waypoints import make_waypoints_request


def _joint_timestamps(req) -> list[int]:
    return [w.timestamp for w in req.waypoints]


def _joint_steps(req) -> list[list[float]]:
    return [list(w.waypoint.joints) for w in req.waypoints]


# ---------------------------------------------------------------------------
# Exact controller-timestamp mode
# ---------------------------------------------------------------------------


def test_absolute_mode_places_timestamps_starting_at_the_exact_base():
    """first_timestamp_ms=100, dt=10 -> [100, 110, 120]; steps preserved."""
    clock = JoggingTimeClock()
    steps = [[0.0] * 6, [0.1] * 6, [0.2] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        first_timestamp_ms=100,
    )
    assert isinstance(req, api.models.ActionChunkRequest)
    assert all(w.waypoint.kind == "JOINTS" for w in req.waypoints)
    assert _joint_timestamps(req) == [100, 110, 120]
    assert _joint_steps(req) == steps


def test_spacing_is_never_rescaled_by_a_derived_clock_rate():
    """dt is used verbatim: server milliseconds are real milliseconds.

    Deriving a server/client rate ratio and stretching dt by it is the tempting
    alternative. On a real UR10e that ratio settles near 1.09 and slows the robot
    by the same proportion, while an unscaled timeline runs at exactly the
    commanded speed.
    """
    clock = JoggingTimeClock()
    clock.update(5_000)
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        first_timestamp_ms=100,
    )
    assert _joint_timestamps(req) == [100, 110, 120]


def test_timestamp_offset_is_applied_in_the_server_clock_domain():
    clock = JoggingTimeClock()
    clock.update(5_000)
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[[0.0] * 6],
        effective_dt_ms=10.0,
        first_timestamp_ms=135,
        timestamp_offset_steps=1,
    )
    assert _joint_timestamps(req) == [145]


def test_explicit_server_spacing_overrides_the_requested_dt():
    clock = JoggingTimeClock()
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        first_timestamp_ms=135,
        server_dt_ms=20.0,
    )

    assert _joint_timestamps(req) == [135, 155, 175]


# ---------------------------------------------------------------------------
# Server "now" with a +1-step offset
# ---------------------------------------------------------------------------


def test_now_timestamp_one_step_ahead_starts_one_dt_after_now_not_at_now():
    """A fresh clock reports zero; +1 step produces [dt, 2dt, ...]."""
    clock = JoggingTimeClock()
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        timestamp_offset_steps=1,
    )
    assert _joint_timestamps(req) == [10, 20, 30]


def test_synced_now_timestamp_uses_server_clock_without_shared_origin_assumption(manual_time):
    """Server now is based on the latest server sample, not client elapsed time."""
    clock = JoggingTimeClock()
    clock.update(5_000)
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[[0.0] * 6, [0.0] * 6],
        effective_dt_ms=10.0,
        timestamp_offset_steps=1,
    )

    timestamps = _joint_timestamps(req)
    assert timestamps == [5_010, 5_020]


# ---------------------------------------------------------------------------
# Backdated server "now", resolved at yield time
# ---------------------------------------------------------------------------


def test_backdated_now_timestamp_is_clamped_to_zero():
    clock = JoggingTimeClock()
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        timestamp_offset_steps=-2,
    )
    assert _joint_timestamps(req) == [0, 10, 20]


def test_now_timestamp_is_read_at_yield_time_not_precomputed(manual_time):
    """Advancing the session clock shifts the whole progression."""
    clock = JoggingTimeClock()
    clock.start()
    manual_time.advance(0.5)
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[[0.0] * 6],
        effective_dt_ms=10.0,
        timestamp_offset_steps=-10,
    )
    assert req.waypoints[0].timestamp in {399, 400}


# ---------------------------------------------------------------------------
# Request-type dispatch + cartesian payload layout
# ---------------------------------------------------------------------------


def test_cartesian_mode_builds_pose_waypoints_splitting_position_and_orientation():
    """[x, y, z, rx, ry, rz] maps to NOVA position and rotation vector."""
    clock = JoggingTimeClock()
    steps = [[500.0, 200.0, 300.0, 0.1, 0.2, 0.3]]
    req = make_waypoints_request(
        clock,
        "cartesian",
        steps=steps,
        effective_dt_ms=10.0,
        first_timestamp_ms=0,
    )
    assert isinstance(req, api.models.ActionChunkRequest)
    waypoint = req.waypoints[0]
    assert waypoint.waypoint.kind == "POSE"
    assert waypoint.timestamp == 0
    pose = waypoint.waypoint.pose
    assert list(pose.position) == [500.0, 200.0, 300.0]
    assert list(pose.orientation) == [0.1, 0.2, 0.3]


def test_empty_steps_produce_no_waypoints():
    clock = JoggingTimeClock()
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[],
        effective_dt_ms=10.0,
        first_timestamp_ms=0,
    )
    assert req.waypoints == []


# ---------------------------------------------------------------------------
# Property: timestamps are an ordered arithmetic progression for any input
# ---------------------------------------------------------------------------

_DT = st.floats(min_value=1.0, max_value=200.0, allow_nan=False, allow_infinity=False)
_START = st.integers(min_value=0, max_value=100_000)
_N = st.integers(min_value=1, max_value=16)


@given(dt=_DT, start=_START, n=_N)
@settings(max_examples=200, deadline=None)
def test_absolute_timestamps_are_a_nondecreasing_progression_from_the_base(dt, start, n):
    """Exact server timestamps remain the base and never go backwards."""
    clock = JoggingTimeClock()
    steps = [[0.0] * 6 for _ in range(n)]
    timestamps = _joint_timestamps(
        make_waypoints_request(
            clock,
            "joint",
            steps=steps,
            effective_dt_ms=dt,
            first_timestamp_ms=start,
        )
    )
    assert len(timestamps) == n
    assert timestamps[0] == start
    assert all(b >= a for a, b in itertools.pairwise(timestamps))


# ---------------------------------------------------------------------------
# WaypointConfig spacing — the divisor the live horizon is laid out on.
# ---------------------------------------------------------------------------


def test_a_zero_step_spacing_is_rejected_at_construction():
    """``single_step_dt_ms`` is a divisor, so zero has to fail loudly and early.

    ``dt_ms=0`` means "single step" elsewhere in this API, which makes it a
    plausible thing to pass here too — where it instead leaves the live path with
    no timeline to lay waypoints on, and divides by zero on the first target.
    """
    import pytest

    from novapolicy.types import WaypointConfig

    with pytest.raises(ValueError, match="single_step_dt_ms"):
        _ = WaypointConfig(single_step_dt_ms=0.0)
    with pytest.raises(ValueError, match="single_step_dt_ms"):
        _ = WaypointConfig(single_step_dt_ms=-10.0)
    with pytest.raises(ValueError, match="min_chunk_horizon_ms"):
        _ = WaypointConfig(min_chunk_horizon_ms=-1.0)

    # Zero is documented as "no automatic extension", so it must stay legal.
    assert WaypointConfig(min_chunk_horizon_ms=0.0).min_chunk_horizon_ms == 0.0
