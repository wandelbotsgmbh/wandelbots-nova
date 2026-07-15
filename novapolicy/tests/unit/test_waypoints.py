"""Tests for make_waypoints_request — the documented jogging timestamp protocol.

This pure function turns raw steps and timing into a NOVA
JointWaypointsRequest or PoseWaypointsRequest. Every waypoint is
``base + i*dt``: an exact raw NOVA timestamp may provide ``base``; otherwise
server "now" is resolved at call time. ``timestamp_offset_steps`` shifts that
base by whole intervals.
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
    return [list(w.joints.root) for w in req.waypoints]


# ---------------------------------------------------------------------------
# Exact controller-timestamp mode
# ---------------------------------------------------------------------------


def test_absolute_mode_places_timestamps_starting_at_the_exact_base():
    """first_timestamp_ms=100, dt=10 -> [100, 110, 120]; steps preserved."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    steps = [[0.0] * 6, [0.1] * 6, [0.2] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        first_timestamp_ms=100,
    )
    assert isinstance(req, api.models.JointWaypointsRequest)
    assert _joint_timestamps(req) == [100, 110, 120]
    assert _joint_steps(req) == steps


def test_absolute_mode_scales_spacing_without_rescaling_the_server_timestamp():
    """speed_ratio=2 preserves base 100 and stretches dt 10->20."""
    clock = JoggingTimeClock(speed_ratio=2.0)
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        first_timestamp_ms=100,
    )
    assert _joint_timestamps(req) == [100, 120, 140]


def test_timestamp_offset_is_applied_in_the_server_clock_domain():
    clock = JoggingTimeClock(speed_ratio=2.0)
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[[0.0] * 6],
        effective_dt_ms=10.0,
        first_timestamp_ms=135,
        timestamp_offset_steps=1,
    )
    assert _joint_timestamps(req) == [155]


def test_controller_timed_policy_spacing_bypasses_client_wall_clock_scaling():
    clock = JoggingTimeClock(speed_ratio=2.0)
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        first_timestamp_ms=135,
        server_dt_ms=10.0,
    )

    assert _joint_timestamps(req) == [135, 145, 155]


# ---------------------------------------------------------------------------
# Server "now" with a +1-step offset
# ---------------------------------------------------------------------------


def test_now_timestamp_one_step_ahead_starts_one_dt_after_now_not_at_now():
    """A fresh clock reports zero; +1 step produces [dt, 2dt, ...]."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        timestamp_offset_steps=1,
    )
    assert _joint_timestamps(req) == [10, 20, 30]


def test_synced_now_timestamp_uses_server_clock_without_shared_origin_assumption():
    """Server now is based on the latest server sample, not client elapsed time."""
    import time as _t

    clock = JoggingTimeClock(speed_ratio=2.0, synced=True)
    clock._client_start_time = _t.monotonic() - 0.100
    clock._last_server_ts_ms = 5_000
    clock._last_server_wall = _t.monotonic()
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[[0.0] * 6, [0.0] * 6],
        effective_dt_ms=10.0,
        timestamp_offset_steps=1,
    )

    timestamps = _joint_timestamps(req)
    assert 5_020 <= timestamps[0] <= 5_022
    assert timestamps[1] - timestamps[0] == 20


# ---------------------------------------------------------------------------
# Backdated server "now", resolved at yield time
# ---------------------------------------------------------------------------


def test_backdated_now_timestamp_is_clamped_to_zero():
    clock = JoggingTimeClock(speed_ratio=1.0)
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        timestamp_offset_steps=-2,
    )
    assert _joint_timestamps(req) == [0, 10, 20]


def test_now_timestamp_is_read_at_yield_time_not_precomputed():
    """Advancing the session clock shifts the whole progression."""
    import time as _t

    clock = JoggingTimeClock(speed_ratio=1.0)
    clock.start()
    clock._client_start_time = _t.monotonic() - 0.500
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[[0.0] * 6],
        effective_dt_ms=10.0,
        timestamp_offset_steps=-10,
    )
    assert 380 <= req.waypoints[0].timestamp <= 460


# ---------------------------------------------------------------------------
# Request-type dispatch + cartesian payload layout
# ---------------------------------------------------------------------------


def test_cartesian_mode_builds_a_pose_request_splitting_position_and_orientation():
    """[x, y, z, rx, ry, rz] maps to NOVA position and rotation vector."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    steps = [[500.0, 200.0, 300.0, 0.1, 0.2, 0.3]]
    req = make_waypoints_request(
        clock,
        "cartesian",
        steps=steps,
        effective_dt_ms=10.0,
        first_timestamp_ms=0,
    )
    assert isinstance(req, api.models.PoseWaypointsRequest)
    waypoint = req.waypoints[0]
    assert waypoint.timestamp == 0
    assert list(waypoint.pose.position.root) == [500.0, 200.0, 300.0]
    assert list(waypoint.pose.orientation.root) == [0.1, 0.2, 0.3]


def test_joint_mode_builds_a_joint_request():
    clock = JoggingTimeClock(speed_ratio=1.0)
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[[0.0] * 6],
        effective_dt_ms=10.0,
        first_timestamp_ms=0,
    )
    assert isinstance(req, api.models.JointWaypointsRequest)


def test_empty_steps_produce_no_waypoints():
    clock = JoggingTimeClock(speed_ratio=1.0)
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

_RATIO = st.floats(min_value=1.0, max_value=20.0, allow_nan=False, allow_infinity=False)
_DT = st.floats(min_value=1.0, max_value=200.0, allow_nan=False, allow_infinity=False)
_START = st.integers(min_value=0, max_value=100_000)
_N = st.integers(min_value=1, max_value=16)


@given(ratio=_RATIO, dt=_DT, start=_START, n=_N)
@settings(max_examples=200, deadline=None)
def test_absolute_timestamps_are_a_nondecreasing_progression_from_the_base(ratio, dt, start, n):
    """Exact server timestamps remain the base and never go backwards."""
    clock = JoggingTimeClock(speed_ratio=ratio)
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
