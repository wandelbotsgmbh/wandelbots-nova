"""Tests for make_waypoints_request — the documented jogging timestamp protocol.

This pure function is what executor.md's "Timestamp Protocol" and jogging.md's
"Waypoint request types" tables actually describe: it turns raw steps + dt + a
start timestamp into a NOVA JointWaypointsRequest or PoseWaypointsRequest, scaling
every timestamp by the clock's speed ratio.

The two timing modes have a deliberate off-by-one that matters on the robot:
  * absolute (first_timestamp_ms >= 0): timestamps are
    [base, base + dt, base + 2*dt, ...] starting *at* base;
  * relative (first_timestamp_ms == -1): timestamps are
    [now + dt, now + 2*dt, ...] starting one dt *after* now.
"""

from __future__ import annotations

import itertools

from hypothesis import given, settings, strategies as st

from nova import api
from policy.jogging.clock import JoggingTimeClock
from policy.jogging.waypoints import make_waypoints_request


def _joint_timestamps(req) -> list[int]:
    return [w.timestamp for w in req.waypoints]


def _joint_steps(req) -> list[list[float]]:
    return [list(w.joints.root) for w in req.waypoints]


# ---------------------------------------------------------------------------
# Trajectory-absolute mode (first_timestamp_ms >= 0)
# ---------------------------------------------------------------------------


def test_absolute_mode_places_timestamps_starting_at_the_scaled_base():
    """first_timestamp_ms=100, dt=10, ratio=1 -> [100, 110, 120]; steps preserved."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    steps = [[0.0] * 6, [0.1] * 6, [0.2] * 6]
    req = make_waypoints_request(
        clock, "joint", steps=steps, effective_dt_ms=10.0, first_timestamp_ms=100
    )
    assert isinstance(req, api.models.JointWaypointsRequest)
    assert _joint_timestamps(req) == [100, 110, 120]
    assert _joint_steps(req) == steps


def test_absolute_mode_scales_both_the_base_and_the_step_spacing():
    """speed_ratio=2 stretches base 100->200 and dt 10->20 -> [200, 220, 240]."""
    clock = JoggingTimeClock(speed_ratio=2.0)
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock, "joint", steps=steps, effective_dt_ms=10.0, first_timestamp_ms=100
    )
    assert _joint_timestamps(req) == [200, 220, 240]


# ---------------------------------------------------------------------------
# Relative mode (first_timestamp_ms == -1): the off-by-one
# ---------------------------------------------------------------------------


def test_relative_mode_starts_one_dt_after_now_not_at_now():
    """A fresh (unstarted) clock has elapsed 0; relative timestamps are [dt, 2dt, ...]."""
    clock = JoggingTimeClock(speed_ratio=1.0)  # never started -> client_elapsed_ms == 0
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock, "joint", steps=steps, effective_dt_ms=10.0, first_timestamp_ms=-1
    )
    # First waypoint is dt in the future, NOT 0 — the server interpolates toward it.
    assert _joint_timestamps(req) == [10, 20, 30]


# ---------------------------------------------------------------------------
# Request-type dispatch + cartesian payload layout
# ---------------------------------------------------------------------------


def test_cartesian_mode_builds_a_pose_request_splitting_position_and_orientation():
    """[x, y, z, rx, ry, rz] -> position=[x,y,z] (mm), orientation=[rx,ry,rz] (rad)."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    steps = [[500.0, 200.0, 300.0, 0.1, 0.2, 0.3]]
    req = make_waypoints_request(
        clock, "cartesian", steps=steps, effective_dt_ms=10.0, first_timestamp_ms=0
    )
    assert isinstance(req, api.models.PoseWaypointsRequest)
    wp = req.waypoints[0]
    assert wp.timestamp == 0
    assert list(wp.pose.position.root) == [500.0, 200.0, 300.0]
    assert list(wp.pose.orientation.root) == [0.1, 0.2, 0.3]


def test_joint_mode_builds_a_joint_request():
    """The mode argument selects the request type: 'joint' -> JointWaypointsRequest."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    req = make_waypoints_request(
        clock, "joint", steps=[[0.0] * 6], effective_dt_ms=10.0, first_timestamp_ms=0
    )
    assert isinstance(req, api.models.JointWaypointsRequest)


def test_empty_steps_produce_no_waypoints():
    """A chunk with no steps yields an empty waypoint list (nothing to send)."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    req = make_waypoints_request(
        clock, "joint", steps=[], effective_dt_ms=10.0, first_timestamp_ms=0
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
    """For any ratio/dt/start, absolute timestamps start at the scaled base and
    never go backwards (the server needs a monotonic timeline)."""
    clock = JoggingTimeClock(speed_ratio=ratio)
    steps = [[0.0] * 6 for _ in range(n)]
    ts = _joint_timestamps(
        make_waypoints_request(
            clock, "joint", steps=steps, effective_dt_ms=dt, first_timestamp_ms=start
        )
    )
    assert len(ts) == n
    assert ts[0] == clock.scale_timestamp(start)
    assert all(b >= a for a, b in itertools.pairwise(ts))
