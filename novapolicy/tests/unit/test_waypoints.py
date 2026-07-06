"""Tests for make_waypoints_request — the documented jogging timestamp protocol.

This pure function turns raw steps + dt + an anchor into a NOVA
``ActionChunkRequest`` (a single list of timestamped waypoints, each carrying
either joint or Cartesian coordinates), scaling every timestamp by the clock's
speed ratio. Every waypoint is ``base + i*dt``; only ``base`` varies:
  * explicit anchor (``anchor_ms >= 0``): base is that anchor;
  * "now" anchor (``anchor_ms == NOW``): base is the clock's current elapsed;
  * ``anchor_offset_steps`` then shifts base by whole dt steps (+1 ahead for
    live targets, negative to backdate an RTC seam).
"""

from __future__ import annotations

import itertools

from hypothesis import given, settings, strategies as st

from nova import api
from novapolicy.jogging.clock import JoggingTimeClock
from novapolicy.jogging.waypoints import NOW, make_waypoints_request


def _joint_timestamps(req) -> list[int]:
    return [w.timestamp for w in req.waypoints]


def _joint_steps(req) -> list[list[float]]:
    return [list(w.waypoint.root.joints.root) for w in req.waypoints]


# ---------------------------------------------------------------------------
# Trajectory-absolute mode (first_timestamp_ms >= 0)
# ---------------------------------------------------------------------------


def test_absolute_mode_places_timestamps_starting_at_the_scaled_base():
    """anchor_ms=100, dt=10, ratio=1 -> [100, 110, 120]; steps preserved."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    steps = [[0.0] * 6, [0.1] * 6, [0.2] * 6]
    req = make_waypoints_request(clock, "joint", steps=steps, effective_dt_ms=10.0, anchor_ms=100)
    assert isinstance(req, api.models.ActionChunkRequest)
    assert _joint_timestamps(req) == [100, 110, 120]
    assert _joint_steps(req) == steps


def test_absolute_mode_scales_both_the_base_and_the_step_spacing():
    """speed_ratio=2 stretches base 100->200 and dt 10->20 -> [200, 220, 240]."""
    clock = JoggingTimeClock(speed_ratio=2.0)
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(clock, "joint", steps=steps, effective_dt_ms=10.0, anchor_ms=100)
    assert _joint_timestamps(req) == [200, 220, 240]


# ---------------------------------------------------------------------------
# "Now" anchor with a +1-step offset: the sequential off-by-one
# ---------------------------------------------------------------------------


def test_now_anchor_one_step_ahead_starts_one_dt_after_now_not_at_now():
    """A fresh (unstarted) clock has elapsed 0; +1-step offset -> [dt, 2dt, ...]."""
    clock = JoggingTimeClock(speed_ratio=1.0)  # never started -> client_elapsed_ms == 0
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        anchor_ms=NOW,
        anchor_offset_steps=1,
    )
    # First waypoint is dt in the future, NOT 0 — the server interpolates toward it.
    assert _joint_timestamps(req) == [10, 20, 30]


# ---------------------------------------------------------------------------
# Backdated "now" anchor (RTC seam), resolved at yield time
# ---------------------------------------------------------------------------


def test_backdated_now_anchor_anchors_in_the_past_so_the_backdated_step_lands_at_now():
    """A negative offset backdates the anchor so an already-passed step lands at now.

    A fresh clock reports elapsed 0, so base = max(0, 0 - 2*dt) = 0 and the
    timestamps are [0, dt, 2dt, ...] (start *at* the anchor). The point matching
    the robot lands at step 2 (the backdate).
    """
    clock = JoggingTimeClock(speed_ratio=1.0)  # never started -> client_elapsed_ms == 0
    steps = [[0.0] * 6, [0.0] * 6, [0.0] * 6]
    req = make_waypoints_request(
        clock,
        "joint",
        steps=steps,
        effective_dt_ms=10.0,
        anchor_ms=NOW,
        anchor_offset_steps=-2,
    )
    assert _joint_timestamps(req) == [0, 10, 20]


def test_now_anchor_is_read_at_yield_time_not_precomputed():
    """The 'now' anchor comes from the clock at call time, so advancing the
    session clock shifts the whole progression — this is the staleness fix."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    clock.start()
    # Force a known elapsed by backdating the clock's start marker.
    import time as _t

    clock._client_start_time = _t.monotonic() - 0.500  # ~500 ms elapsed
    req = make_waypoints_request(
        clock,
        "joint",
        steps=[[0.0] * 6],
        effective_dt_ms=10.0,
        anchor_ms=NOW,
        anchor_offset_steps=-10,  # backdate 10 steps * 10ms = 100 ms
    )
    # base ~= 500 - 100 = 400 ms (allow scheduling slack)
    assert 380 <= req.waypoints[0].timestamp <= 460


# ---------------------------------------------------------------------------
# Request-type dispatch + cartesian payload layout
# ---------------------------------------------------------------------------


def test_cartesian_mode_builds_a_pose_request_splitting_position_and_orientation():
    """[x, y, z, rx, ry, rz] -> position=[x,y,z] (mm), orientation=[rx,ry,rz] (rad)."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    steps = [[500.0, 200.0, 300.0, 0.1, 0.2, 0.3]]
    req = make_waypoints_request(clock, "cartesian", steps=steps, effective_dt_ms=10.0, anchor_ms=0)
    assert isinstance(req, api.models.ActionChunkRequest)
    wp = req.waypoints[0]
    assert wp.timestamp == 0
    assert wp.waypoint.root.kind == "POSE"
    assert list(wp.waypoint.root.pose.position.root) == [500.0, 200.0, 300.0]
    assert list(wp.waypoint.root.pose.orientation.root) == [0.1, 0.2, 0.3]


def test_joint_mode_builds_a_joint_request():
    """The mode argument selects the waypoint kind: 'joint' -> JOINTS waypoints."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    req = make_waypoints_request(
        clock, "joint", steps=[[0.0] * 6], effective_dt_ms=10.0, anchor_ms=0
    )
    assert isinstance(req, api.models.ActionChunkRequest)
    assert req.waypoints[0].waypoint.root.kind == "JOINTS"


def test_empty_steps_produce_no_waypoints():
    """A chunk with no steps yields an empty waypoint list (nothing to send)."""
    clock = JoggingTimeClock(speed_ratio=1.0)
    req = make_waypoints_request(clock, "joint", steps=[], effective_dt_ms=10.0, anchor_ms=0)
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
        make_waypoints_request(clock, "joint", steps=steps, effective_dt_ms=dt, anchor_ms=start)
    )
    assert len(ts) == n
    assert ts[0] == clock.scale_timestamp(start)
    assert all(b >= a for a, b in itertools.pairwise(ts))
