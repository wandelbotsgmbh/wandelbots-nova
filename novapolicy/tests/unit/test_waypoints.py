"""Tests for building the action chunk request and for the waypoint configuration."""

from __future__ import annotations

import json

import pytest

from nova import api
from novapolicy.jogging.waypoints import make_timed_waypoints_request
from novapolicy.types import WaypointConfig


def _wire(req) -> dict:
    """The request as the action chunk stream sends it."""
    return json.loads(req.model_dump_json(exclude_none=True))


def test_cartesian_mode_builds_pose_waypoints_splitting_position_and_orientation():
    """[x, y, z, rx, ry, rz] maps to NOVA position and rotation vector."""
    req = make_timed_waypoints_request("cartesian", [(0, [500.0, 200.0, 300.0, 0.1, 0.2, 0.3])])
    assert isinstance(req, api.models.ActionChunkRequest)
    waypoint = req.waypoints[0]
    assert waypoint.waypoint.kind == "POSE"
    assert waypoint.timestamp == 0
    pose = waypoint.waypoint.pose
    assert list(pose.position) == [500.0, 200.0, 300.0]
    assert list(pose.orientation) == [0.1, 0.2, 0.3]


def test_empty_steps_produce_no_waypoints():
    assert make_timed_waypoints_request("joint", []).waypoints == []


def test_delay_until_braking_is_sent_with_the_waypoints():
    """The pinned API client predates the field, so the request has to carry it itself."""
    req = make_timed_waypoints_request("joint", [(100, [0.0] * 6)], delay_until_braking_ms=200)
    assert _wire(req)["delay_until_braking"] == 200


def test_without_a_delay_the_request_is_unchanged_for_older_servers():
    assert "delay_until_braking" not in _wire(
        make_timed_waypoints_request("joint", [(100, [0.0] * 6)])
    )


def test_a_negative_delay_until_braking_is_rejected_at_construction():
    with pytest.raises(ValueError, match="delay_until_braking_ms"):
        _ = WaypointConfig(delay_until_braking_ms=-1)
    assert WaypointConfig().delay_until_braking_ms is None


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


def test_hold_padding_refuses_a_negative_horizon_and_allows_zero() -> None:
    """Zero means no padding, so it must stay legal."""
    import pytest

    from novapolicy.jogging.modules import HoldPadding

    with pytest.raises(ValueError, match="min_horizon_ms"):
        _ = HoldPadding(min_horizon_ms=-1.0)
    assert HoldPadding(min_horizon_ms=0.0).min_horizon_ms == 0.0
