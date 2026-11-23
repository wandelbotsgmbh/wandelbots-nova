"""Test doubles shared by the novapolicy tests: motion groups and their states.

Imported as ``novapolicy.tests.doubles`` (``novapolicy/tests`` is a namespace package).
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock


def mg(mg_id: str = "0@ur10e", controller_id: str | None = None) -> MagicMock:
    """A motion group stand-in; its controller is the part of the id after ``@``."""
    group = MagicMock()
    group.id = mg_id
    group._controller_id = controller_id or mg_id.split("@")[1]
    group._cell = "cell"
    group._api_client = MagicMock(close=AsyncMock())
    return group


def state(joints: tuple[float, ...] = (0.0,) * 6, pose: object | None = None) -> MagicMock:
    """A motion group state with joints and an optional pose, but no TCP."""
    s = MagicMock()
    s.joints = joints
    s.pose = pose
    s.tcp = None
    return s


def tcp_state(values: tuple[float, float, float, float, float, float]) -> MagicMock:
    """A state at the TCP pose ``(x, y, z, rx, ry, rz)``."""
    return state(pose=SimpleNamespace(position=values[:3], orientation=values[3:]))


def fake_session(num_joints: int = 6, *, mode: str = "joint") -> MagicMock:
    """A stand-in for a live WaypointJoggingSession (the robot transport)."""
    session = MagicMock()
    session.motion_group = MagicMock()
    session.motion_group_id = "0@ur10e"
    session.num_joints = num_joints
    session.mode = mode
    session.estimated_server_timestamp_ms = 0
    session.single_step_dt_ms = 50.0
    session.has_failed = False
    session.failure_reason = ""
    session.failure_exception = None
    session.stop_condition_triggered = None
    session.jogging_state = None
    # Explicit, not left to auto-attribute: a bare MagicMock attribute is
    # truthy, which would silently satisfy the settle check's standstill term
    # and stop these tests exercising it at all.
    session.is_at_standstill = False
    session.queued_chunk_count = 0
    session.queued_chunk = None  # no Bridge module in front
    session.executing_action_timestep = -1
    session.scheduled_chunk_count = 0
    session.scheduled_until_server_ms = 0
    session.scheduled_waypoint_timestamps = ()
    session.last_server_timestamp_ms = 0
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.drain = AsyncMock(return_value=True)
    session.wait_ready = AsyncMock()
    session.write_ios = AsyncMock()

    def update_chunk(*_args: object, **_kwargs: object) -> None:
        session.queued_chunk_count += 1

    session.update_chunk = MagicMock(side_effect=update_chunk)
    session.current_state = state((0.0,) * num_joints)
    return session
