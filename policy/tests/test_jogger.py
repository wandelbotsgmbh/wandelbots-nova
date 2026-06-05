"""Behavioural tests for jog_joints().

The jogger lets you stream joint targets to one or more robots and surfaces
faults through its ``async for`` loop. These tests state that contract through
the public ``jog_joints()`` API, substituting only the robot transport
(``WaypointJoggingSession``) — they never reach into the jogger's internals.
"""

from __future__ import annotations

from collections.abc import Iterator
import contextlib
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from policy.jogging import jog_joints
from policy.types import MotionError


def _mg(mg_id: str) -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    return mg


def _fake_session(num_joints: int = 6) -> MagicMock:
    session = MagicMock()
    session.num_joints = num_joints
    session.mode = "joint"
    session.has_failed = False
    session.failure_exception = None
    session.stop_condition_triggered = None
    session.session_elapsed_ms = 0
    session.current_state = MagicMock(joints=(0.0,) * num_joints)
    session.update_chunk = MagicMock()
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.wait_ready = AsyncMock()
    return session


@contextlib.contextmanager
def _jogger(*mg_ids: str, num_joints: int = 6) -> Iterator[tuple]:
    """Build a real jogger over fake robot transports.

    Yields ``(jogger, motion_groups, sessions_by_mg)`` so a test can assert on
    what the jogger streamed to each robot.
    """
    mgs = [_mg(mid) for mid in mg_ids]
    sessions: dict[object, MagicMock] = {}

    def make_session(*, motion_group: object, **_kw: object) -> MagicMock:
        sessions[motion_group] = _fake_session(num_joints)
        return sessions[motion_group]

    with patch("policy.jogging.jogger.WaypointJoggingSession", side_effect=make_session):
        jogger = jog_joints(mgs if len(mgs) > 1 else mgs[0])
    yield jogger, mgs, sessions


# ---------------------------------------------------------------------------
# Setting a target streams waypoints to the robot
# ---------------------------------------------------------------------------


def test_setting_a_single_target_streams_it_to_the_robot():
    """A single joint target is pushed to the session as one waypoint step."""
    with _jogger("0@ur10e") as (jogger, (mg,), sessions):
        jogger.set_target([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    sessions[mg].update_chunk.assert_called_once_with(
        steps=[[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]], dt_ms=0.0
    )
    assert jogger.target == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_setting_a_chunk_streams_every_step_and_tracks_the_last():
    """A chunk of future targets is streamed whole; the last step is the target."""
    chunk = [[float(i)] * 6 for i in range(4)]
    with _jogger("0@ur10e") as (jogger, (mg,), sessions):
        jogger.set_target(chunk, dt_ms=33.0)
    sessions[mg].update_chunk.assert_called_once_with(steps=chunk, dt_ms=33.0, start_time_ms=0)
    assert jogger.target == chunk[-1]


def test_each_arm_in_a_dual_setup_receives_its_own_target():
    """With two robots, each motion group is streamed only its own target."""
    with _jogger("0@ur5e-left", "0@ur5e-right") as (jogger, (left, right), sessions):
        jogger.set_target({left: [1.0] * 6, right: [2.0] * 6})
    sessions[left].update_chunk.assert_called_once()
    sessions[right].update_chunk.assert_called_once()
    assert jogger.target == {left: [1.0] * 6, right: [2.0] * 6}


# ---------------------------------------------------------------------------
# Bad targets are rejected before anything is streamed
# ---------------------------------------------------------------------------


def test_a_target_with_the_wrong_joint_count_is_rejected():
    """A 3-value target for a 6-joint robot raises before any waypoint is sent."""
    with _jogger("0@ur10e") as (jogger, (mg,), sessions):
        with pytest.raises(ValueError, match="expects 6"):
            jogger.set_target([1.0, 2.0, 3.0])
        sessions[mg].update_chunk.assert_not_called()


def test_a_bare_list_for_a_dual_setup_is_rejected():
    """Two robots need a dict target; a bare list is a usage error."""
    with (
        _jogger("0@ur5e-left", "0@ur5e-right") as (jogger, _mgs, _sessions),
        pytest.raises(TypeError, match="dict"),
    ):
        jogger.set_target([1.0] * 6)


# ---------------------------------------------------------------------------
# Faults and stops surface through the loop
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_motion_fault_surfaces_as_an_exception_through_the_loop():
    """A joint-limit / collision fault on the session is raised to the caller."""
    with _jogger("0@ur10e") as (jogger, (mg,), sessions):
        sessions[mg].has_failed = True
        sessions[mg].failure_exception = MotionError("0@ur10e", "joint_limit")
        with pytest.raises(MotionError):
            async for _ in jogger:
                break


@pytest.mark.asyncio
async def test_a_lost_connection_surfaces_as_an_exception_through_the_loop():
    """A dropped jogging connection is raised, not swallowed."""
    with _jogger("0@ur10e") as (jogger, (mg,), sessions):
        sessions[mg].has_failed = True
        sessions[mg].failure_exception = RuntimeError("connection reset")
        with pytest.raises(RuntimeError, match="connection reset"):
            async for _ in jogger:
                break


@pytest.mark.asyncio
async def test_a_stop_condition_ends_the_loop_without_raising():
    """A fired stop condition ends iteration normally and is reported by name."""
    with _jogger("0@ur10e") as (jogger, (mg,), sessions):
        sessions[mg].stop_condition_triggered = "workspace_limit"
        iterations = 0
        async for _ in jogger:
            iterations += 1
            break
    assert iterations == 0  # loop ended before yielding any state
    assert jogger.stop_condition_triggered == "workspace_limit"
