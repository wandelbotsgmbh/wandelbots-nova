"""Behavioural tests for jog_joints().

The jogger lets you stream joint targets to one or more robots and surfaces
faults through its ``async for`` loop. These tests state that contract through
the public ``jog_joints()`` API, substituting only the robot transport
(``WaypointJoggingSession``) — they never reach into the jogger's internals.
"""

from __future__ import annotations

import contextlib
import logging
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from novapolicy.jogging import jog_joints, jog_tcp
from novapolicy.jogging.jogger import JointJogger, TcpJogger
from novapolicy.jogging.live_buffer import LiveBuffer
from novapolicy.types import MotionError

_JOGGER = "novapolicy.jogging.jogger"

_CLOCK_BASE = 1_000.0


class _FakeClock:
    """Monotonic clock the tests drive explicitly.

    The trajectory clock is rate-limited against real time, so a test that
    advances the jog timeline has to advance the wall clock with it.
    """

    def __init__(self) -> None:
        self.now = _CLOCK_BASE

    def monotonic(self) -> float:
        return self.now


_CLOCK = _FakeClock()


@pytest.fixture(autouse=True)
def _fake_monotonic(monkeypatch):
    _CLOCK.now = _CLOCK_BASE
    monkeypatch.setattr(f"{_JOGGER}.time", _CLOCK)


@contextlib.contextmanager
def _no_estop_no_rerun():
    """Stub the e-stop monitor and disable Rerun for context-lifecycle tests."""
    estop = MagicMock()
    estop.start = AsyncMock()
    estop.stop = AsyncMock()
    with (
        patch(f"{_JOGGER}.EstopMonitor", return_value=estop),
        patch("novapolicy.rerun._is_rerun_active", return_value=False),
    ):
        yield


_JointSetup = tuple[JointJogger, list[MagicMock], dict[object, MagicMock]]
_TcpSetup = tuple[TcpJogger, MagicMock, dict[object, MagicMock]]


def _mg(mg_id: str) -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    return mg


def _fake_session(num_joints: int = 6, *, mode: str = "joint") -> MagicMock:
    session = MagicMock()
    session.num_joints = num_joints
    session.mode = mode
    session.has_failed = False
    session.failure_exception = None
    session.stop_condition_triggered = None
    session.estimated_server_timestamp_ms = 0
    session.session_elapsed_ms = 0.0
    session.single_step_dt_ms = 50.0
    session.min_chunk_horizon_ms = 200.0
    session.current_state = MagicMock(joints=(0.0,) * num_joints)
    session.update_chunk = MagicMock()
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.drain = AsyncMock(return_value=True)
    session.wait_ready = AsyncMock()
    return session


def _build_joint_jogger(
    *mg_ids: str,
    num_joints: int = 6,
    pipeline: tuple[object, ...] = (),
) -> _JointSetup:
    """Build a real joint jogger over fake robot transports.

    The transport is only patched while the jogger is being constructed (that
    is when the sessions are created); the returned fakes are kept so a test
    can assert what got streamed to each robot.
    """
    mgs = [_mg(mid) for mid in mg_ids]
    sessions: dict[object, MagicMock] = {}

    def make_session(*, motion_group: object, **kw: object) -> MagicMock:
        sessions[motion_group] = _fake_session(num_joints)
        sessions[motion_group].pipeline = kw.get("pipeline")
        return sessions[motion_group]

    with patch("novapolicy.jogging.jogger.WaypointJoggingSession", side_effect=make_session):
        jogger = jog_joints(mgs if len(mgs) > 1 else mgs[0], pipeline=pipeline)
    return jogger, mgs, sessions


def _build_tcp_jogger(
    mg_id: str,
    tcp: str = "Flange",
    *,
    num_joints: int = 6,
    pipeline: tuple[object, ...] = (),
) -> _TcpSetup:
    """Build a real single-arm TCP jogger over a fake robot transport."""
    mg = _mg(mg_id)
    sessions: dict[object, MagicMock] = {}

    def make_session(*, motion_group: object, **kw: object) -> MagicMock:
        sessions[motion_group] = _fake_session(num_joints, mode="cartesian")
        sessions[motion_group].pipeline = kw.get("pipeline")
        return sessions[motion_group]

    with patch("novapolicy.jogging.jogger.WaypointJoggingSession", side_effect=make_session):
        jogger = jog_tcp(mg, tcp=tcp, pipeline=pipeline)
    return jogger, mg, sessions


# ---------------------------------------------------------------------------
# Setting a target streams waypoints to the robot
# ---------------------------------------------------------------------------


def _tick(jogger, sessions, elapsed_ms: float) -> None:
    """Advance the shared jog timeline to ``elapsed_ms``."""
    _CLOCK.now = _CLOCK_BASE + elapsed_ms / 1000.0
    jogger._loop_t0 = _CLOCK_BASE  # type: ignore[attr-defined]
    jogger._ack0_ms = 0.0  # type: ignore[attr-defined]
    for session in sessions.values():
        session.session_elapsed_ms = elapsed_ms


def test_setting_a_single_target_streams_it_to_the_robot():
    """The very first live target has no motion history, so it goes out alone."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    jogger.set_target([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    sessions[mg].update_chunk.assert_called_once_with(
        steps=[[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]],
        dt_ms=0.0,
        first_timestamp_ms=None,
        timestamp_offset_steps=1,
        extend_buffer=False,
    )
    assert jogger.target == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_the_pipeline_is_handed_to_every_session():
    """Opt-in modules (e.g. LiveBuffer, EaseIn) run in the sessions; the jogger does not shape
    targets itself, and a live target goes out unchanged by default."""
    from novapolicy import EaseIn

    pipeline = (LiveBuffer(window_ms=30.0), EaseIn(duration_s=1.0))
    jogger, mgs, sessions = _build_joint_jogger("0@left", "0@right", pipeline=pipeline)
    assert all(sessions[mg].pipeline == pipeline for mg in mgs)
    _, tcp_mg, tcp_sessions = _build_tcp_jogger("0@ur10e", pipeline=pipeline)
    assert tcp_sessions[tcp_mg].pipeline == pipeline
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    jogger.set_target([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert sessions[mg].update_chunk.call_args.kwargs["steps"][0] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_setting_a_chunk_streams_every_step_and_tracks_the_last():
    """A chunk of future targets is streamed whole; the last step is the target."""
    chunk = [[float(i)] * 6 for i in range(4)]
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    jogger.set_chunk(chunk, dt_ms=33.0)
    sessions[mg].update_chunk.assert_called_once_with(
        steps=chunk,
        dt_ms=33.0,
        first_timestamp_ms=0,
        extend_buffer=True,
    )
    assert jogger.target == chunk[-1]


def test_with_a_live_buffer_the_jogger_still_sends_each_live_target_alone(caplog):
    """The buffering moved into the session's pipeline (see the LiveBuffer tests in
    test_waypoint_session.py): the jogger sends every live target as it came, one waypoint,
    no padding, and a chunk as it came, without clearing or warning about anything."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e", pipeline=(LiveBuffer(window_ms=30.0),))

    with caplog.at_level(logging.WARNING, logger="novapolicy.jogging.jogger"):
        for i, value in enumerate((1.0, 2.0, 3.0)):
            _tick(jogger, sessions, i * 10.0)
            jogger.set_target([value] * 6)
        jogger.set_chunk([[9.0] * 6, [9.0] * 6], dt_ms=10.0)

    calls = [call.kwargs for call in sessions[mg].update_chunk.call_args_list]
    assert [kwargs["steps"] for kwargs in calls] == [
        [[1.0] * 6],
        [[2.0] * 6],
        [[3.0] * 6],
        [[9.0] * 6, [9.0] * 6],
    ]
    assert all(kwargs["dt_ms"] == 0.0 for kwargs in calls[:3])
    assert all(kwargs["extend_buffer"] is False for kwargs in calls[:3])
    assert calls[3]["extend_buffer"] is True
    assert [r for r in caplog.records if r.name == "novapolicy.jogging.jogger"] == []


def test_each_arm_in_a_dual_setup_receives_its_own_target():
    """With two robots, each motion group is streamed only its own target."""
    jogger, (left, right), sessions = _build_joint_jogger("0@ur5e-left", "0@ur5e-right")
    jogger.set_target({left: [1.0] * 6, right: [2.0] * 6})
    sessions[left].update_chunk.assert_called_once()
    sessions[right].update_chunk.assert_called_once()
    assert jogger.target == {left: [1.0] * 6, right: [2.0] * 6}


# ---------------------------------------------------------------------------
# Bad targets are rejected before anything is streamed
# ---------------------------------------------------------------------------


def test_a_chunk_handed_to_set_target_is_rejected():
    """The split exists so a chunk cannot arrive where a LiveBuffer's live path applies.

    Accepting it there meant the buffer was silently irrelevant for that push,
    with nothing to tell the caller.
    """
    jogger, _, _ = _build_joint_jogger("0@ur10e")

    with pytest.raises(TypeError, match="set_chunk"):
        jogger.set_target([[1.0] * 6, [2.0] * 6])


def test_a_single_target_handed_to_set_chunk_is_rejected():
    """And the reverse, so dt_ms is never applied to something without steps."""
    jogger, _, _ = _build_joint_jogger("0@ur10e")

    with pytest.raises(TypeError, match="set_target"):
        jogger.set_chunk([1.0] * 6, dt_ms=10.0)


def test_set_chunk_requires_a_positive_dt_ms():
    """dt_ms is the step spacing; there is no meaningful default for it."""
    jogger, _, _ = _build_joint_jogger("0@ur10e")

    with pytest.raises(ValueError, match="dt_ms"):
        jogger.set_chunk([[1.0] * 6, [2.0] * 6], dt_ms=0.0)


def test_an_empty_chunk_is_rejected_rather_than_sent():
    """An empty list has no shape to infer and no waypoints to send."""
    jogger, _, _ = _build_joint_jogger("0@ur10e")

    with pytest.raises(TypeError, match="empty"):
        jogger.set_chunk([], dt_ms=10.0)


def test_a_pose_chunk_handed_to_tcp_set_target_is_rejected():
    """Same split on the TCP jogger, where the live form is a Pose."""
    jogger, _, _ = _build_tcp_jogger("0@ur10e")

    with pytest.raises(TypeError, match="set_chunk"):
        jogger.set_target([[500.0, 200.0, 300.0, 0.0, 3.14, 0.0]])


def test_a_target_with_the_wrong_joint_count_is_rejected():
    """A 3-value target for a 6-joint robot raises before any waypoint is sent."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    with pytest.raises(ValueError, match="expects 6"):
        jogger.set_target([1.0, 2.0, 3.0])
    sessions[mg].update_chunk.assert_not_called()


def test_a_bare_list_for_a_dual_setup_is_rejected():
    """Two robots need a dict target; a bare list is a usage error."""
    jogger, _mgs, _sessions = _build_joint_jogger("0@ur5e-left", "0@ur5e-right")
    with pytest.raises(TypeError, match="dict"):
        jogger.set_target([1.0] * 6)


# ---------------------------------------------------------------------------
# Faults and stops surface through the loop
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_motion_fault_surfaces_as_an_exception_through_the_loop():
    """A joint-limit / collision fault on the session is raised to the caller."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    sessions[mg].has_failed = True
    sessions[mg].failure_exception = MotionError("0@ur10e", "joint_limit")
    with pytest.raises(MotionError):
        async for _ in jogger:
            break


@pytest.mark.asyncio
async def test_a_lost_connection_surfaces_as_an_exception_through_the_loop():
    """A dropped jogging connection is raised, not swallowed."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    sessions[mg].has_failed = True
    sessions[mg].failure_exception = RuntimeError("connection reset")
    with pytest.raises(RuntimeError, match="connection reset"):
        async for _ in jogger:
            break


@pytest.mark.asyncio
async def test_a_stop_condition_ends_the_loop_without_raising():
    """A fired stop condition ends iteration normally and is reported by name."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    sessions[mg].stop_condition_triggered = "workspace_limit"
    iterations = 0
    async for _ in jogger:
        iterations += 1
        break
    assert iterations == 0  # loop ended before yielding any state
    assert jogger.stop_condition_triggered == "workspace_limit"


# ---------------------------------------------------------------------------
# TCP jogging streams a pose as a 6-DoF Cartesian waypoint
# ---------------------------------------------------------------------------


def test_tcp_jogging_streams_a_pose_as_a_cartesian_waypoint():
    """jog_tcp(set_target(Pose)) pushes [x, y, z, rx, ry, rz] to the robot."""
    from nova.types import Pose

    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", tcp="Flange")
    jogger.set_target(Pose(500, 200, 300, 0, 3.14, 0))
    sessions[mg].update_chunk.assert_called_once_with(
        steps=[[500, 200, 300, 0, 3.14, 0]],
        dt_ms=0.0,
        first_timestamp_ms=None,
        timestamp_offset_steps=1,
        extend_buffer=False,
    )


def test_tcp_jogging_streams_a_chunk_of_future_poses():
    """jog_tcp accepts a chunk of [x, y, z, rx, ry, rz] steps for smoother motion."""
    chunk = [[500.0 + i, 200.0, 300.0, 0.0, 3.14, 0.0] for i in range(4)]
    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", tcp="Flange")
    jogger.set_chunk(chunk, dt_ms=33.0)
    sessions[mg].update_chunk.assert_called_once_with(
        steps=chunk,
        dt_ms=33.0,
        first_timestamp_ms=0,
        extend_buffer=True,
    )


def test_with_a_live_buffer_the_tcp_jogger_still_sends_each_pose_alone():
    from nova.types import Pose

    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", pipeline=(LiveBuffer(window_ms=30.0),))

    for i, x in enumerate((500, 510, 520)):
        _tick(jogger, {mg: sessions[mg]}, i * 10.0)
        jogger.set_target(Pose(x, 200, 300, 0, 3.14, 0))

    calls = [call.kwargs for call in sessions[mg].update_chunk.call_args_list]
    assert [kwargs["steps"] for kwargs in calls] == [
        [[500, 200, 300, 0, 3.14, 0]],
        [[510, 200, 300, 0, 3.14, 0]],
        [[520, 200, 300, 0, 3.14, 0]],
    ]
    assert all(kwargs["extend_buffer"] is False for kwargs in calls)


# ---------------------------------------------------------------------------
# Context lifecycle: entering starts the robots, exiting stops them
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_context_starts_waits_for_and_stops_every_session():
    """The context owns the complete lifecycle of every robot session."""
    jogger, (left, right), sessions = _build_joint_jogger("0@ur5e-left", "0@ur5e-right")
    with _no_estop_no_rerun():
        async with jogger:
            for mg in (left, right):
                sessions[mg].start.assert_awaited_once()
                sessions[mg].wait_ready.assert_awaited_once()
    for mg in (left, right):
        sessions[mg].stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_state_returns_the_current_robot_state():
    """state() reports the live robot state from the session."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    sessions[mg].current_state = MagicMock(joints=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    with _no_estop_no_rerun():
        async with jogger:
            state = jogger.state()
    assert state is sessions[mg].current_state


# ---------------------------------------------------------------------------
# Shutdown: waypoints already accepted are owed to the caller
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_clean_exit_lets_the_sent_waypoints_finish_before_stopping():
    """Leaving the loop must not cut the commanded path short.

    Everything the buffer sends lies in the future, so cancelling the session the
    instant the loop ends discards up to a full buffer of motion the caller
    already asked for — the robot stops part-way along the path.
    """
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    order: list[str] = []
    sessions[mg].drain = AsyncMock(side_effect=lambda *a, **k: order.append("drain"))
    sessions[mg].stop = AsyncMock(side_effect=lambda *a, **k: order.append("stop"))

    with _no_estop_no_rerun():
        async with jogger:
            pass

    assert order == ["drain", "stop"]


@pytest.mark.asyncio
async def test_a_fault_stops_the_robot_without_draining():
    """An exception on the way out means stop now, not after the horizon."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")

    async def fail_inside() -> None:
        async with jogger:
            raise RuntimeError("boom")

    with _no_estop_no_rerun(), pytest.raises(RuntimeError, match="boom"):
        await fail_inside()

    sessions[mg].drain.assert_not_awaited()
    sessions[mg].stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_a_fired_stop_condition_stops_the_robot_without_draining():
    """A stop condition asked the robot to stop; finishing the path would defy it."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    sessions[mg].stop_condition_triggered = "force_exceeded"

    with _no_estop_no_rerun():
        async with jogger:
            pass

    sessions[mg].drain.assert_not_awaited()
    sessions[mg].stop.assert_awaited_once()
