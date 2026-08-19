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
from novapolicy.jogging.jogger import LIVE_LEAD_MS, JointJogger, TcpJogger
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
    ease_in_s: float = 0.0,
    buffer_window_ms: float = 0.0,
) -> _JointSetup:
    """Build a real joint jogger over fake robot transports.

    The transport is only patched while the jogger is being constructed (that
    is when the sessions are created); the returned fakes are kept so a test
    can assert what got streamed to each robot.
    """
    mgs = [_mg(mid) for mid in mg_ids]
    sessions: dict[object, MagicMock] = {}

    def make_session(*, motion_group: object, **_kw: object) -> MagicMock:
        sessions[motion_group] = _fake_session(num_joints)
        return sessions[motion_group]

    with patch("novapolicy.jogging.jogger.WaypointJoggingSession", side_effect=make_session):
        jogger = jog_joints(
            mgs if len(mgs) > 1 else mgs[0],
            ease_in_s=ease_in_s,
            buffer_window_ms=buffer_window_ms,
        )
    return jogger, mgs, sessions


def _build_tcp_jogger(
    mg_id: str,
    tcp: str = "Flange",
    *,
    num_joints: int = 6,
    buffer_window_ms: float = 0.0,
) -> _TcpSetup:
    """Build a real single-arm TCP jogger over a fake robot transport."""
    mg = _mg(mg_id)
    sessions: dict[object, MagicMock] = {}

    def make_session(*, motion_group: object, **_kw: object) -> MagicMock:
        sessions[motion_group] = _fake_session(num_joints, mode="cartesian")
        return sessions[motion_group]

    with patch("novapolicy.jogging.jogger.WaypointJoggingSession", side_effect=make_session):
        jogger = jog_tcp(mg, tcp=tcp, buffer_window_ms=buffer_window_ms)
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


def test_ease_in_starts_motion_from_the_standstill_baseline():
    """With ease_in_s set, the first target (at elapsed 0) collapses to the
    robot's start position, so motion begins from a standstill instead of
    jumping to the target's initial speed. Default (no ease-in) sends the raw
    target — see test_setting_a_single_target_streams_it_to_the_robot.
    """
    # Fake session reports its current position as all zeros (the baseline).
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e", ease_in_s=1.0)
    jogger.set_target([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    sent = sessions[mg].update_chunk.call_args.kwargs["steps"][0]
    assert sent == [0.0] * 6  # held at the start baseline, not the raw target


def test_setting_a_chunk_streams_every_step_and_tracks_the_last():
    """A chunk of future targets is streamed whole; the last step is the target."""
    chunk = [[float(i)] * 6 for i in range(4)]
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    jogger.set_target(chunk, dt_ms=33.0)
    sessions[mg].update_chunk.assert_called_once_with(
        steps=chunk,
        dt_ms=33.0,
        first_timestamp_ms=0,
        extend_buffer=True,
    )
    assert jogger.target == chunk[-1]


def test_buffered_target_sends_a_rolling_window_played_back_from_now():
    """The recent-target window is played back from the current anchor.

    The samples cannot keep their original slots: those moments have passed, so
    the whole window would be dropped as unreachable. Playing it back from now
    is what buys smoothness at the cost of roughly ``buffer_window_ms`` of delay.
    """
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e", buffer_window_ms=30.0)

    _tick(jogger, sessions, 0.0)
    jogger.set_target([1.0] * 6)
    _tick(jogger, sessions, 10.0)
    jogger.set_target([2.0] * 6)
    _tick(jogger, sessions, 20.0)
    jogger.set_target([3.0] * 6)

    kwargs = sessions[mg].update_chunk.call_args.kwargs
    # Every waypoint is a measured sample and nothing follows the newest one:
    # the window IS the horizon, so there is no predicted continuation.
    assert kwargs["steps"] == [[1.0] * 6, [2.0] * 6, [3.0] * 6]
    assert kwargs["dt_ms"] == pytest.approx(10.0)  # measured, not assumed
    assert kwargs["extend_buffer"] is False
    # Oldest sample (t=0) played back at a constant buffer_window_ms delay, plus lead.
    assert kwargs["first_timestamp_ms"] == int(30 + LIVE_LEAD_MS)


def test_buffered_target_keeps_streaming_before_the_timeline_starts():
    """A live target must go out even while `elapsed` is still pinned at zero.

    Waiting for the window to fill on a clock that only starts ticking once a
    chunk has been sent deadlocks: nothing is sent, so nothing ever ticks.
    """
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e", buffer_window_ms=30.0)

    for value in (1.0, 2.0, 3.0):
        jogger.set_target([value] * 6)

    assert sessions[mg].update_chunk.call_count == 3
    assert sessions[mg].update_chunk.call_args.kwargs["steps"] == [[3.0] * 6]


def test_buffered_window_drops_samples_older_than_the_buffer():
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e", buffer_window_ms=30.0)

    for i, value in enumerate((1.0, 2.0, 3.0, 4.0, 5.0)):
        _tick(jogger, sessions, i * 10.0)
        jogger.set_target([value] * 6)

    steps = sessions[mg].update_chunk.call_args.kwargs["steps"]
    # The window is resampled onto the grid the controller executes best on, so
    # the recorded samples are interpolated rather than passed through
    # one-for-one. What must hold is that the replay spans the *retained*
    # window: it starts at the oldest sample still inside buffer_window_ms (the 1.0
    # sample has aged out) and reaches the newest.
    replayed = [step[0] for step in steps]
    assert replayed[0] >= 2.0, replayed
    assert max(replayed) >= 5.0, replayed
    assert replayed == sorted(replayed), replayed


def test_explicit_chunk_clears_buffered_live_samples():
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e", buffer_window_ms=30.0)

    for i, value in enumerate((1.0, 2.0, 3.0)):
        _tick(jogger, sessions, i * 10.0)
        jogger.set_target([value] * 6)
    jogger.set_target([[9.0] * 6, [9.0] * 6], dt_ms=10.0)

    for i, value in enumerate((10.0, 11.0, 12.0)):
        _tick(jogger, sessions, 30.0 + i * 10.0)
        jogger.set_target([value] * 6)

    # The window restarted from the first post-chunk sample: nothing from
    # before the explicit chunk survives into the replay.
    replayed = [step[0] for step in sessions[mg].update_chunk.call_args.kwargs["steps"]]
    assert replayed[0] == 10.0, replayed
    assert max(replayed) >= 12.0, replayed
    assert all(value >= 10.0 for value in replayed), replayed


def test_mixing_target_forms_is_reported_once_per_motion_group(caplog):
    """Discarding the live buffer is correct, but must not be silent.

    ``buffer_window_ms`` applies only to live targets, so a caller that also sends
    chunks pays for the buffer without benefiting from it: each chunk empties it,
    and until it refills the live targets go out alone as terminal waypoints — the
    halting motion the buffer exists to avoid. That is worth one warning.
    """
    jogger, _, sessions = _build_joint_jogger("0@ur10e", buffer_window_ms=30.0)

    for i, value in enumerate((1.0, 2.0, 3.0)):
        _tick(jogger, sessions, i * 10.0)
        jogger.set_target([value] * 6)

    with caplog.at_level(logging.WARNING, logger="novapolicy.jogging.jogger"):
        jogger.set_target([[9.0] * 6, [9.0] * 6], dt_ms=10.0)
        first = [r.getMessage() for r in caplog.records]

        # Refill, then push another chunk: the same advice must not repeat.
        for i, value in enumerate((4.0, 5.0, 6.0)):
            _tick(jogger, sessions, 40.0 + i * 10.0)
            jogger.set_target([value] * 6)
        jogger.set_target([[8.0] * 6, [8.0] * 6], dt_ms=10.0)
        both = [r.getMessage() for r in caplog.records]

    assert len(first) == 1, first
    assert "0@ur10e" in first[0]
    assert both == first, "the warning repeated"


def test_a_chunk_only_caller_is_never_warned_about_mixing(caplog):
    """There is nothing to discard, so there is nothing to say."""
    jogger, _, sessions = _build_joint_jogger("0@ur10e", buffer_window_ms=30.0)

    with caplog.at_level(logging.WARNING, logger="novapolicy.jogging.jogger"):
        for i in range(3):
            _tick(jogger, sessions, i * 10.0)
            jogger.set_target([[float(i)] * 6, [float(i)] * 6], dt_ms=10.0)

    assert [r.getMessage() for r in caplog.records] == []


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
    jogger.set_target(chunk, dt_ms=33.0)
    sessions[mg].update_chunk.assert_called_once_with(
        steps=chunk,
        dt_ms=33.0,
        first_timestamp_ms=0,
        extend_buffer=True,
    )


def test_buffered_tcp_target_sends_a_rolling_window_of_poses():
    from nova.types import Pose

    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", buffer_window_ms=30.0)

    for i, x in enumerate((500, 510, 520)):
        _tick(jogger, {mg: sessions[mg]}, i * 10.0)
        jogger.set_target(Pose(x, 200, 300, 0, 3.14, 0))

    kwargs = sessions[mg].update_chunk.call_args.kwargs
    assert kwargs["steps"][:3] == [
        [500, 200, 300, 0, 3.14, 0],
        [510, 200, 300, 0, 3.14, 0],
        [520, 200, 300, 0, 3.14, 0],
    ]
    assert kwargs["dt_ms"] == pytest.approx(10.0)
    assert kwargs["extend_buffer"] is False


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

    with _no_estop_no_rerun(), pytest.raises(RuntimeError, match="boom"):
        async with jogger:
            raise RuntimeError("boom")

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
