"""Behavioural tests for jog_joints().

The jogger lets you stream joint targets to one or more robots and surfaces
faults through its ``async for`` loop. These tests state that contract through
the public ``jog_joints()`` API, substituting only the robot transport
(``WaypointJoggingSession``) — they never reach into the jogger's internals.
"""

from __future__ import annotations

import contextlib
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from novapolicy.jogging import jog_joints, jog_tcp
from novapolicy.jogging.jogger import JointJogger, TcpJogger
from novapolicy.types import MotionError, WaypointConfig

_JOGGER = "novapolicy.jogging.jogger"


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
    session.session_elapsed_ms = 0
    session.config = WaypointConfig(min_buffer_ms=30.0)
    session.current_state = MagicMock(joints=(0.0,) * num_joints)
    session.update_chunk = MagicMock()
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.wait_ready = AsyncMock()
    return session


def _build_joint_jogger(*mg_ids: str, num_joints: int = 6, ease_in_s: float = 0.0) -> _JointSetup:
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
        jogger = jog_joints(mgs if len(mgs) > 1 else mgs[0], ease_in_s=ease_in_s)
    return jogger, mgs, sessions


def _build_tcp_jogger(mg_id: str, tcp: str = "Flange", *, num_joints: int = 6) -> _TcpSetup:
    """Build a real single-arm TCP jogger over a fake robot transport."""
    mg = _mg(mg_id)
    sessions: dict[object, MagicMock] = {}

    def make_session(*, motion_group: object, **_kw: object) -> MagicMock:
        sessions[motion_group] = _fake_session(num_joints, mode="cartesian")
        return sessions[motion_group]

    with patch("novapolicy.jogging.jogger.WaypointJoggingSession", side_effect=make_session):
        jogger = jog_tcp(mg, tcp=tcp)
    return jogger, mg, sessions


# ---------------------------------------------------------------------------
# Setting a target streams waypoints to the robot
# ---------------------------------------------------------------------------


def test_setting_a_single_target_streams_it_to_the_robot():
    """A single joint target is pushed to the session as one waypoint step."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    jogger.set_target([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    sessions[mg].update_chunk.assert_called_once_with(
        steps=[[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]], dt_ms=0.0, anchor_offset_steps=1
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
    sessions[mg].update_chunk.assert_called_once_with(steps=chunk, dt_ms=33.0, anchor_ms=0)
    assert jogger.target == chunk[-1]


def test_appending_joint_targets_primes_then_sends_a_rolling_buffer():
    """append_target adds latency, then streams a moving buffered chunk."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")

    assert jogger.append_target([1.0] * 6, dt_ms=10.0) is False
    assert jogger.append_target([2.0] * 6, dt_ms=10.0) is False
    sessions[mg].update_chunk.assert_not_called()

    assert jogger.append_target([3.0] * 6, dt_ms=10.0) is True
    sessions[mg].update_chunk.assert_called_once_with(
        steps=[[1.0] * 6, [2.0] * 6, [3.0] * 6],
        dt_ms=10.0,
        anchor_ms=0,
        extend_buffer=False,
    )

    assert jogger.append_target([4.0] * 6, dt_ms=10.0) is True
    assert sessions[mg].update_chunk.call_args.kwargs["steps"] == [
        [2.0] * 6,
        [3.0] * 6,
        [4.0] * 6,
    ]


def test_set_target_clears_append_target_buffer():
    """Switching to immediate targets must not reuse stale buffered samples."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")

    assert jogger.append_target([1.0] * 6, dt_ms=10.0) is False
    assert jogger.append_target([2.0] * 6, dt_ms=10.0) is False
    assert jogger.append_target([3.0] * 6, dt_ms=10.0) is True
    assert sessions[mg].update_chunk.call_count == 1

    jogger.set_target([9.0] * 6)
    assert sessions[mg].update_chunk.call_count == 2

    assert jogger.append_target([10.0] * 6, dt_ms=10.0) is False
    assert jogger.append_target([11.0] * 6, dt_ms=10.0) is False
    assert sessions[mg].update_chunk.call_count == 2

    assert jogger.append_target([12.0] * 6, dt_ms=10.0) is True
    assert sessions[mg].update_chunk.call_args.kwargs["steps"] == [
        [10.0] * 6,
        [11.0] * 6,
        [12.0] * 6,
    ]


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
        steps=[[500, 200, 300, 0, 3.14, 0]], dt_ms=0.0, anchor_offset_steps=1
    )


def test_tcp_jogging_streams_a_chunk_of_future_poses():
    """jog_tcp accepts a chunk of [x, y, z, rx, ry, rz] steps for smoother motion."""
    chunk = [[500.0 + i, 200.0, 300.0, 0.0, 3.14, 0.0] for i in range(4)]
    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", tcp="Flange")
    jogger.set_target(chunk, dt_ms=33.0)
    sessions[mg].update_chunk.assert_called_once_with(steps=chunk, dt_ms=33.0, anchor_ms=0)


def test_appending_tcp_targets_primes_then_sends_a_rolling_buffer():
    """TCP append_target buffers samples before sending a rolling chunk."""
    from nova.types import Pose

    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", tcp="Flange")

    assert jogger.append_target(Pose(500, 200, 300, 0, 3.14, 0), dt_ms=10.0) is False
    assert jogger.append_target(Pose(510, 200, 300, 0, 3.14, 0), dt_ms=10.0) is False
    sessions[mg].update_chunk.assert_not_called()

    assert jogger.append_target(Pose(520, 200, 300, 0, 3.14, 0), dt_ms=10.0) is True
    sessions[mg].update_chunk.assert_called_once_with(
        steps=[
            [500, 200, 300, 0, 3.14, 0],
            [510, 200, 300, 0, 3.14, 0],
            [520, 200, 300, 0, 3.14, 0],
        ],
        dt_ms=10.0,
        anchor_ms=0,
        extend_buffer=False,
    )


def test_appending_tcp_targets_uses_wall_time_only_to_prime_before_elapsed_advances(monkeypatch):
    """Startup cannot wait for elapsed: first chunk is needed before RUNNING."""
    from nova.types import Pose

    times = iter([0.00, 0.01, 0.02])
    monkeypatch.setattr("novapolicy.jogging.jogger.time.monotonic", lambda: next(times))

    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", tcp="Flange")
    # elapsed stays at zero before the first chunk makes the robot RUNNING.
    jogger._loop_t0 = 0.0  # type: ignore[attr-defined]
    jogger._ack0_ms = 0.0  # type: ignore[attr-defined]
    sessions[mg].session_elapsed_ms = 0

    assert jogger.append_target(Pose(500, 200, 300, 0, 3.14, 0)) is False
    assert jogger.append_target(Pose(510, 200, 300, 0, 3.14, 0)) is False
    assert jogger.append_target(Pose(520, 200, 300, 0, 3.14, 0)) is True

    sessions[mg].update_chunk.assert_called_once_with(
        steps=[
            [500, 200, 300, 0, 3.14, 0],
            [510, 200, 300, 0, 3.14, 0],
            [520, 200, 300, 0, 3.14, 0],
        ],
        dt_ms=10.0,
        anchor_ms=0,
        extend_buffer=False,
    )


def test_append_target_does_not_use_wall_time_after_first_buffered_chunk(monkeypatch):
    """After priming, automatic timing waits for acknowledged jogger time."""
    from nova.types import Pose

    times = iter([0.00, 0.01, 0.02, 0.03, 0.04, 0.05])
    monkeypatch.setattr("novapolicy.jogging.jogger.time.monotonic", lambda: next(times))

    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", tcp="Flange")
    jogger._loop_t0 = 0.0  # type: ignore[attr-defined]
    jogger._ack0_ms = 0.0  # type: ignore[attr-defined]
    sessions[mg].session_elapsed_ms = 0

    assert jogger.append_target(Pose(500, 200, 300, 0, 3.14, 0)) is False
    assert jogger.append_target(Pose(510, 200, 300, 0, 3.14, 0)) is False
    assert jogger.append_target(Pose(520, 200, 300, 0, 3.14, 0)) is True
    sessions[mg].update_chunk.reset_mock()

    assert jogger.append_target(Pose(530, 200, 300, 0, 3.14, 0)) is False
    assert jogger.append_target(Pose(540, 200, 300, 0, 3.14, 0)) is False
    assert jogger.append_target(Pose(550, 200, 300, 0, 3.14, 0)) is False
    sessions[mg].update_chunk.assert_not_called()
    assert len(jogger._target_buffers[mg]) == 3  # type: ignore[attr-defined]


def test_append_target_uses_each_motion_groups_default_buffer_horizon():
    """Default append buffering comes from the matching motion group's session."""
    jogger, (left, right), sessions = _build_joint_jogger("0@ur5e-left", "0@ur5e-right")
    sessions[left].config = WaypointConfig(min_buffer_ms=10.0)
    sessions[right].config = WaypointConfig(min_buffer_ms=30.0)

    sent = jogger.append_target(
        {
            left: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            right: [7.0, 8.0, 9.0, 10.0, 11.0, 12.0],
        },
        dt_ms=10.0,
    )

    assert sent is True
    sessions[left].update_chunk.assert_called_once()
    sessions[right].update_chunk.assert_not_called()


def test_appending_tcp_targets_can_infer_dt_from_jogger_elapsed():
    """After startup, append_target uses acknowledged jogger time between samples."""
    from nova.types import Pose

    jogger, mg, sessions = _build_tcp_jogger("0@ur10e", tcp="Flange")
    jogger._loop_t0 = 0.0  # type: ignore[attr-defined]
    jogger._ack0_ms = 0.0  # type: ignore[attr-defined]

    sessions[mg].session_elapsed_ms = 0
    assert jogger.append_target(Pose(500, 200, 300, 0, 3.14, 0)) is False
    sessions[mg].session_elapsed_ms = 10
    assert jogger.append_target(Pose(510, 200, 300, 0, 3.14, 0)) is False
    sessions[mg].session_elapsed_ms = 20
    assert jogger.append_target(Pose(520, 200, 300, 0, 3.14, 0)) is True

    sessions[mg].update_chunk.assert_called_once_with(
        steps=[
            [500, 200, 300, 0, 3.14, 0],
            [510, 200, 300, 0, 3.14, 0],
            [520, 200, 300, 0, 3.14, 0],
        ],
        dt_ms=10.0,
        anchor_ms=20,
        extend_buffer=False,
    )


# ---------------------------------------------------------------------------
# Context lifecycle: entering starts the robots, exiting stops them
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_entering_the_context_starts_every_session_and_waits_ready():
    """`async with jog_joints(...)` starts each robot and waits until it's ready."""
    jogger, (left, right), sessions = _build_joint_jogger("0@ur5e-left", "0@ur5e-right")
    with _no_estop_no_rerun():
        async with jogger:
            for mg in (left, right):
                sessions[mg].start.assert_awaited_once()
                sessions[mg].wait_ready.assert_awaited_once()


@pytest.mark.asyncio
async def test_exiting_the_context_stops_every_session():
    """Leaving the context tears down each robot's jogging session."""
    jogger, (left, right), sessions = _build_joint_jogger("0@ur5e-left", "0@ur5e-right")
    with _no_estop_no_rerun():
        async with jogger:
            pass
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
