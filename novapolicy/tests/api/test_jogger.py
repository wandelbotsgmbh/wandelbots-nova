"""Behavioural tests for jog_joints().

The jogger lets you stream joint targets to one or more robots and surfaces
faults through its ``async for`` loop. These tests state that contract through
the public ``jog_joints()`` API, substituting only the robot transport
(``ActionChunkSession``) — they never reach into the jogger's internals.
"""

from __future__ import annotations

import asyncio
import contextlib
from functools import partial
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from novapolicy.jogging import jog_joints, jog_tcp
from novapolicy.jogging.jogger import JointJogger, TcpJogger
from novapolicy.pipeline.modules.servo_horizon import ServoHorizon
from novapolicy.tests.doubles import fake_session as _fake_session, mg as _mg
from novapolicy.types import ActionChunkConfig, MotionError

_JOGGER = "novapolicy.jogging.jogger"
_LIFECYCLE = "novapolicy.transport.lifecycle"

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
        patch(f"{_LIFECYCLE}.EstopMonitor", return_value=estop),
        patch("novapolicy.rerun._is_rerun_active", return_value=False),
    ):
        yield


_JointSetup = tuple[JointJogger, list[MagicMock], dict[object, MagicMock]]
_TcpSetup = tuple[TcpJogger, MagicMock, dict[object, MagicMock]]


def _build_joint_jogger(
    *mg_ids: str,
    num_joints: int = 6,
    pipeline: tuple[object, ...] = (),
    start_joint_position: list[float] | None = None,
    config: ActionChunkConfig | None = None,
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
        sessions[motion_group].motion_group_id = motion_group.id  # type: ignore[attr-defined]
        sessions[motion_group].pipeline = kw.get("pipeline")
        return sessions[motion_group]

    with patch("novapolicy.transport.lifecycle.ActionChunkSession", side_effect=make_session):
        jogger = jog_joints(
            mgs if len(mgs) > 1 else mgs[0],
            pipeline=pipeline,
            start_joint_position=start_joint_position,
            config=config,
        )
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

    with patch("novapolicy.transport.lifecycle.ActionChunkSession", side_effect=make_session):
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
        session.estimated_server_timestamp_ms = elapsed_ms


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


@pytest.mark.parametrize(
    ("config", "lead_ms"), [(None, 100), (ActionChunkConfig(max_expected_latency_ms=150), 150)]
)
def test_targets_and_chunks_are_due_max_expected_latency_ms_after_the_loop_time(config, lead_ms):
    """The joggers take their lead from ActionChunkConfig, as a policy's chunks do."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e", config=config)
    _tick(jogger, sessions, 2_000.0)
    jogger.set_target([1.0] * 6)
    jogger.set_chunk([[1.0] * 6, [2.0] * 6], dt_ms=50.0)
    starts = [c.kwargs["first_timestamp_ms"] for c in sessions[mg].update_chunk.call_args_list]
    assert starts == [2_000 + lead_ms, 2_000 + lead_ms]


def test_the_pipeline_is_handed_to_every_session():
    """Opt-in modules (e.g. ServoHorizon, EaseIn) run in the sessions; the jogger does not shape
    targets itself, and a live target goes out unchanged by default."""
    from novapolicy import EaseIn

    pipeline = (ServoHorizon(), EaseIn(duration_s=1.0))
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


_single = partial(_build_joint_jogger, "0@ur10e")
_dual = partial(_build_joint_jogger, "0@ur5e-left", "0@ur5e-right")
_tcp = partial(_build_tcp_jogger, "0@ur10e")


@pytest.mark.parametrize(
    ("build", "push", "error", "match"),
    [
        # The split exists so a chunk cannot arrive where a live target's path applies:
        # accepting it there meant the buffer was silently irrelevant for that push.
        (_single, lambda j: j.set_target([[1.0] * 6, [2.0] * 6]), TypeError, "set_chunk"),
        # And the reverse, so dt_ms is never applied to something without steps.
        (_single, lambda j: j.set_chunk([1.0] * 6, dt_ms=10.0), TypeError, "set_target"),
        # dt_ms is the step spacing; there is no meaningful default for it.
        (_single, lambda j: j.set_chunk([[1.0] * 6, [2.0] * 6], dt_ms=0.0), ValueError, "dt_ms"),
        # An empty list has no shape to infer and no waypoints to send.
        (_single, lambda j: j.set_chunk([], dt_ms=10.0), TypeError, "empty"),
        # Same split on the TCP jogger, where the live form is a Pose.
        (
            _tcp,
            lambda j: j.set_target([[500.0, 200.0, 300.0, 0.0, 3.14, 0.0]]),
            TypeError,
            "set_chunk",
        ),
        # A 3-value target for a 6-joint robot.
        (_single, lambda j: j.set_target([1.0, 2.0, 3.0]), ValueError, "expects 6"),
        # Two robots need a dict target; a bare list is a usage error.
        (_dual, lambda j: j.set_target([1.0] * 6), TypeError, "dict"),
        # A value that is not finite never reaches the robot.
        (_single, lambda j: j.set_target([float("nan")] * 6), ValueError, "not finite"),
        # Every arm is checked before any is sent: a bad second arm leaves the first unsent.
        (
            _dual,
            lambda j: j.set_target(dict(zip(j._mg_list, ([0.0] * 6, [0.0] * 5), strict=True))),
            ValueError,
            "'0@ur5e-right' expects 6",
        ),
        (
            _dual,
            lambda j: j.set_chunk(
                dict(zip(j._mg_list, ([[0.0] * 6], [[0.0] * 5 + [float("inf")]]), strict=True)),
                dt_ms=10.0,
            ),
            ValueError,
            r"Step 0 for motion group '0@ur5e-right' .*not finite",
        ),
        (_single, lambda j: j.set_chunk([[0.0] * 6, [0.0] * 7], dt_ms=10.0), ValueError, "Step 1"),
    ],
    ids=[
        "chunk-as-target",
        "target-as-chunk",
        "zero-dt",
        "empty-chunk",
        "tcp-chunk-as-target",
        "wrong-joint-count",
        "bare-list-for-two-arms",
        "nan-target",
        "second-arm-wrong-width",
        "second-arm-chunk-not-finite",
        "chunk-step-too-wide",
    ],
)
def test_a_bad_target_is_rejected_before_anything_is_streamed(build, push, error, match):
    jogger, _, sessions = build()
    with pytest.raises(error, match=match):
        push(jogger)
    for session in sessions.values():
        session.update_chunk.assert_not_called()


# ---------------------------------------------------------------------------
# Faults and stops surface through the loop
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("failure", "match"),
    [(MotionError("0@ur10e", "joint_limit"), None), (RuntimeError("connection reset"), "reset")],
    ids=["motion-fault", "lost-connection"],
)
async def test_a_session_failure_surfaces_as_an_exception_through_the_loop(failure, match):
    """A joint-limit / collision fault or a dropped connection is raised, not swallowed."""
    jogger, (mg,), sessions = _build_joint_jogger("0@ur10e")
    sessions[mg].has_failed = True
    sessions[mg].failure_exception = failure
    with pytest.raises(type(failure), match=match):
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
async def test_the_start_move_keeps_the_cells_collision_setups():
    """The start move is planned like any other move, as in the executor."""
    jogger, (mg,), _sessions = _build_joint_jogger("0@ur10e", start_joint_position=[0.1] * 6)
    setup = SimpleNamespace(collision_setups={"cell": object()})
    collision = setup.collision_setups["cell"]
    mg.active_tcp_name = AsyncMock(return_value="Flange")
    mg.get_setup = AsyncMock(return_value=setup)
    mg.plan = AsyncMock(return_value="trajectory")
    mg.execute = AsyncMock()

    with _no_estop_no_rerun():
        async with jogger:
            pass

    mg.execute.assert_awaited_once()
    assert mg.plan.await_args.kwargs["motion_group_setup"] is setup
    assert setup.collision_setups == {"cell": collision}


@pytest.mark.asyncio
async def test_a_failing_rerun_teardown_still_stops_every_session():
    """With a viewer active, Rerun streams every session; its teardown failing must not leave a
    robot session running."""
    jogger, (left, right), sessions = _build_joint_jogger("0@ur5e-left", "0@ur5e-right")
    rerun = MagicMock(initialize=AsyncMock(), stop_streaming=AsyncMock(side_effect=OSError))
    estop = MagicMock(start=AsyncMock(), stop=AsyncMock())
    with (
        patch(f"{_LIFECYCLE}.EstopMonitor", return_value=estop),
        patch("novapolicy.rerun._is_rerun_active", return_value=True),
        patch("novapolicy.rerun.PolicyRerunLogger", return_value=rerun),
        pytest.raises(OSError),
    ):
        async with jogger:
            pass
    (streamed,), _ = rerun.start_streaming.call_args
    assert streamed == {mg.id: sessions[mg] for mg in (left, right)}
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


@pytest.mark.asyncio
async def test_a_cancel_while_draining_stops_the_sessions_and_reaches_the_caller() -> None:
    from novapolicy.transport.lifecycle import SessionLifecycle

    drained = asyncio.Event()

    class _Session:
        stopped = False

        async def drain(self) -> bool:
            drained.set()
            await asyncio.sleep(10)
            return True

        async def stop(self) -> None:
            self.stopped = True

    session = _Session()
    lifecycle = SessionLifecycle(config=ActionChunkConfig())
    lifecycle.sessions["0@ur"] = session  # ty: ignore[invalid-assignment]
    task = asyncio.create_task(lifecycle.close(drain=True))
    await drained.wait()
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert session.stopped
