"""Live-target pushes and per-state-packet Rerun tracking."""

from __future__ import annotations

from itertools import pairwise
from typing import TYPE_CHECKING, cast
from unittest.mock import AsyncMock, MagicMock

import pytest

from novapolicy.jogging.jogger import TcpJogger
from novapolicy.rerun.logger import PolicyRerunLogger

if TYPE_CHECKING:
    from collections.abc import Callable

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.transport.session import WaypointJoggingSession


class _FakeSession:
    """Just the surface the live-target push path reads."""

    def __init__(self, mode: str = "cartesian") -> None:
        self.mode = mode
        self.single_step_dt_ms = 50.0


def _jogger(mode: str = "cartesian") -> tuple[TcpJogger, object, _FakeSession]:
    jogger = TcpJogger.__new__(TcpJogger)
    session = _FakeSession(mode)
    mg = object()
    jogger._mg_list = cast("list[MotionGroup]", [mg])
    jogger._multi = False
    jogger._sessions = cast("dict[MotionGroup, WaypointJoggingSession]", {mg: session})
    jogger._rerun = None
    jogger._timeline_ms = 0.0
    # Drive the session clock directly: elapsed reads the per-tick sample.
    jogger._loop_t0 = 0.0
    jogger._ack0_ms = 0.0
    jogger._tick_ms = 0.0
    return jogger, mg, session


class _TrackedSession:
    """What per-packet tracking reads off a session: its mode and what it sent when."""

    motion_group_id = "mg"

    def __init__(self, mode: str = "cartesian", sent: list[float] | None = None) -> None:
        self.mode = mode
        self.sent = sent  # what was sent, ramping by 1 per ms from server time 1000
        self.observers: list[Callable[[object, int, float], None]] = []

    def commanded_at(self, server_ms: int) -> list[float] | None:
        if self.sent is None:
            return None
        return [value + server_ms - 1000 for value in self.sent]

    def add_state_observer(self, observer: Callable[[object, int, float], None]) -> None:
        self.observers.append(observer)

    def remove_state_observer(self, observer: Callable[[object, int, float], None]) -> None:
        self.observers.remove(observer)


_Entry = tuple[str, list[float], int, float | None]


class _RecordingLogger(PolicyRerunLogger):
    """A Rerun logger that records its per-packet writes instead of sending them."""

    def __init__(self) -> None:
        self._step = 0
        self._observed = []
        self.tcp: list[_Entry] = []
        self.joint: list[_Entry] = []
        self.robot: list[tuple[str, float]] = []

    def log_robot_state(self, mg_id: str, state: RobotState, *, at: float) -> None:
        self.robot.append((mg_id, at))

    def log_tcp_tracking(
        self, mg_id: str, target: list[float], actual: RobotState, step: int, *, at=None
    ) -> None:
        self.tcp.append((mg_id, list(target), step, at))

    def log_joint_tracking(
        self, mg_id: str, target: list[float], actual: RobotState, step: int, *, at=None
    ) -> None:
        self.joint.append((mg_id, list(target), step, at))


_STATE = cast("RobotState", object())


def test_tracking_is_logged_per_state_at_that_state_s_own_instant() -> None:
    """Each state packet gets its own entry, stamped when it was generated, against what was
    sent for that packet's own server timestamp.

    Sampling once per control tick instead re-reads the cached pose, the same packet for ~90ms
    during a delivery burst, and draws the tracking error as a flat shelf.
    """
    logger = _RecordingLogger()
    session = cast("WaypointJoggingSession", _TrackedSession(sent=[0.0]))

    # a burst: three packets generated 8ms apart, delivered together
    for offset, generated in ((0, 100.0), (8, 100.008), (16, 100.016)):
        logger.log_state_packet(session, _STATE, 1000 + offset, generated)

    assert [entry[3] for entry in logger.tcp] == [100.0, 100.008, 100.016]
    assert [entry[1][0] for entry in logger.tcp] == [0.0, 8.0, 16.0]


def test_tracking_is_skipped_when_nothing_was_sent_yet() -> None:
    """No sent history means no misleading entry; the 3D robot still moves."""
    logger = _RecordingLogger()

    logger.log_state_packet(cast("WaypointJoggingSession", _TrackedSession()), _STATE, 1000, 100.0)

    assert logger.tcp == []
    assert logger.robot == [("mg", 100.0)]


def test_joint_mode_tracking_uses_the_joint_logger_at_the_latest_observed_step() -> None:
    logger = _RecordingLogger()
    logger._step = 7  # what log_observation last saw
    session = cast("WaypointJoggingSession", _TrackedSession("joint", sent=[1.0, 2.0]))

    logger.log_state_packet(session, _STATE, 1000, 100.0)

    assert logger.joint == [("mg", [1.0, 2.0], 7, 100.0)]
    assert not logger.tcp


@pytest.mark.asyncio
async def test_streaming_observes_every_session_until_it_stops(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Executor and joggers alike: start_streaming hooks each session's state packets,
    stop_streaming unhooks them."""
    from novapolicy.rerun.context import RerunContext  # imports rerun, which may be skipped

    streamer = MagicMock(start=MagicMock(), stop=AsyncMock())
    monkeypatch.setattr(
        "novapolicy.rerun.streaming.StateStreamer", MagicMock(return_value=streamer)
    )
    logger = _RecordingLogger()
    logger._ctx = RerunContext(start_time=0.0, recording=None)
    logger._streamer = None
    logger._recording = None
    logger._sink = MagicMock()
    session = _TrackedSession(sent=[3.0])

    logger.start_streaming({"mg": cast("WaypointJoggingSession", session)})
    (observer,) = session.observers
    observer(_STATE, 1000, 100.0)
    await logger.stop_streaming()

    assert logger.tcp == [("mg", [3.0], 0, 100.0)]
    assert session.observers == []


# ---------------------------------------------------------------------------
# The live-target push: one waypoint, exactly where the target is. Buffering it into
# a horizon is the LiveBuffer pipeline module's job, in the session (see the LiveBuffer
# tests in tests/api/test_waypoint_session.py).
# ---------------------------------------------------------------------------


class _PushSession(_FakeSession):
    """A session that accepts pushes, for driving ``_set_live_target``."""

    num_joints = 6
    estimated_server_timestamp_ms = 0

    def __init__(self, mode: str = "cartesian") -> None:
        super().__init__()
        self.mode = mode
        self.chunks: list[dict] = []

    queued_chunk = None  # what the session queued after its chunk modules

    def update_chunk(self, **kwargs) -> None:
        self.chunks.append(kwargs)

    def check_steps(self, steps: list[list[float]]) -> None:
        pass


class _NamedMotionGroup:
    """``_set_live_target`` reaches the Rerun helpers, which key off the id."""

    id = "mg"


def _push_jogger():
    """A jogger wired to record what the live-target path pushes to its session."""
    jogger, _stub, _session = _jogger()
    mg = cast("MotionGroup", _NamedMotionGroup())
    session = _PushSession()
    jogger._mg_list = [mg]
    jogger._sessions = cast("dict[MotionGroup, WaypointJoggingSession]", {mg: session})
    return jogger, mg, session


def test_a_live_target_is_sent_alone_and_unmodified() -> None:
    """Exactly one waypoint, exactly where the target is, every time."""
    jogger, mg, session = _push_jogger()

    for i in range(3):
        jogger._tick_ms = i * 10.0
        jogger._set_live_target(mg, [12.0 + i, 34.0, 56.0, 0.0, 0.0, 0.0])

    assert [chunk["steps"] for chunk in session.chunks] == [
        [[12.0 + i, 34.0, 56.0, 0.0, 0.0, 0.0]] for i in range(3)
    ]
    assert all(chunk["dt_ms"] == 0.0 for chunk in session.chunks)
    assert all(chunk["extend_buffer"] is False for chunk in session.chunks)


def test_nothing_is_buffered_or_eased_unless_passed() -> None:
    """The default pipeline holds only the built-ins: no LiveBuffer, no EaseIn."""
    import inspect

    from novapolicy.jogging import EaseIn, LiveBuffer, default_pipeline
    from novapolicy.jogging.jogger import jog_joints, jog_tcp

    assert not any(isinstance(m, (LiveBuffer, EaseIn)) for m in default_pipeline())
    for factory in (jog_joints, jog_tcp):
        params = inspect.signature(factory).parameters
        assert params["pipeline"].default is None, factory.__name__


@pytest.mark.parametrize("ack0_ms", [8.0, 8.6])  # sessions start near 0 ms
def test_live_targets_carry_the_loop_time_in_exact_grid_steps(ack0_ms: float) -> None:
    """A LiveBuffer times targets by observed_at_ms on its 10 ms grid: converting the loop time
    to ms must not land a target 1 ms short (``0.29 * 1000`` truncates to 289), a whole grid
    step there, whether or not the loop's anchor is a whole ms."""
    jogger, mg, session = _push_jogger()
    jogger._loop_t0 = 0.0
    jogger._ack0_ms = ack0_ms
    for i in range(1000):  # 10 s: 2.01 s is the first to truncate from 8 ms
        jogger._tick_ms = ack0_ms + i * 10.0 + 0.3  # server ticks fall anywhere in a grid step
        jogger._set_live_target(mg, [float(i), 0.0, 0.0, 0.0, 0.0, 0.0])
    produced = [chunk["observed_at_ms"] for chunk in session.chunks]
    assert all(b - a == 10 for a, b in pairwise(produced)), produced


@pytest.mark.parametrize("ack0_ms", [8.0, 8.6])
def test_chunks_start_on_one_grid_in_exact_steps(ack0_ms: float) -> None:
    """Chunks re-sent every loop iteration must start on one 10 ms grid: a chunk 1 ms early
    leaves a waypoint 1 ms after the previous chunk's, which the follower takes as a speed jump."""
    jogger, _mg, session = _push_jogger()
    jogger._loop_t0 = 0.0
    jogger._ack0_ms = ack0_ms
    for i in range(1000):
        jogger._tick_ms = ack0_ms + i * 10.0 + 0.3
        jogger.set_chunk([[float(i), 0.0, 0.0, 0.0, 0.0, 0.0]] * 3, dt_ms=10.0)
    starts = [chunk["first_timestamp_ms"] for chunk in session.chunks]
    assert all(b - a == 10 for a, b in pairwise(starts)), starts
