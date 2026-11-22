"""Live-target pushes, commanded-history bookkeeping and Rerun tracking alignment."""

from __future__ import annotations

from itertools import pairwise
from typing import TYPE_CHECKING, cast

import pytest

from novapolicy.jogging.jogger import TcpJogger

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from novapolicy.jogging.waypoint_session import WaypointJoggingSession
    from novapolicy.rerun import PolicyRerunLogger


class _FakeSession:
    """Just the surface the live-target push path reads."""

    def __init__(self, mode: str = "cartesian") -> None:
        self.mode = mode
        self.single_step_dt_ms = 50.0
        self.min_chunk_horizon_ms = 500.0


def _jogger(mode: str = "cartesian") -> tuple[TcpJogger, object, _FakeSession]:
    jogger = TcpJogger.__new__(TcpJogger)
    session = _FakeSession(mode)
    mg = object()
    jogger._mg_list = cast("list[MotionGroup]", [mg])
    jogger._multi = False
    jogger._sessions = cast("dict[MotionGroup, WaypointJoggingSession]", {mg: session})
    jogger._rerun = None
    jogger._commanded = {}
    jogger._timeline_ms = 0.0
    # Drive the session clock directly: elapsed reads the per-tick sample.
    jogger._loop_t0 = 0.0
    jogger._ack0_ms = 0.0
    jogger._tick_ms = 0.0
    return jogger, mg, session


def test_commanded_at_resolves_the_value_for_a_past_timestamp() -> None:
    """Tracking compares against what was commanded for the state's own moment."""
    jogger, _mg, _session = _jogger()
    # Recording is gated on an active Rerun logger; only its presence matters.
    jogger._rerun = cast("PolicyRerunLogger", object())

    jogger._record_commanded("mg", [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]], 50.0, 1000)

    assert jogger._commanded_at("mg", 1000) == [0.0, 0.0, 0.0]
    assert jogger._commanded_at("mg", 1050) == [10.0, 0.0, 0.0]
    # halfway between the two waypoints
    assert jogger._commanded_at("mg", 1025) == [5.0, 0.0, 0.0]
    # a stale state from before the chunk clamps to its first waypoint
    assert jogger._commanded_at("mg", 900) == [0.0, 0.0, 0.0]


def test_commanded_history_prefers_the_newest_chunk() -> None:
    """A replacement chunk overwrites the old command at the same timestamp."""
    jogger, _mg, _session = _jogger()
    jogger._rerun = cast("PolicyRerunLogger", object())

    jogger._record_commanded("mg", [[1.0, 0.0, 0.0]], 0.0, 1000)
    jogger._record_commanded("mg", [[2.0, 0.0, 0.0]], 0.0, 1000)

    assert jogger._commanded_at("mg", 1000) == [2.0, 0.0, 0.0]


def test_commanded_history_is_not_recorded_without_rerun() -> None:
    """The history exists only to feed the plot; it must cost nothing otherwise."""
    jogger, _mg, _session = _jogger()

    jogger._record_commanded("mg", [[1.0, 0.0, 0.0]], 0.0, 1000)

    assert jogger._commanded_at("mg", 1000) is None


class _FakeRerun:
    """Records what would have been sent to Rerun."""

    def __init__(self) -> None:
        self.tcp: list[tuple[str, list[float], float | None]] = []
        self.joint: list[tuple[str, list[float], float | None]] = []
        self.robot: list[tuple[str, float]] = []

    def log_robot_state(self, mg_id, state, *, at):
        _ = state
        self.robot.append((mg_id, at))

    def log_tcp_tracking(self, mg_id, target, actual, step, *, at=None):
        _ = actual, step
        self.tcp.append((mg_id, list(target), at))

    def log_joint_tracking(self, mg_id, target, actual, step, *, at=None):
        _ = actual, step
        self.joint.append((mg_id, list(target), at))


def test_tracking_is_logged_per_state_at_that_state_s_own_instant() -> None:
    """Each state packet gets its own entry, stamped when it was generated.

    Sampling once per control tick instead re-reads the cached pose, which is
    the same packet for ~90ms during a delivery burst, and draws the tracking
    error as a flat shelf.
    """
    jogger, _mg, _session = _jogger()
    fake = _FakeRerun()
    jogger._rerun = cast("PolicyRerunLogger", fake)
    jogger._record_commanded("mg", [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]], 50.0, 1000)

    state = object()
    # a burst: three packets generated 8ms apart, delivered together
    for offset, generated in ((0, 100.0), (8, 100.008), (16, 100.016)):
        jogger._log_state_tracking("mg", "cartesian", state, 1000 + offset, generated)

    assert [entry[2] for entry in fake.tcp] == [100.0, 100.008, 100.016]
    # each is the command interpolated for its own server timestamp, so no two
    # consecutive entries are the same point
    plotted = [entry[1] for entry in fake.tcp]
    assert plotted[0] != plotted[1] != plotted[2]
    assert plotted[1][0] == pytest.approx(1.6)


def test_tracking_is_skipped_when_nothing_was_commanded_for_that_instant() -> None:
    """No command history for the packet means no misleading entry."""
    jogger, _mg, _session = _jogger()
    fake = _FakeRerun()
    jogger._rerun = cast("PolicyRerunLogger", fake)

    jogger._log_state_tracking("mg", "cartesian", object(), 1000, 100.0)

    assert fake.tcp == []
    assert fake.robot == [("mg", 100.0)]  # the 3D robot still moves


def test_joint_mode_tracking_uses_the_joint_logger() -> None:
    jogger, _mg, _session = _jogger(mode="joint")
    fake = _FakeRerun()
    jogger._rerun = cast("PolicyRerunLogger", fake)
    jogger._record_commanded("mg", [[1.0, 2.0, 3.0]], 0.0, 1000)

    jogger._log_state_tracking("mg", "joint", object(), 1000, 100.0)

    assert fake.joint and not fake.tcp
    assert fake.joint[0][1] == [1.0, 2.0, 3.0]


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


def test_nothing_is_buffered_or_shaped_unless_passed() -> None:
    """Opt-in modules: the pipeline is empty by default."""
    import inspect

    from novapolicy.jogging.jogger import jog_joints, jog_tcp

    for factory in (jog_joints, jog_tcp):
        params = inspect.signature(factory).parameters
        assert tuple(params["pipeline"].default) == (), factory.__name__
        assert "live_buffer" not in params, factory.__name__
        assert "stages" not in params, factory.__name__


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


def test_the_tracking_record_follows_what_the_session_queued() -> None:
    """With a LiveBuffer the session commands a target about the window later; the tracking
    record must show that, not the raw target."""
    from novapolicy.jogging.waypoints import PendingChunk

    jogger, mg, session = _push_jogger()

    class _Rerun:
        def log_action_chunk(self, chunk: object, step: int) -> None:
            pass

    jogger._rerun = _Rerun()
    jogger._loop_t0 = 0.0
    session.queued_chunk = PendingChunk(
        steps=[[1.0] * 6, [2.0] * 6], dt_ms=50.0, first_timestamp_ms=2_000
    )
    jogger._set_live_target(mg, [9.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    history = jogger._commanded["mg"]
    assert min(history) == 2_000
    assert history[2_000][0] == 1.0
