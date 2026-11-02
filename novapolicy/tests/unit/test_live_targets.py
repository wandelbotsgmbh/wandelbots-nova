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
    jogger._target_buffers = {}
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


def test_ease_in_releases_without_a_velocity_step() -> None:
    """The ease blend must be smooth in velocity where it completes.

    A straight ramp reaches full blend rate right up to the moment it finishes
    and then stops dead; that discontinuity is executed as a stumble.
    """
    jogger, mg, _session = _jogger()
    jogger._ease_in_s = 0.5
    jogger._ease_baseline = cast("dict[MotionGroup, list[float]]", {mg: [0.0] * 6})

    # Blend one fixed target through the window and read the resulting motion.
    target = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    positions = []
    for i in range(60):
        jogger._tick_ms = i * 10.0
        positions.append(jogger._ease_steps(mg, [list(target)], 0.0)[0][0])

    velocity = [b - a for a, b in pairwise(positions)]
    accel = [abs(b - a) for a, b in pairwise(velocity)]
    # No single step may dominate: a linear ramp dumps the whole blend rate in
    # one sample at the end, which shows up as one acceleration spike far above
    # the rest.
    assert max(accel) < 3 * (sum(accel) / len(accel))


def test_ease_in_starts_and_ends_without_an_acceleration_step() -> None:
    """The blend must be smooth in acceleration too, not just in velocity.

    A plain smoothstep zeroes the blend velocity at both ends but enters and
    leaves at its *peak* acceleration — the same defect as a linear ramp, one
    order up. Smootherstep zeroes both derivatives, so the edges of the window
    sit well below the peak. Measured over the window on the jogger's 10ms grid:
    smoothstep gives 1.00x peak at both edges, smootherstep 0.19x.
    """
    jogger, mg, _session = _jogger()
    jogger._ease_in_s = 0.5
    jogger._ease_baseline = cast("dict[MotionGroup, list[float]]", {mg: [0.0] * 6})

    target = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    positions = []
    for i in range(51):  # exactly the 500ms window, inclusive of both ends
        jogger._tick_ms = i * 10.0
        positions.append(jogger._ease_steps(mg, [list(target)], 0.0)[0][0])

    velocity = [b - a for a, b in pairwise(positions)]
    accel = [abs(b - a) for a, b in pairwise(velocity)]
    peak = max(accel)

    assert accel[0] < 0.5 * peak, f"entered the window at {accel[0] / peak:.2f}x peak accel"
    assert accel[-1] < 0.5 * peak, f"left the window at {accel[-1] / peak:.2f}x peak accel"


def test_ease_in_spans_the_full_window() -> None:
    """Easing still starts at the baseline and finishes exactly on the target."""
    jogger, mg, _session = _jogger()
    jogger._ease_in_s = 0.5
    jogger._ease_baseline = cast("dict[MotionGroup, list[float]]", {mg: [0.0] * 6})
    target = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    jogger._tick_ms = 0.0
    assert jogger._ease_steps(mg, [list(target)], 0.0)[0][0] == pytest.approx(0.0)

    jogger._tick_ms = 500.0
    assert jogger._ease_steps(mg, [list(target)], 0.0)[0][0] == pytest.approx(100.0)


class _FakeRerun:
    """Records what would have been sent to Rerun."""

    def __init__(self) -> None:
        self.tcp: list[tuple[str, list[float], float | None]] = []
        self.joint: list[tuple[str, list[float], float | None]] = []

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


def test_joint_mode_tracking_uses_the_joint_logger() -> None:
    jogger, _mg, _session = _jogger(mode="joint")
    fake = _FakeRerun()
    jogger._rerun = cast("PolicyRerunLogger", fake)
    jogger._record_commanded("mg", [[1.0, 2.0, 3.0]], 0.0, 1000)

    jogger._log_state_tracking("mg", "joint", object(), 1000, 100.0)

    assert fake.joint and not fake.tcp
    assert fake.joint[0][1] == [1.0, 2.0, 3.0]


# ---------------------------------------------------------------------------
# The rolling buffer: what it sends, and that it cannot grow without bound.
# ---------------------------------------------------------------------------


class _PushSession(_FakeSession):
    """A session that accepts pushes, for driving ``_set_live_target``."""

    num_joints = 6
    estimated_server_timestamp_ms = 0

    def __init__(self, mode: str = "cartesian") -> None:
        super().__init__()
        self.mode = mode
        self.chunks: list[dict] = []

    def update_chunk(self, **kwargs) -> None:
        self.chunks.append(kwargs)


class _NamedMotionGroup:
    """``_set_live_target`` reaches the Rerun helpers, which key off the id."""

    id = "mg"


def test_the_rolling_target_buffer_stays_bounded_before_the_timeline_starts() -> None:
    """The buffered path prunes by age too, against the same pinned clock.

    Its cutoff is ``now - buffer_window_ms``, which goes negative while ``elapsed`` is
    pinned at 0.0, so no sample is ever older than it and the window grows for as
    long as the session takes to start.
    """
    from novapolicy.jogging.jogger import _MAX_LIVE_SAMPLES

    jogger, _stub, _session = _jogger()
    mg = cast("MotionGroup", _NamedMotionGroup())
    jogger._mg_list = [mg]
    jogger._sessions = cast("dict[MotionGroup, WaypointJoggingSession]", {mg: _PushSession()})
    jogger._buffer_window_ms = 30.0
    jogger._ease_in_s = 0.0
    jogger._ease_baseline = {}
    jogger._loop_t0 = None  # the timeline is not anchored: elapsed is pinned
    assert jogger.elapsed == 0.0

    for _ in range(_MAX_LIVE_SAMPLES * 4):
        jogger._set_live_target(mg, [1.0] * 6)

    assert len(jogger._target_buffers[mg]) <= _MAX_LIVE_SAMPLES


def _buffered_jogger(buffer_window_ms: float = 500.0):
    """A jogger wired to record what the buffered path pushes to its session."""
    jogger, _stub, _session = _jogger()
    mg = cast("MotionGroup", _NamedMotionGroup())
    session = _PushSession()
    jogger._mg_list = [mg]
    jogger._sessions = cast("dict[MotionGroup, WaypointJoggingSession]", {mg: session})
    jogger._buffer_window_ms = buffer_window_ms
    jogger._ease_in_s = 0.0
    jogger._ease_baseline = {}
    return jogger, mg, session


def test_nothing_sent_is_ever_beyond_the_newest_measured_target() -> None:
    """The whole point of the buffer: the horizon is measured, never guessed.

    A target ramping along x means every commanded x must be one the target has
    actually reached. Extrapolation of any kind puts a waypoint past the newest
    sample, so the newest sample is the ceiling the pushes are checked against.
    """
    jogger, mg, session = _buffered_jogger()
    speed = 100.0  # mm/s along x

    for i in range(120):
        t = i * 0.01
        jogger._tick_ms = t * 1000.0
        jogger._set_live_target(mg, [speed * t, 0.0, 0.0, 0.0, 0.0, 0.0])

    assert session.chunks, "the buffer never pushed anything"
    newest_x = speed * (119 * 0.01)
    commanded = [step[0] for chunk in session.chunks for step in chunk["steps"]]
    assert max(commanded) <= newest_x + 1e-9, (max(commanded), newest_x)


def test_the_buffered_window_spans_the_buffer_duration() -> None:
    """``buffer_window_ms`` is the horizon, so the window has to actually cover it.

    The server caps its speed at whatever it can brake to a stop within, so a
    window that collapsed to a couple of waypoints would make the robot creep.
    """
    jogger, mg, session = _buffered_jogger(buffer_window_ms=500.0)

    for i in range(120):
        t = i * 0.01
        jogger._tick_ms = t * 1000.0
        jogger._set_live_target(mg, [100.0 * t, 0.0, 0.0, 0.0, 0.0, 0.0])

    last = session.chunks[-1]
    span_ms = last["dt_ms"] * (len(last["steps"]) - 1)
    assert span_ms == pytest.approx(500.0, abs=60.0), span_ms


def test_a_disabled_buffer_sends_the_target_alone_and_unmodified() -> None:
    """``buffer_window_ms=0`` means exactly one waypoint, exactly where the target is."""
    jogger, mg, session = _buffered_jogger(buffer_window_ms=0.0)
    jogger._tick_ms = 0.0

    jogger._set_live_target(mg, [12.0, 34.0, 56.0, 0.0, 0.0, 0.0])

    assert len(session.chunks) == 1
    assert session.chunks[0]["steps"] == [[12.0, 34.0, 56.0, 0.0, 0.0, 0.0]]
    assert session.chunks[0]["dt_ms"] == 0.0


def test_live_jogging_buffers_by_default() -> None:
    """The default has to be the buffer, not the bare single target.

    ``buffer_window_ms=0`` sends lone terminal waypoints, which the server decelerates
    to; it is a deliberate opt-out, not a sensible starting point for tracking a
    moving target.
    """
    import inspect

    from novapolicy.jogging.jogger import jog_joints, jog_tcp

    for factory in (jog_joints, jog_tcp):
        default = inspect.signature(factory).parameters["buffer_window_ms"].default
        assert default == 500.0, (factory.__name__, default)
