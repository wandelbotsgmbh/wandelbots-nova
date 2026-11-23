"""Behavioural tests for WaypointJoggingSession.

These drive the *real* async jogging + state-stream loops, substituting only
the NOVA SDK boundary: the motion group, the API gateway, and the small
``_sdk`` accessors. The fake gateway plays the role of the server — it feeds the
session a response stream and records the ``ExecuteActionChunksRequest`` messages the
session yields back.

Assertions are on the contract, not internals:
  * what the session sends over the wire (init first, then timestamped
    waypoints),
  * what surfaces to a caller (``has_failed`` / ``failure_exception`` /
    ``stop_condition_triggered`` / ``current_state``).

Nothing here touches private attributes, so a behaviour-preserving refactor of
the loop internals should leave these tests intact.
"""

from __future__ import annotations

import asyncio
import contextlib
from datetime import UTC, datetime
import itertools
import logging
import time
from types import SimpleNamespace
from typing import TYPE_CHECKING
from unittest.mock import AsyncMock, MagicMock, PropertyMock, patch

import pytest
from websockets.exceptions import InvalidStatus

from nova import api
from nova.types import Pose
from novapolicy.jogging import jog_tcp
from novapolicy.jogging.clock import ServerMs
from novapolicy.jogging.live_buffer import LIVE_LEAD_MS, LiveBuffer
from novapolicy.jogging.modules import HoldPadding, default_pipeline
from novapolicy.jogging.waypoint_session import MIN_LEAD_MS, WaypointJoggingSession
from novapolicy.jogging.waypoints import PendingChunk
from novapolicy.types import JoggingNotSupportedError, MotionError, WaypointConfig

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, AsyncIterator, Callable, Iterator, Sequence
    from typing import Any

_SESSION = "novapolicy.jogging.waypoint_session"


def _unpadded() -> list[object]:
    """The default pipeline without hold padding."""
    return [m for m in default_pipeline() if not isinstance(m, HoldPadding)]


def _padded(min_horizon_ms: float) -> list[object]:
    """The default pipeline with hold padding up to ``min_horizon_ms``."""
    return [
        HoldPadding(min_horizon_ms) if isinstance(m, HoldPadding) else m for m in default_pipeline()
    ]


# ---------------------------------------------------------------------------
# Fake server: stands in for api_gateway.action_chunk_streaming_api.execute_action_chunks
# ---------------------------------------------------------------------------


def _ok() -> object:
    """A normal jogging response (no motion error)."""
    return SimpleNamespace(root=SimpleNamespace())


def _motion_error(message: str = "joint_limit") -> object:
    """A response that tells the session the server hit a motion error."""
    return SimpleNamespace(kind="MOTION_ERROR", message=message)


class FakeJoggingServer:
    """Consumes the session's request generator and replays scripted responses.

    The session yields an ``InitializeActionChunksRequest`` first, then action
    chunk requests independently of response latency. We keep emitting ``_ok()``
    responses on a configurable cadence until stopped, recording every request
    the session produces. Like NOVA, the fake answers a ``StopActionChunksRequest``
    with a ``StopActionChunksResponse`` (ADR 0019) unless ``acknowledge_stop`` is off;
    ``stop_ack_gate``, when set, holds that answer back until the event is set.
    """

    def __init__(
        self,
        *,
        fault: object | None = None,
        raise_exc: BaseException | None = None,
        response_delay: float = 0.003,
        acknowledge_stop: bool = True,
    ) -> None:
        self.requests: list[object] = []
        self._fault = fault
        self._raise_exc = raise_exc
        self._response_delay = response_delay
        self._acknowledge_stop = acknowledge_stop
        self._stop = asyncio.Event()
        self.stop_received = asyncio.Event()
        self.stop_ack_gate: asyncio.Event | None = None
        self.stop_acknowledged = False
        """Whether the fake has answered the session's stop request."""
        self.closed = False
        """Whether the session closed the stream (its request generator ended)."""

    async def execute_action_chunks(
        self,
        *,
        cell: str,
        controller: str,
        client_request_generator: Callable[
            [AsyncGenerator[object, None]], AsyncGenerator[object, None]
        ],
    ) -> None:
        if self._raise_exc is not None:
            # Stand in for an api-gateway that rejects the websocket upgrade.
            raise self._raise_exc

        async def responses() -> AsyncGenerator[object, None]:
            if self._fault is not None:
                # The session checks for a motion error at the top of each
                # response iteration, before it waits for a chunk to send, so a
                # fault on the first response is enough to surface it.
                yield self._fault
                return
            while not self._stop.is_set():
                if (
                    self._acknowledge_stop
                    and not self.stop_acknowledged
                    and self.stop_received.is_set()
                    and (self.stop_ack_gate is None or self.stop_ack_gate.is_set())
                ):
                    self.stop_acknowledged = True
                    yield api.models.StopActionChunksResponse()
                else:
                    yield _ok()
                await asyncio.sleep(self._response_delay)

        try:
            async for request in client_request_generator(responses()):
                self.requests.append(request)
                if isinstance(_inner(request), api.models.StopActionChunksRequest):
                    self.stop_received.set()
        finally:
            self.closed = True

    def stop(self) -> None:
        self._stop.set()


# ---------------------------------------------------------------------------
# Harness
# ---------------------------------------------------------------------------


def _initial_state() -> object:
    return SimpleNamespace(
        joints=(0.0,) * 6,
        pose=Pose(0, 0, 300, 0, 0, 0),
        tcp="Flange",
    )


def _stream_state(state: object, *, ts_ms: int = 0, kind: str | None = None) -> object:
    """One MotionGroupState as yielded by ``stream_state``."""
    jog_state = SimpleNamespace(kind=kind) if kind is not None else None
    details = SimpleNamespace(jogger_session_timestamp_ms=ts_ms, state=jog_state)
    return SimpleNamespace(
        joint_position=list(state.joints),
        tcp_pose=None,  # keep the initial pose; avoids constructing a Pose here
        tcp="Flange",
        execute=SimpleNamespace(details=details),
        timestamp=datetime.now(UTC),
    )


def _build_session(
    *,
    fault: object | None = None,
    raise_exc: BaseException | None = None,
    states: Sequence[object] = (),
    stop_conditions: list[object] | None = None,
    mode: str = "joint",
    config: WaypointConfig | None = None,
    response_delay: float = 0.003,
    pipeline: Sequence[object] | None = None,
    acknowledge_stop: bool = True,
) -> tuple[WaypointJoggingSession, FakeJoggingServer]:
    server = FakeJoggingServer(
        fault=fault,
        raise_exc=raise_exc,
        response_delay=response_delay,
        acknowledge_stop=acknowledge_stop,
    )

    async def state_stream(**_kw: object) -> AsyncGenerator[object, None]:
        for s in states:
            yield s
            await asyncio.sleep(0.003)
        idle_state = states[-1] if states else _stream_state(_initial_state())
        while True:  # idle until cancelled on stop()
            yield idle_state
            await asyncio.sleep(0.003)

    mg = MagicMock()
    mg.id = "0@ur10e"
    mg.get_state = AsyncMock(return_value=_initial_state())
    mg.stream_state = MagicMock(side_effect=state_stream)
    mg.active_tcp_name = AsyncMock(return_value="Flange")
    mg.tcp_names = AsyncMock(return_value=["Flange"])

    gateway = MagicMock()
    gateway.action_chunk_streaming_api.execute_action_chunks = server.execute_action_chunks

    session = WaypointJoggingSession(
        motion_group=mg,
        config=config or WaypointConfig(),
        tcp="Flange",
        mode=mode,
        stop_conditions=stop_conditions,
        pipeline=pipeline,
    )

    patches = [
        patch(f"{_SESSION}.get_api_gateway", return_value=gateway),
        patch(f"{_SESSION}.get_cell", return_value="cell"),
        patch(f"{_SESSION}.get_controller_id", return_value="0@ur10e"),
    ]
    for p in patches:
        p.start()
    session._test_patches = patches  # type: ignore[attr-defined]  # torn down by _built
    return session, server


@contextlib.contextmanager
def _built(**kwargs: Any) -> Iterator[tuple[WaypointJoggingSession, FakeJoggingServer]]:
    """``_build_session(**kwargs)``, with its SDK patches undone afterwards."""
    session, server = _build_session(**kwargs)
    try:
        yield session, server
    finally:
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@contextlib.asynccontextmanager
async def _started(
    *, ready: bool = True, **kwargs: Any
) -> AsyncIterator[tuple[WaypointJoggingSession, FakeJoggingServer]]:
    """A built session, started (and ready unless ``ready=False``), stopped even on failure."""
    with _built(**kwargs) as (session, server):
        try:
            await session.start()
            if ready:
                await session.wait_ready()
            yield session, server
        finally:
            await session.stop()
            server.stop()


def _server_now(session: WaypointJoggingSession) -> Any:
    """Patch the server time the session's clock estimates; set it on the yielded mock."""
    return patch.object(
        type(session._clock), "estimated_server_timestamp_ms", new_callable=PropertyMock
    )


async def _wait_until(predicate: Callable[[], bool], *, timeout: float = 1.0) -> None:
    loop = asyncio.get_event_loop()
    deadline = loop.time() + timeout
    while not predicate():
        if loop.time() > deadline:
            raise AssertionError("condition not reached within timeout")
        await asyncio.sleep(0.005)


def _inner(request: object) -> object:
    """Unwrap the ExecuteActionChunksRequest envelope to the concrete message."""
    return getattr(request, "root", request)


def _is_joint_chunk(message: object) -> bool:
    """True for an ActionChunkRequest whose waypoints carry joint coordinates.

    Both modes share one request type now, so the mode a chunk was built for
    only shows up on the waypoints themselves.
    """
    return isinstance(message, api.models.ActionChunkRequest) and all(
        waypoint.waypoint.kind == "JOINTS" for waypoint in message.waypoints
    )


def _is_pose_chunk(message: object) -> bool:
    """True for an ActionChunkRequest whose waypoints carry pose coordinates."""
    return isinstance(message, api.models.ActionChunkRequest) and all(
        waypoint.waypoint.kind == "POSE" for waypoint in message.waypoints
    )


def _joints(waypoint: object) -> list[float]:
    """Joint values of one waypoint, past the coordinates envelope."""
    return list(waypoint.waypoint.joints)


def _pose(waypoint: object) -> object:
    """Pose of one waypoint, past the coordinates envelope."""
    return waypoint.waypoint.pose


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_it_initializes_the_jogging_session_before_any_waypoints():
    """The very first message to the server is an InitializeActionChunksRequest."""
    async with _started() as (_session, server):
        await _wait_until(lambda: len(server.requests) >= 1)
        assert isinstance(_inner(server.requests[0]), api.models.InitializeActionChunksRequest)


@pytest.mark.asyncio
async def test_a_queued_joint_chunk_is_sent_as_a_timestamped_waypoint():
    """update_chunk(step) reaches the server as a joint-flavoured ActionChunkRequest."""
    async with _started(pipeline=_unpadded()) as (session, server):
        target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        session.update_chunk(
            steps=[target],
            dt_ms=50.0,
            first_timestamp_ms=5_000,
            action_timestep=7,
        )

        await _wait_until(lambda: len(server.requests) >= 2)
        waypoint_req = _inner(server.requests[1])
        assert _is_joint_chunk(waypoint_req)
        assert len(waypoint_req.waypoints) == 1
        sent = waypoint_req.waypoints[0]
        assert _joints(sent) == target
        assert sent.timestamp == 5_000  # absolute anchor honoured exactly
        assert session.scheduled.action_timestep == 7
        assert session.scheduled.waypoint_timestamps == (5_000,)


@pytest.mark.asyncio
async def test_lead_chunks_resend_the_earlier_waypoints_that_are_not_yet_due():
    """A rolling window: a new chunk would otherwise delete the waypoints queued before it."""
    async with _started(pipeline=_unpadded()) as (session, server):
        with _server_now(session) as estimated:
            estimated.return_value = 1_000
            for k, observed in enumerate((1_000, 1_033, 1_066)):
                session.update_chunk(
                    steps=[[float(k)] * 6], dt_ms=1000 / 30, lead=True, observed_at_ms=observed
                )
                await _wait_until(lambda k=k: len(server.requests) >= 2 + k)
            estimated.return_value = 1_120
            session.update_chunk(
                steps=[[3.0] * 6], dt_ms=1000 / 30, lead=True, observed_at_ms=1_100
            )
            await _wait_until(lambda: len(server.requests) >= 5)

        third = _inner(server.requests[3]).waypoints
        assert [w.timestamp for w in third] == [1_100, 1_133, 1_166]
        assert [_joints(w)[0] for w in third] == [0.0, 1.0, 2.0]
        fourth = _inner(server.requests[4]).waypoints
        # due within MIN_LEAD_MS of now (1100, 1133): left out
        assert [w.timestamp for w in fourth] == [1_166, 1_200]


@pytest.mark.asyncio
async def test_a_lead_chunk_of_several_steps_is_dated_by_its_observation():
    """Step k is due the lead plus k steps after the observation; steps already too close to
    send are dropped and the rest keep their times; earlier waypoints not yet due are resent."""
    async with _started(pipeline=_unpadded()) as (session, server):
        with _server_now(session) as estimated:
            estimated.return_value = 1_030  # 30 ms of inference: nothing is late yet
            session.update_chunk(
                steps=[[float(k)] * 6 for k in range(4)], dt_ms=40, lead=True, observed_at_ms=1_000
            )
            await _wait_until(lambda: len(server.requests) >= 2)
            # The next chunk took 200 ms: its first steps are too close to send.
            estimated.return_value = 1_280
            session.update_chunk(
                steps=[[10.0 + k] * 6 for k in range(6)], dt_ms=40, lead=True, observed_at_ms=1_080
            )
            await _wait_until(lambda: len(server.requests) >= 3)

        first = _inner(server.requests[1]).waypoints
        assert [w.timestamp for w in first] == [1_100, 1_140, 1_180, 1_220]
        second = _inner(server.requests[2]).waypoints
        # Dated 1180, 1220, 1260, 1300, ... from its observation. Below now + MIN_LEAD_MS (1300)
        # they are left out; the rest keep their times.
        assert [w.timestamp for w in second] == [1_300, 1_340, 1_380]
        assert [_joints(w)[0] for w in second] == [13.0, 14.0, 15.0]
        # The dropped steps 0 to 2 kept 1180, 1220 and 1260: step 3 sits three steps after.
        assert second[0].timestamp == 1_180 + 3 * 40


@pytest.mark.asyncio
async def test_a_lead_chunk_dated_from_the_send_keeps_every_step():
    """Without an observation the send is the reference, so nothing is ever late."""
    async with _started(pipeline=_unpadded()) as (session, server):
        with _server_now(session) as estimated:
            estimated.return_value = 50_000
            session.update_chunk(steps=[[float(k)] * 6 for k in range(4)], dt_ms=40, lead=True)
            await _wait_until(lambda: len(server.requests) >= 2)
        sent = _inner(server.requests[1]).waypoints
        assert [w.timestamp for w in sent] == [50_100, 50_140, 50_180, 50_220]


@pytest.mark.asyncio
async def test_a_lead_chunk_resends_the_earlier_waypoints_due_before_it():
    async with _started(pipeline=_unpadded()) as (session, server):
        with _server_now(session) as estimated:
            estimated.return_value = 1_010
            session.update_chunk(
                steps=[[float(k)] * 6 for k in range(4)], dt_ms=40, lead=True, observed_at_ms=1_000
            )
            await _wait_until(lambda: len(server.requests) >= 2)
            estimated.return_value = 1_050
            session.update_chunk(
                steps=[[10.0 + k] * 6 for k in range(4)], dt_ms=40, lead=True, observed_at_ms=1_040
            )
            await _wait_until(lambda: len(server.requests) >= 3)

        second = _inner(server.requests[2]).waypoints
        # The first chunk's 1100 is not yet due and comes before the new chunk's 1140.
        assert [w.timestamp for w in second] == [1_100, 1_140, 1_180, 1_220, 1_260]
        assert [_joints(w)[0] for w in second] == [0.0, 10.0, 11.0, 12.0, 13.0]


def test_a_lead_chunk_that_is_late_entirely_is_not_sent_and_not_guarded():
    """Whole chunk in the past: nothing to send. Multi-step chunks never count as late."""
    with _built(pipeline=_unpadded()) as (session, _server):
        with _server_now(session) as estimated:
            estimated.return_value = 2_000
            pending = PendingChunk(
                steps=[[0.0] * 6] * 3, dt_ms=40, lead=True, observed_at_ms=1_000, sequence=1
            )
            request, stamps, skipped = session._schedule(pending)
        assert request is None
        assert skipped == 3
        assert stamps == [1_100, 1_140, 1_180]


@pytest.mark.asyncio
async def test_a_chunk_action_runs_when_its_step_is_due_or_at_once_when_past():
    ran: list[str] = []

    def action(name: str):
        async def run() -> None:
            ran.append(name)

        return run

    with _built() as (session, _server), _server_now(session) as estimated:
        session._running = True  # a started session, without its network loop
        estimated.return_value = 1_000
        later = PendingChunk(steps=[[0.0] * 6] * 2, dt_ms=50, when_due=(1, action("later")))
        session._run_when_due(later, [1_000, 1_050])
        past = PendingChunk(steps=[[0.0] * 6], dt_ms=50, when_due=(0, action("past")))
        session._run_when_due(past, [900])
        await asyncio.sleep(0.01)
        assert ran == ["past"]
        await asyncio.sleep(0.06)
        assert ran == ["past", "later"]


@pytest.mark.asyncio
async def test_a_chunk_action_fails_the_session_when_it_raises():
    async def broken() -> None:
        raise RuntimeError("gripper unreachable")

    with _built() as (session, _server):
        session._running = True  # a started session, without its network loop
        pending = PendingChunk(steps=[[0.0] * 6], dt_ms=50, when_due=(0, broken))
        session._run_when_due(pending, [0])
        await asyncio.sleep(0.01)
        assert session.has_failed
        assert isinstance(session.failure_exception, RuntimeError)


@pytest.mark.asyncio
async def test_a_chunk_action_is_skipped_when_the_motion_stopped_before_it_was_due():
    """A stop condition or a failure ends the motion; the gripper must not still fire."""
    ran: list[str] = []

    async def run() -> None:
        ran.append("ran")

    with _built() as (session, _server):
        session._running = True
        for stop in ("stop condition", "failure"):
            pending = PendingChunk(steps=[[0.0] * 6], dt_ms=50, when_due=(0, run))
            due = int(session.estimated_server_timestamp_ms) + 30
            session._run_when_due(pending, [due])
            if stop == "stop condition":
                session._stop_condition = "door opened"
            else:
                session._stop_condition = None
                session._failed = True
            await asyncio.sleep(0.08)
        assert ran == []


@pytest.mark.asyncio
async def test_a_replaced_chunk_never_runs_its_action_and_stop_cancels_the_rest():
    """The replaced chunk's motion never ran, so neither does its action; one not yet due when
    the session stops belongs to a stopped motion."""
    ran: list[str] = []

    def action(name: str):
        async def run() -> None:
            ran.append(name)

        return run

    async with _started(pipeline=_unpadded()) as (session, server):
        # Both queued before the loop sends: the second replaces the first.
        session.update_chunk(steps=[[0.0] * 6], dt_ms=33, when_due=(0, action("replaced")))
        session.update_chunk(
            steps=[[0.1] * 6],
            dt_ms=33,
            first_timestamp_ms=int(session.estimated_server_timestamp_ms) + 10_000,
            when_due=(0, action("not yet due")),
        )
        await _wait_until(lambda: len(server.requests) >= 2)
        await session.stop()
        await asyncio.sleep(0.01)
        assert ran == []


class _AddOne:
    """A waypoint module that adds 1 to joint 0 and counts how often it ran."""

    def __init__(self) -> None:
        self.calls = 0

    def on_waypoints(self, waypoints, context):
        self.calls += 1
        return [[v[0] + 1.0, *v[1:]] for _, v in waypoints]


@pytest.mark.asyncio
async def test_waypoint_modules_reshape_the_values_but_never_the_timestamps():
    """A re-sent waypoint is reshaped from its raw value again, so it gets the same value in
    every request; the drain's final target is what was actually sent."""
    async with _started(pipeline=[_AddOne()]) as (session, server):
        with _server_now(session) as estimated:
            estimated.return_value = 1_000
            for k, observed in enumerate((1_000, 1_033)):
                session.update_chunk(
                    steps=[[float(k)] * 6], dt_ms=1000 / 30, lead=True, observed_at_ms=observed
                )
                await _wait_until(lambda k=k: len(server.requests) >= 2 + k)
        first, second = (_inner(r).waypoints for r in server.requests[1:3])
        assert [_joints(w)[0] for w in first] == [1.0]
        assert [_joints(w)[0] for w in second] == [1.0, 2.0]  # re-sent once, not reshaped twice
        assert [w.timestamp for w in second] == [1_100, 1_133]  # timestamps untouched
        assert session._last_sent[1][0] == 2.0


@pytest.mark.asyncio
async def test_commanded_at_is_what_was_sent_for_that_moment():
    """Tracking compares a state with what was sent for its own moment: after the waypoint
    modules, interpolated between waypoints, clamped outside them, a request replacing what was
    sent from its first timestamp on."""
    async with _started(pipeline=[_AddOne()]) as (session, server):
        assert session.commanded_at(5_000) is None
        session.update_chunk(
            steps=[[0.0] * 6, [10.0] * 6], dt_ms=50.0, first_timestamp_ms=ServerMs(5_000)
        )
        await _wait_until(lambda: len(server.requests) >= 2)
        assert [session.commanded_at(t)[0] for t in (4_000, 5_000, 5_025, 9_000)] == [
            1.0,
            1.0,
            pytest.approx(6.0),
            11.0,
        ]
        session.update_chunk(steps=[[20.0] * 6], dt_ms=50.0, first_timestamp_ms=ServerMs(5_025))
        await _wait_until(lambda: len(server.requests) >= 3)
        assert [session.commanded_at(t)[0] for t in (5_000, 5_025, 5_050)] == [1.0, 21.0, 21.0]


@pytest.mark.asyncio
async def test_every_state_observer_sees_each_packet_even_when_another_fails():
    """A failing Rerun observer must not starve the policy's own observation."""
    seen: list[int] = []

    def failing(*_args: object) -> None:
        raise RuntimeError("viewer gone")

    def recording(_state: object, ts_ms: int, _generated_at: float) -> None:
        seen.append(ts_ms)

    states = [_stream_state(_initial_state(), ts_ms=t) for t in (10, 20, 30)]
    with _built(states=states) as (session, server):
        session.add_state_observer(failing)
        session.add_state_observer(recording)
        try:
            await session.start()
            await _wait_until(lambda: 30 in seen)
            session.remove_state_observer(recording)
            count = len(seen)
            await asyncio.sleep(0.02)
        finally:
            await session.stop()
            server.stop()
    assert seen[:3] == [10, 20, 30]
    assert len(seen) == count


def test_each_session_runs_its_own_copy_of_a_module():
    module = _AddOne()
    with _built(pipeline=[module]) as (left, _), _built(pipeline=[module]) as (right, _):
        assert left._waypoint_modules[0] is not right._waypoint_modules[0]
        assert left._waypoint_modules[0] is not module


def test_a_waypoint_module_may_not_add_or_drop_waypoints():
    class _DropsOne:
        def on_waypoints(self, waypoints, context):
            return [v for _, v in waypoints][1:]

    with _built(pipeline=[_DropsOne()]) as (session, _):
        pending = PendingChunk(steps=[[0.0] * 6] * 2, dt_ms=40, first_timestamp_ms=5_000)
        with pytest.raises(ValueError, match="returned 1 values for 2 waypoints"):
            session._schedule(pending)


# ---------------------------------------------------------------------------
# A LiveBuffer in the pipeline: what the session queues and sends
# ---------------------------------------------------------------------------

_LIVE_START_MS = 1_000
_LIVE_TICK_MS = 20
"""Session time between live targets, i.e. the caller's own step."""


@contextlib.contextmanager
def _live_session(
    window_ms: float = 500.0, *, mode: str = "joint", config: WaypointConfig | None = None
) -> Iterator[tuple[WaypointJoggingSession, PropertyMock]]:
    """A session with ``LiveBuffer(window_ms)`` in its pipeline and a clock the test sets."""
    pipeline = [LiveBuffer(window_ms=window_ms), *default_pipeline()]
    with (
        _built(pipeline=pipeline, mode=mode, config=config) as (session, _),
        _server_now(session) as now,
    ):
        now.return_value = _LIVE_START_MS
        yield session, now


def _live_buffer(session: WaypointJoggingSession) -> LiveBuffer:
    """The session's own copy of the LiveBuffer."""
    module = session._chunk_modules[0]
    assert isinstance(module, LiveBuffer)
    return module


def _push_live(
    session: WaypointJoggingSession,
    now: PropertyMock,
    target: list[float],
    at_ms: int,
    **kwargs: object,
) -> PendingChunk:
    """Queue one live target at session time ``at_ms``, as the jogger sends it (alone, no
    padding), and return what the session will send."""
    now.return_value = at_ms
    kwargs.setdefault("extend_buffer", False)
    session.update_chunk(steps=[list(target)], dt_ms=0.0, **kwargs)  # type: ignore[arg-type]
    assert session._pending_request is not None
    return session._pending_request


def _feed_live(
    session: WaypointJoggingSession,
    now: PropertyMock,
    count: int,
    *,
    step: float = 0.01,
    tick_ms: int = _LIVE_TICK_MS,
    **kwargs: object,
) -> list[PendingChunk]:
    """Queue ``count`` live targets ramping by ``step``, one tick apart."""
    return [
        _push_live(session, now, [i * step] * 6, _LIVE_START_MS + i * tick_ms, **kwargs)
        for i in range(count)
    ]


def _horizons(pendings: list[PendingChunk]) -> list[PendingChunk]:
    return [pending for pending in pendings if len(pending.steps) > 1]


def test_a_live_buffer_sends_the_first_target_alone():
    """One sample cannot define a spacing; waiting for the window to fill would never move."""
    with _live_session() as (session, now):
        pending = _push_live(session, now, [0.5] * 6, _LIVE_START_MS)
        assert pending.steps == [[0.5] * 6]
        assert pending.first_timestamp_ms is None  # the caller's placement, untouched
        sent = session._schedule(pending)[0]
        assert sent is not None
        assert [_joints(w) for w in sent.waypoints] == [[0.5] * 6]


def test_a_filled_live_buffer_is_replayed_as_a_horizon():
    with _live_session() as (session, now):
        last = _feed_live(session, now, 40)[-1]
        assert len(last.steps) > 1, "a horizon, not a lone terminal waypoint"
        assert last.dt_ms == pytest.approx(50.0)  # the session's single_step_dt_ms grid
        assert last.lead is False


def test_a_replayed_horizon_is_never_padded_even_when_the_caller_asks():
    """Holding one pose is not the same as replaying where the target went. The lone target
    before the buffer can define a spacing passes through unchanged, so it keeps the caller's
    padding (500 ms of holds on the 50 ms grid)."""
    with _live_session(window_ms=100.0) as (
        session,
        now,
    ):
        pendings = _feed_live(session, now, 40, extend_buffer=True)
        assert pendings[0].steps == [[0.0] * 6] * 10
        assert pendings[0].caller_step_count == 1
        horizons = pendings[1:]
        assert all(len(pending.steps) > 1 for pending in horizons)
        for pending in horizons:
            assert pending.extend_buffer is False
            assert len(pending.steps) == pending.caller_step_count
            assert len(pending.steps) * pending.dt_ms < 500.0, "padding would reach 500 ms"
            assert pending.steps[-1][0] != pending.steps[-2][0], "a hold was appended"


def test_every_replayed_send_carries_an_absolute_first_timestamp():
    """Never a "now"-relative or lead placement, also for a caller that asked for the lead."""
    with _live_session() as (session, now):
        pendings = [
            _push_live(session, now, [i * 0.01] * 6, at, lead=True, observed_at_ms=at - 30)
            for i, at in enumerate(range(_LIVE_START_MS, _LIVE_START_MS + 800, _LIVE_TICK_MS))
        ]
        horizons = _horizons(pendings)
        assert horizons
        for pending in horizons:
            assert pending.first_timestamp_ms is not None
            assert pending.timestamp_offset_steps == 0
            assert pending.lead is False
            assert pending.observed_at_ms is None
        sent = session._schedule(horizons[-1])[0]
        assert sent is not None
        assert sent.waypoints[0].timestamp == horizons[-1].first_timestamp_ms


def test_a_live_target_keeps_its_timestamp_as_the_window_is_resent():
    """The property the buffer exists for, read off the requests the session sends: a target
    in several consecutive replays is commanded at one absolute time in all of them."""
    with _live_session() as (session, now):
        stamps: dict[float, set[int]] = {}
        sends: dict[float, int] = {}
        for i in range(80):
            at = _LIVE_START_MS + i * _LIVE_TICK_MS
            pending = _push_live(session, now, [i * 0.01] * 6, at)
            if len(pending.steps) == 1 or pending.dt_ms != pytest.approx(50.0):
                continue  # still filling: see the test above
            sent = session._schedule(pending)[0]
            assert sent is not None
            for waypoint in sent.waypoints:
                key = round(_joints(waypoint)[0], 9)
                stamps.setdefault(key, set()).add(waypoint.timestamp)
                sends[key] = sends.get(key, 0) + 1
        assert max(sends.values()) > 1, "no target was re-sent, so nothing was checked"
        moved = {k: v for k, v in stamps.items() if len(v) > 1}
        assert not moved, f"{len(moved)} targets were commanded at more than one time"


def test_a_live_buffer_replays_its_window_from_its_own_timeline():
    """Each target keeps the moment it arrived; the window is played back ``window_ms`` late
    plus the lead, from the session clock at the first target."""
    with _live_session(window_ms=30.0) as (session, now):
        for k, value in enumerate((1.0, 2.0, 3.0)):
            pending = _push_live(session, now, [value] * 6, _LIVE_START_MS + 10 * k)
        assert pending.steps == [[1.0] * 6, [2.0] * 6, [3.0] * 6]
        assert pending.dt_ms == pytest.approx(10.0)  # measured, not assumed
        assert pending.first_timestamp_ms == int(_LIVE_START_MS + 30 + LIVE_LEAD_MS)


def test_a_live_buffer_replays_tcp_poses():
    with _live_session(window_ms=30.0, mode="cartesian") as (session, now):
        poses = [[x, 200.0, 300.0, 0.0, 3.14, 0.0] for x in (500.0, 510.0, 520.0)]
        for k, pose in enumerate(poses):
            pending = _push_live(session, now, pose, _LIVE_START_MS + 10 * k)
        assert pending.steps == poses
        assert pending.dt_ms == pytest.approx(10.0)
        sent = session._schedule(pending)[0]
        assert sent is not None
        assert _is_pose_chunk(sent)


def test_a_chunk_passes_a_live_buffer_and_clears_it_with_one_warning(caplog):
    """A chunk carries its own horizon; the held targets are stale once it arrives, which is
    worth one warning, not one per chunk."""
    chunk = [[9.0] * 6, [9.0] * 6]
    with (
        _live_session(window_ms=30.0) as (session, now),
        caplog.at_level(logging.WARNING, logger="novapolicy.jogging.live_buffer"),
    ):
        for k, value in enumerate((1.0, 2.0, 3.0)):
            _push_live(session, now, [value] * 6, _LIVE_START_MS + 10 * k)
        session.update_chunk(steps=chunk, dt_ms=10.0, first_timestamp_ms=5_000)
        pending = session._pending_request
        assert pending is not None
        assert pending.steps[:2] == chunk  # padded by the session as usual, not replayed
        assert pending.first_timestamp_ms == 5_000
        assert pending.dt_ms == 10.0
        assert _live_buffer(session)._points == []

        # The window restarts from the first target after the chunk.
        pending = _push_live(session, now, [10.0] * 6, _LIVE_START_MS + 30)
        assert pending.steps == [[10.0] * 6]
        for k, value in enumerate((11.0, 12.0), start=1):
            pending = _push_live(session, now, [value] * 6, _LIVE_START_MS + 30 + 10 * k)
        replayed = [step[0] for step in pending.steps]
        assert len(replayed) > 1
        assert all(value >= 10.0 for value in replayed), replayed
        assert max(replayed) == pytest.approx(12.0)

        session.update_chunk(steps=chunk, dt_ms=10.0, first_timestamp_ms=6_000)
    warnings = [r for r in caplog.records if r.name == "novapolicy.jogging.live_buffer"]
    assert len(warnings) == 1, [r.getMessage() for r in warnings]


def test_a_chunk_only_caller_is_never_warned_by_a_live_buffer(caplog):
    """Nothing held, nothing discarded, nothing to say."""
    with (
        _live_session(window_ms=30.0) as (session, now),
        caplog.at_level(logging.WARNING, logger="novapolicy.jogging.live_buffer"),
    ):
        for k in range(3):
            now.return_value = _LIVE_START_MS + 10 * k
            session.update_chunk(steps=[[float(k)] * 6] * 2, dt_ms=10.0)
    assert [r for r in caplog.records if r.name == "novapolicy.jogging.live_buffer"] == []


def test_a_live_buffer_moves_when_due_to_the_newest_target():
    """The newest target is the last replayed step; its IO and hooks go with it."""

    def action(name: str):
        async def run() -> None:
            return None

        run.__name__ = name
        return run

    with _live_session() as (session, now):
        first_action = action("first")
        pending = _push_live(session, now, [0.0] * 6, _LIVE_START_MS, when_due=(0, first_action))
        assert pending.when_due == (0, first_action)  # alone: step 0 is the target
        for i in range(1, 40):
            newest = action(f"step {i}")
            pending = _push_live(
                session, now, [i * 0.01] * 6, _LIVE_START_MS + i * 20, when_due=(0, newest)
            )
        assert len(pending.steps) > 1
        assert pending.when_due == (len(pending.steps) - 1, newest)
        assert pending.steps[-1][0] == pytest.approx(39 * 0.01)
        assert pending.caller_step_count == len(pending.steps)
        assert pending.last_caller_step == len(pending.steps) - 1


def test_a_live_buffer_times_targets_by_when_they_were_produced():
    """With observed_at_ms (the jogger's one loop time for all arms) the replay follows when the
    targets were produced, not each session's clock: frozen here, it would replay nothing."""
    with (
        _built(pipeline=[LiveBuffer(window_ms=30.0)]) as (session, _server),
        _server_now(session) as estimated,
    ):
        estimated.return_value = 5_000  # the session clock does not move
        for k in range(8):
            session.update_chunk(
                steps=[[float(k)] * 6],
                dt_ms=0.0,
                extend_buffer=False,
                observed_at_ms=ServerMs(1_000 + 10 * k),
            )
        pending = session._pending_request
        assert len(pending.steps) > 1  # replayed as a horizon on the produced times
        assert pending.steps[-1][0] == pytest.approx(7.0)


def test_a_live_buffer_times_lead_chunks_by_the_session_clock():
    """A policy step's observed_at_ms is its state's timestamp, not when it was produced: for a
    chunk placed by the lead rule the buffer keeps to the session clock."""
    with (
        _built(pipeline=[LiveBuffer(window_ms=30.0)]) as (session, _server),
        _server_now(session) as estimated,
    ):
        estimated.return_value = 5_000  # frozen: lead chunks must not follow observed_at_ms
        for k in range(8):
            session.update_chunk(
                steps=[[float(k)] * 6],
                dt_ms=1000 / 30,
                lead=True,
                extend_buffer=False,
                observed_at_ms=ServerMs(1_000 + 10 * k),
            )
        assert len(session._pending_request.steps) == 1  # no spacing on a frozen clock


def test_sessions_given_one_live_buffer_do_not_share_its_points():
    """Each arm runs its own copy: one arm's targets never reach another arm's horizon."""
    shared = LiveBuffer(window_ms=100.0)
    with (
        _built(pipeline=[shared]) as (left, _),
        _built(pipeline=[shared]) as (right, _),
        _server_now(left) as now,
    ):
        _feed_live(left, now, 10)
        assert _live_buffer(left)._points
        assert _live_buffer(right)._points == []
        assert shared._points == []
        assert _live_buffer(left) is not _live_buffer(right)
        assert _push_live(right, now, [7.0] * 6, _LIVE_START_MS + 500).steps == [[7.0] * 6]


def test_one_step_waypoints_are_spaced_by_the_policy_step_despite_bursty_observations():
    """Observations dated 0 to 60 ms apart must not become speed jumps: the playback clock
    keeps one step between waypoints and follows the observations slowly."""
    step = 1000 / 30
    with _built(config=WaypointConfig(max_expected_latency_ms=150)) as (session, _server):
        observed = [1_000.0, 1_008.0, 1_070.0, 1_100.0, 1_105.0, 1_170.0, 1_200.0, 1_233.0]
        due = [session._playback.due_ms(t + 150, step, earliest_ms=0) for t in observed]
        spacing = [b - a for a, b in itertools.pairwise(due)]
        assert max(spacing) - min(spacing) < 0.3 * step  # observations jumped by 0 to 62 ms
        assert all(0.85 * step < gap < 1.2 * step for gap in spacing)
        assert abs(due[-1] - (observed[-1] + 150)) < 25  # it still follows the observations
        # A stall: the next observation is 400 ms later; the clock starts again from it.
        assert session._playback.due_ms(1_633 + 150, step, earliest_ms=0) == 1_633 + 150
        # An observation dated far earlier never moves the clock backwards.
        assert session._playback.due_ms(1_200 + 150, step, earliest_ms=0) > 1_633 + 150
        # Observations too old to leave the controller its margin: the waypoint goes at the
        # earliest plannable time, and the clock keeps real time instead of falling behind.
        due = [session._playback.due_ms(1_800 + k * step, step, 2_200 + k * step) for k in range(5)]
        assert all(d >= 2_200 + k * step for k, d in enumerate(due))
        assert all(0.5 * step < b - a < 1.5 * step for a, b in itertools.pairwise(due))


@pytest.mark.asyncio
async def test_a_short_chunk_holds_its_final_target_to_fill_the_buffer():
    target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    async with _started(pipeline=_padded(100.0)) as (session, server):
        session.update_chunk(steps=[target], dt_ms=50.0, first_timestamp_ms=5_000)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        assert _is_joint_chunk(request)
        assert [_joints(waypoint) for waypoint in request.waypoints] == [target, target]
        assert [waypoint.timestamp for waypoint in request.waypoints] == [5_000, 5_050]


@pytest.mark.asyncio
async def test_waypoints_already_in_the_past_are_trimmed_before_sending():
    """Only the still-reachable tail of a chunk is sent.

    A chunk is timestamped when the caller builds it but sent slightly later.
    Waypoints whose moment has passed are unreachable, and commanding them makes
    the server jump to catch up, which is heavy velocity ripple on the robot.
    The absolute time-to-position mapping of the surviving waypoints is
    unchanged, so trimming never distorts the trajectory.
    """
    async with _started(pipeline=_unpadded()) as (session, server):
        steps = [[float(i)] * 6 for i in range(6)]
        # Anchored in the past: only the steps at/after now+min_lead survive.
        session.update_chunk(steps=steps, dt_ms=50.0, first_timestamp_ms=0)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        timestamps = [waypoint.timestamp for waypoint in request.waypoints]
        sent = [_joints(waypoint) for waypoint in request.waypoints]

        assert timestamps  # something survived
        assert len(timestamps) < len(steps)  # but not the whole chunk
        assert timestamps[0] >= MIN_LEAD_MS  # still reachable
        # Surviving waypoints keep their original absolute slots.
        for timestamp, step in zip(timestamps, sent, strict=True):
            assert step == steps[timestamp // 50]


@pytest.mark.asyncio
async def test_a_trimmed_chunk_reports_the_timestep_it_actually_starts_at():
    """``scheduled.action_timestep`` names the policy step ``steps[0]`` came from.

    Trimming drops leading waypoints, so the first waypoint sent is not the
    caller's step zero. Reporting the untrimmed timestep misattributes every
    plotted and logged waypoint to a step the robot never received.
    """
    async with _started(pipeline=_unpadded()) as (session, server):
        steps = [[float(i)] * 6 for i in range(6)]
        session.update_chunk(steps=steps, dt_ms=50.0, first_timestamp_ms=0, action_timestep=100)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        skipped = len(steps) - len(request.waypoints)

        assert skipped > 0  # the premise: something was trimmed
        assert session.scheduled.action_timestep == 100 + skipped


@pytest.mark.asyncio
async def test_a_trimmed_chunk_still_reports_where_each_caller_step_landed():
    """Trimming must stay invisible to anyone asking about their own steps.

    Callers place meaning on step indices — a policy queue reads the absolute
    timestamp of its own waypoint zero off the session and treats it as the
    immutable origin of the whole queue. Trimming drops leading waypoints, so
    indexing the *sent* request shifts that origin by however many were dropped,
    on every chunk, compounding.
    """
    seen: list[tuple[int, ServerMs] | None] = []

    class _SeesStart:
        """A chunk module noting where the session says the policy's first step sat."""

        def on_chunk(self, chunk: PendingChunk, context: Any) -> PendingChunk:
            seen.append(context.scheduled_start)  # a closure: survives the per-session copy
            return chunk

    async with _started(pipeline=[_SeesStart(), *_unpadded()]) as (session, server):
        steps = [[float(i)] * 6 for i in range(6)]
        session.update_chunk(steps=steps, dt_ms=50.0, first_timestamp_ms=0, action_timestep=100)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        assert _is_joint_chunk(request)
        skipped = len(steps) - len(request.waypoints)
        assert skipped > 0  # the premise: something was trimmed
        # A surviving step keeps the slot it was laid out on.
        assert request.waypoints[0].timestamp == skipped * 50

        # The next chunk sees the policy's step zero where the caller laid it out, not where
        # the first surviving waypoint went.
        session.update_chunk(steps=steps, dt_ms=50.0)
        assert seen[-1] == (100, 0)


@pytest.mark.asyncio
async def test_the_padded_tail_of_a_short_chunk_is_not_motion_anyone_waits_for():
    """``scheduled.until_server_ms`` ends at the caller's last step.

    A short chunk is padded to ``HoldPadding.min_horizon_ms`` by repeating its final target,
    which gives the server room to brake. Treating that hold as commanded motion
    makes every caller waiting for the chunk to finish sit at the final target
    for the rest of the buffer first — at a 500ms buffer, most of the wait.
    """
    target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    async with _started(pipeline=_padded(200.0)) as (session, server):
        session.update_chunk(steps=[target, target], dt_ms=50.0, first_timestamp_ms=5_000)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        assert _is_joint_chunk(request)
        sent = [waypoint.timestamp for waypoint in request.waypoints]

        # The padding really was sent, so the server still gets its horizon.
        assert sent == [5_000, 5_050, 5_100, 5_150]
        assert session.scheduled.waypoint_timestamps == (5_000, 5_050, 5_100, 5_150)
        # But the commanded motion runs out at the caller's own last step.
        assert session.scheduled.until_server_ms == 5_050


@pytest.mark.asyncio
async def test_a_fully_elapsed_chunk_is_dropped_instead_of_sent():
    """A chunk whose every waypoint has passed is not worth sending at all."""
    due = asyncio.Event()

    async def boundary() -> None:
        due.set()

    async with _started(pipeline=_unpadded()) as (session, server):
        session.update_chunk(
            steps=[[1.0] * 6, [2.0] * 6],
            dt_ms=1.0,
            first_timestamp_ms=0,
            when_due=(1, boundary),
        )
        await _wait_until(lambda: session.scheduled.chunk_count == session.queued_chunk_count)

        waypoint_requests = [r for r in server.requests if _is_joint_chunk(_inner(r))]
        assert waypoint_requests == []
        # Nothing went on the wire...
        assert session.scheduled.waypoint_timestamps == ()
        # ...but the chunk still counts as scheduled. Callers wait for this to
        # reach what they queued, so skipping it silently strands them forever.
        assert session.scheduled.chunk_count == session.queued_chunk_count
        # Its steps keep the timestamps they were given, all of them in the past,
        # which is what lets a policy boundary read as already due.
        await asyncio.wait_for(due.wait(), 0.5)


@pytest.mark.asyncio
async def test_live_updates_are_sent_without_waiting_for_the_next_response():
    first = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    stale = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    freshest = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
    async with _started(pipeline=_unpadded(), response_delay=0.2) as (session, server):
        session.update_chunk(steps=[first], dt_ms=100.0)
        await _wait_until(lambda: len(server.requests) >= 2)

        # Back-to-back updates coalesce to the freshest pending target, and it
        # is sent before the deliberately slow next server acknowledgement.
        session.update_chunk(steps=[stale], dt_ms=100.0)
        session.update_chunk(steps=[freshest], dt_ms=100.0)
        await _wait_until(lambda: len(server.requests) >= 3, timeout=0.1)

        sent = _inner(server.requests[2])
        assert _is_joint_chunk(sent)
        assert [_joints(waypoint) for waypoint in sent.waypoints] == [freshest]


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("server_fault", "error"),
    [
        ({"fault": _motion_error("joint_limit")}, MotionError),
        # An old gateway rejecting the websocket with HTTP 404.
        ({"raise_exc": InvalidStatus(SimpleNamespace(status_code=404))}, JoggingNotSupportedError),
    ],
    ids=["motion-error", "missing-endpoint-404"],
)
async def test_a_server_fault_surfaces_as_a_typed_failure(
    server_fault: dict[str, object], error: type[Exception]
):
    """A MOTION_ERROR response or a rejected websocket turns into has_failed + a typed error."""
    async with _started(**server_fault, ready=False) as (session, _):
        await _wait_until(lambda: session.has_failed)
        assert session.has_failed is True
        assert isinstance(session.failure_exception, error)


@pytest.mark.asyncio
async def test_a_stop_condition_during_startup_ends_waiting_instead_of_timing_out():
    """A stop condition can end the session before it is ready (e.g. a part detected where
    the policy cannot go); waiting must end normally so the run reports the condition."""
    with _built() as (session, _):
        session._stop_condition = "part_outside_trained_positions"
        started = time.monotonic()
        await session.wait_ready(timeout_s=2.0)
        assert time.monotonic() - started < 0.5
        assert session.stop_condition_triggered == "part_outside_trained_positions"


@pytest.mark.asyncio
async def test_a_dead_state_stream_surfaces_instead_of_timing_out_startup():
    """Startup depends on the state stream, so its death has to be reported.

    Server/client clock skew is calibrated from that stream, so without it
    ``wait_ready`` can only run out its timeout and complain about a missing
    calibration — which says nothing about the stream having died and buries the
    error. The caller should get the real cause, promptly.
    """
    broken = OSError("state stream reset by peer")

    async def dying_stream(**_kw: object):
        raise broken
        yield  # pragma: no cover - generator marker

    # Without a state stream nothing can report the stop settled: don't wait it out.
    with _built() as (session, server), patch(f"{_SESSION}._STOP_TIMEOUT_S", 0.05):
        session._motion_group.stream_state = MagicMock(side_effect=dying_stream)
        try:
            await session.start()
            with pytest.raises(RuntimeError, match="state stream reset by peer"):
                await session.wait_ready(timeout_s=2.0)
            assert session.has_failed is True
            assert session.failure_exception is broken
        finally:
            await session.stop()
            server.stop()


@pytest.mark.asyncio
async def test_the_state_stream_updates_the_observable_robot_state():
    """current_state reflects joints pushed by the server state stream; jogging_state follows
    the stream's jogging state."""
    moved = SimpleNamespace(joints=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    states = [_stream_state(moved, kind="RUNNING")]
    async with _started(states=states, ready=False) as (session, _):
        assert session.jogging_state is None  # no execution state reported yet
        await _wait_until(
            lambda: (
                session.current_state is not None
                and list(session.current_state.joints) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
            )
        )
        await _wait_until(lambda: session.jogging_state == "RUNNING")


@pytest.mark.asyncio
async def test_standstill_is_reported_once_the_joints_stop_changing():
    """``is_at_standstill`` is driven by the joint positions, not by a pause state.

    The stream here only ever reports ``RUNNING`` — which is all a NOVA 26.6
    jogger reports once a waypoint queue drains — so a session that inferred
    standstill from the jogging state would never report it at all.
    """
    still = SimpleNamespace(joints=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    # Identical joints, advancing server timestamps: a robot sitting still.
    states = [_stream_state(still, ts_ms=1_000 + 10 * i, kind="RUNNING") for i in range(20)]
    async with _started(states=states, ready=False) as (session, _server):
        assert session.is_at_standstill is False  # no state sampled yet
        await _wait_until(lambda: session.is_at_standstill)
        assert session.jogging_state == "RUNNING"  # never anything else


@pytest.mark.asyncio
async def test_a_moving_robot_is_never_reported_as_at_standstill():
    """Joints changing between samples keep resetting the hold."""
    moving = [
        _stream_state(
            SimpleNamespace(joints=tuple(0.1 * i for _ in range(6))),
            ts_ms=1_000 + 10 * i,
            kind="RUNNING",
        )
        for i in range(1, 25)
    ]
    async with _started(states=moving) as (session, _server):
        # Sample across the whole moving stretch; it must never read as settled.
        for _ in range(20):
            assert session.is_at_standstill is False
            await asyncio.sleep(0.003)


@pytest.mark.asyncio
async def test_a_fired_stop_condition_ends_the_session_without_failing():
    """A stop condition that returns True stops the session as a normal end."""

    def workspace_limit(_ctx: object) -> bool:
        return True

    async with _started(stop_conditions=[workspace_limit], ready=False) as (session, _server):
        await _wait_until(lambda: session.stop_condition_triggered is not None)
        assert session.stop_condition_triggered == "workspace_limit"
        assert session.has_failed is False


@pytest.mark.asyncio
async def test_jog_tcp_chunk_is_sent_as_evenly_spaced_pose_waypoints():
    """A TCP chunk pushed through ``jog_tcp`` reaches the server as a single
    ``ActionChunkRequest`` whose waypoints carry the chunk's poses and
    timestamps laid out exactly as the waypoint API expects: absolute,
    non-negative, strictly increasing, and spaced by ``dt_ms``.

    This drives the real session end-to-end (only the API gateway, estop and
    Rerun are stubbed), so the timestamps asserted here are the actual values
    the server would receive.
    """
    dt_ms = 50.0
    chunk = [
        [10.0, 0.0, 300.0, 0.0, 0.0, 0.0],
        [20.0, 0.0, 300.0, 0.0, 0.0, 0.0],
        [30.0, 0.0, 300.0, 0.0, 0.0, 0.0],
    ]
    estop = MagicMock(start=AsyncMock(), stop=AsyncMock())
    # No jogger_session_timestamp_ms (ts_ms=0 is ignored), so server-time tracks client-time
    # and dt is used verbatim. jog_tcp opens its own session on the built motion group and
    # the patched gateway.
    running = _stream_state(_initial_state(), kind="RUNNING")
    with (
        _built(mode="cartesian", states=[running]) as (built, server),
        patch("novapolicy.jogging.lifecycle.EstopMonitor", return_value=estop),
        patch("novapolicy.rerun._is_rerun_active", return_value=False),
    ):
        try:
            async with jog_tcp(built._motion_group, tcp="Flange") as jogger:
                jogger.set_chunk(chunk, dt_ms=dt_ms)
                await _wait_until(lambda: any(_is_pose_chunk(_inner(r)) for r in server.requests))
        finally:
            server.stop()

        pose_req = next(_inner(r) for r in server.requests if _is_pose_chunk(_inner(r)))
        waypoints = pose_req.waypoints

        # The chunk steps became pose waypoints, in order, starting at the first
        # step that is still reachable (earlier ones are trimmed as past). A
        # chunk shorter than the configured horizon is extended by holding its
        # final pose, so the robot is never handed a lone terminal waypoint.
        sent_positions = [[_pose(w).position[k] for k in range(3)] for w in waypoints]
        chunk_positions = [step[:3] for step in chunk]
        offset = chunk_positions.index(sent_positions[0])
        for sent, expected in zip(sent_positions, chunk_positions[offset:], strict=False):
            assert sent == expected
        for waypoint in waypoints:
            assert [_pose(waypoint).orientation[k] for k in range(3)] == chunk[0][3:6]

        # Timestamps: the layout the waypoint API expects.
        timestamps = [w.timestamp for w in waypoints]
        assert timestamps[0] >= 0  # absolute anchor on the session timeline
        assert timestamps == sorted(timestamps)  # monotonically increasing
        assert all(
            timestamps[i + 1] - timestamps[i] == int(dt_ms) for i in range(len(timestamps) - 1)
        )  # evenly spaced by dt_ms — server milliseconds are never rescaled


@pytest.mark.asyncio
async def test_overlapping_joint_chunks_share_one_absolute_timeline():
    """Overlapping chunks place identical targets at identical absolute timestamps."""

    def _joint_requests() -> list[object]:
        return [r for r in (_inner(x) for x in server.requests) if _is_joint_chunk(r)]

    async with _started(mode="joint") as (session, server):
        dt = 50.0
        # Tick 1: anchor at 100ms, j0 advancing.
        joints_a = [[v, 0.0, 0.0, 0.0, 0.0, 0.0] for v in (0.1, 0.2, 0.3, 0.4, 0.5)]
        session.update_chunk(steps=joints_a, dt_ms=dt, first_timestamp_ms=100)
        await _wait_until(lambda: len(_joint_requests()) >= 1)

        # Tick 2, one dt later: anchor +dt, content shifts one step -> 4 overlap.
        joints_b = [[v, 0.0, 0.0, 0.0, 0.0, 0.0] for v in (0.2, 0.3, 0.4, 0.5, 0.6)]
        session.update_chunk(steps=joints_b, dt_ms=dt, first_timestamp_ms=150)
        await _wait_until(lambda: len(_joint_requests()) >= 2)

        first, second = _joint_requests()[0], _joint_requests()[1]
        # Compare only the caller's own waypoints: anything beyond them is hold
        # padding that exists to avoid a terminal waypoint, and it is always
        # superseded by the next chunk before the robot reaches it.
        a_by_ts = {w.timestamp: tuple(_joints(w)) for w in first.waypoints[: len(joints_a)]}
        b_by_ts = {w.timestamp: tuple(_joints(w)) for w in second.waypoints[: len(joints_b)]}

        overlap = set(a_by_ts) & set(b_by_ts)
        assert overlap == {150, 200, 250, 300}
        for ts in overlap:
            assert a_by_ts[ts] == b_by_ts[ts]


# ---------------------------------------------------------------------------
# drain — waypoints already accepted are owed to the caller
# ---------------------------------------------------------------------------


def _chunk_requests(server: FakeJoggingServer) -> list[object]:
    return [_inner(r) for r in server.requests if _is_joint_chunk(_inner(r))]


@pytest.mark.asyncio
async def test_drain_stops_on_the_last_target_instead_of_keeping_the_velocity():
    """With a delay until braking the robot would run on past the last target."""
    async with _started(
        config=WaypointConfig(delay_until_braking_ms=200), pipeline=_unpadded()
    ) as (session, server):
        session.update_chunk(steps=[[0.0] * 6, [1.0] * 6], dt_ms=50.0, first_timestamp_ms=5_000)
        await _wait_until(lambda: session.scheduled.until_server_ms == 5_050)

        assert await session.drain(timeout_s=0.05) is False
        chunk, final = _chunk_requests(server)
        assert chunk.delay_until_braking == 200
        assert final.delay_until_braking == 0
        assert [(w.timestamp, _joints(w)) for w in final.waypoints] == [(5_050, [1.0] * 6)]


@pytest.mark.asyncio
async def test_a_final_target_already_passed_is_still_sent_to_stop_on():
    """A robot keeping its velocity past the last target has to come back to it."""
    async with _started(
        config=WaypointConfig(delay_until_braking_ms=200), pipeline=_unpadded()
    ) as (session, server):
        session.update_chunk(steps=[[1.0] * 6], dt_ms=50.0, first_timestamp_ms=5_000)
        await _wait_until(lambda: session.scheduled.until_server_ms == 5_000)
        session._clock.update(5_100)

        await session.drain(timeout_s=0.05)
        final = _chunk_requests(server)[-1]
        assert final.delay_until_braking == 0
        assert final.waypoints[0].timestamp > 5_100
        assert _joints(final.waypoints[0]) == [1.0] * 6


@pytest.mark.asyncio
async def test_drain_waits_for_acknowledgement_not_for_extrapolation():
    """A stalled link must not let a drain declare the motion finished.

    Estimated server time free-runs at wall-clock rate between state samples and
    is deliberately uncapped, so on a stalled link it sails straight past the
    schedule. Draining against it would report the path complete when the robot
    may not even have started it, and the stop that follows would truncate
    exactly the motion the drain exists to protect.
    """
    async with _started(pipeline=_unpadded()) as (session, server):
        session.update_chunk(steps=[[1.0] * 6], dt_ms=50.0, first_timestamp_ms=5_000)
        await _wait_until(lambda: session.scheduled.until_server_ms == 5_000)

        # A link that acknowledged 100ms and then went quiet: the estimate
        # extrapolates a minute past the schedule while the acknowledgement
        # stays far short of it.
        session._clock.update(100)
        session._clock._last_server_wall -= 60.0
        assert session._clock.estimated_server_timestamp_ms > 5_000
        assert session.last_server_timestamp_ms == 100

        assert await session.drain(timeout_s=0.05) is False
        # Without a delay until braking the drain sends nothing more to stop on.
        assert len(_chunk_requests(server)) == 1

        # It finishes as soon as the server actually acknowledges the schedule.
        session._clock.update(5_000)
        assert await session.drain(timeout_s=1.0) is True


@pytest.mark.asyncio
async def test_drain_gives_up_when_a_stop_condition_fires_mid_drain():
    """A stop condition wants the robot stopped, not the rest of the path run."""
    async with _started(pipeline=_unpadded()) as (session, _server):
        session.update_chunk(steps=[[1.0] * 6], dt_ms=50.0, first_timestamp_ms=9_000_000)
        await _wait_until(lambda: session.scheduled.until_server_ms == 9_000_000)

        session._stop_condition = "force_exceeded"

        assert await session.drain(timeout_s=5.0) is False


@pytest.mark.asyncio
async def test_drain_on_a_session_that_never_scheduled_anything_returns_at_once():
    """Nothing sent means nothing owed."""
    async with _started() as (session, _server):
        assert await session.drain(timeout_s=0.01) is True


@pytest.mark.asyncio
async def test_drain_waits_for_a_chunk_that_is_still_only_queued():
    """The caller's last target may not be timestamped yet when the drain starts.

    ``update_chunk`` only queues; the jogging task timestamps and sends it a tick
    later. Reading the schedule once at entry would finish the drain before that
    chunk ever went out, and ``stop`` would then cancel it mid-flight.
    """
    async with _started(pipeline=_unpadded()) as (session, _server):
        # Queue without letting the jogging task pick it up first.
        session.update_chunk(steps=[[1.0] * 6], dt_ms=50.0, first_timestamp_ms=5_000)
        assert session._pending_request is not None

        # Nothing is scheduled yet, so a snapshotting drain would return True here.
        assert await session.drain(timeout_s=0.05) is False
        # ...and it was the queued chunk that held it, not an empty schedule.
        assert session.scheduled.until_server_ms == 5_000


# ---------------------------------------------------------------------------
# Unknown keyword arguments
# ---------------------------------------------------------------------------


def _bare_session() -> WaypointJoggingSession:
    """A session for testing ``update_chunk`` alone, which talks to no gateway."""
    mg = MagicMock()
    mg.id = "0@ur10e"
    return WaypointJoggingSession(motion_group=mg, config=WaypointConfig(), tcp="Flange")


def test_an_unknown_keyword_is_refused() -> None:
    """A misspelled motion parameter must not be accepted and ignored."""
    session = _bare_session()
    with pytest.raises(TypeError, match="unexpected keyword argument"):
        session.update_chunk(steps=[[0.0] * 6], dt_ms=20.0, timestamp_offest_steps=1)


@pytest.mark.asyncio
@pytest.mark.parametrize("clamp", [True, False])
async def test_the_session_reads_joint_limits_only_for_a_joint_clamp(clamp: bool):
    """A JointClamp gets the robot's limits, read once at the start; without one the session
    makes no extra call."""
    from novapolicy.jogging.modules import JointClamp
    from novapolicy.joint_limits import JointLimits

    limits = JointLimits(lower=(-1.0,) * 6, upper=(1.0,) * 6)
    pipeline = [JointClamp(), *_unpadded()] if clamp else _unpadded()
    fetch = AsyncMock(return_value={"0@ur10e": limits})
    with patch(f"{_SESSION}.fetch_joint_limits", fetch):
        async with _started(pipeline=pipeline) as (session, server):
            session.update_chunk(steps=[[2.0, -2.0, 0.5, 0.0, 0.0, 0.0]], dt_ms=50.0)
            await _wait_until(lambda: len(server.requests) >= 2)
            sent = _joints(_inner(server.requests[1]).waypoints[0])
            assert fetch.await_count == (1 if clamp else 0)
            assert sent[:3] == ([1.0, -1.0, 0.5] if clamp else [2.0, -2.0, 0.5])


# ---------------------------------------------------------------------------
# Stop handshake (ADR 0019)
# ---------------------------------------------------------------------------


def _jogging_state(details_state: object, ts_ms: int) -> object:
    """A state packet whose jogging state is a real API model, as NOVA streams it."""
    details = SimpleNamespace(jogger_session_timestamp_ms=ts_ms, state=details_state)
    return SimpleNamespace(
        joint_position=[0.0] * 6,
        tcp_pose=None,
        tcp="Flange",
        execute=SimpleNamespace(details=details),
        timestamp=datetime.now(UTC),
    )


@pytest.mark.asyncio
async def test_stop_asks_the_server_and_waits_for_its_acknowledgement_before_closing():
    """The stop request goes out on the open stream, and the session closes only once NOVA
    acknowledged it and then reported the jogging session no longer running; a paused state
    before the acknowledgement is no answer."""
    stopped_reported = asyncio.Event()
    server_ref: list[FakeJoggingServer] = []

    async def state_stream(**_kw: object) -> AsyncGenerator[object, None]:
        ts = 0
        while True:
            ts += 10
            server = server_ref[0]
            if server.stop_acknowledged and stopped_reported.is_set():
                yield _jogging_state(api.models.JoggingPausedByUser(), ts)
            elif server.stop_acknowledged:
                yield _jogging_state(api.models.JoggingRunning(), ts)
            elif server.stop_received.is_set():
                # Not acknowledged yet: a paused state now must not end the wait.
                yield _jogging_state(api.models.JoggingPausedByUser(), ts)
            else:
                yield _jogging_state(api.models.JoggingRunning(), ts)
            await asyncio.sleep(0.003)

    with _built(pipeline=_unpadded()) as (session, server):
        server_ref.append(server)
        server.stop_ack_gate = asyncio.Event()
        session._motion_group.stream_state = MagicMock(side_effect=state_stream)
        try:
            await session.start()
            await session.wait_ready()
            session.update_chunk(steps=[[0.1] * 6], dt_ms=50.0)
            await _wait_until(lambda: len(server.requests) >= 2)

            stopping = asyncio.create_task(session.stop())
            await asyncio.wait_for(server.stop_received.wait(), timeout=1.0)
            await asyncio.sleep(0.05)
            assert not stopping.done()  # paused, but not acknowledged
            assert not server.closed

            server.stop_ack_gate.set()
            await _wait_until(lambda: server.stop_acknowledged)
            await asyncio.sleep(0.05)
            assert not stopping.done()  # acknowledged, but still RUNNING
            assert not server.closed

            stopped_reported.set()
            await asyncio.wait_for(stopping, timeout=1.0)
            await _wait_until(lambda: server.closed)
            assert isinstance(_inner(server.requests[-1]), api.models.StopActionChunksRequest)
            assert (
                sum(
                    isinstance(_inner(r), api.models.StopActionChunksRequest)
                    for r in server.requests
                )
                == 1
            )
        finally:
            await session.stop()
            server.stop()


@pytest.mark.asyncio
async def test_a_stop_the_server_never_acknowledges_ends_after_the_stop_timeout(caplog):
    """A NOVA that never confirms the stop costs the timeout and a warning, then the session
    closes the stream anyway."""
    with patch(f"{_SESSION}._STOP_TIMEOUT_S", 0.2):
        async with _started(acknowledge_stop=False) as (session, server):
            await _wait_until(lambda: len(server.requests) >= 1)
            started = time.monotonic()
            with caplog.at_level(logging.WARNING, logger=_SESSION):
                await session.stop()
            elapsed = time.monotonic() - started
            await _wait_until(lambda: server.closed)

    assert elapsed >= 0.2
    assert isinstance(_inner(server.requests[-1]), api.models.StopActionChunksRequest)
    assert not server.stop_acknowledged
    assert "did not report the jogging session stopped" in caplog.text
