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
from datetime import UTC, datetime
from types import SimpleNamespace
from typing import TYPE_CHECKING
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from nova import api
from nova.types import Pose
from novapolicy.jogging import jog_tcp
from novapolicy.jogging.waypoint_session import MIN_LEAD_MS, WaypointJoggingSession
from novapolicy.types import JoggingNotSupportedError, MotionError, WaypointConfig

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, Callable, Sequence

_SESSION = "novapolicy.jogging.waypoint_session"


# ---------------------------------------------------------------------------
# Fake server: stands in for api_gateway.action_chunk_streaming_api.execute_action_chunks
# ---------------------------------------------------------------------------


def _ok() -> object:
    """A normal jogging response (no motion error)."""
    return SimpleNamespace(root=SimpleNamespace())


def _motion_error(message: str = "joint_limit") -> object:
    """A response that tells the session the server hit a motion error."""
    return SimpleNamespace(root=SimpleNamespace(kind="MOTION_ERROR", message=message))


class FakeJoggingServer:
    """Consumes the session's request generator and replays scripted responses.

    The session yields an ``InitializeActionChunksRequest`` first, then action
    chunk requests independently of response latency. We keep emitting ``_ok()``
    responses on a configurable cadence until stopped, recording every request
    the session produces.
    """

    def __init__(
        self,
        *,
        fault: object | None = None,
        raise_exc: BaseException | None = None,
        response_delay: float = 0.003,
    ) -> None:
        self.requests: list[object] = []
        self._fault = fault
        self._raise_exc = raise_exc
        self._response_delay = response_delay
        self._stop = asyncio.Event()

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
                yield _ok()
                await asyncio.sleep(self._response_delay)

        async for request in client_request_generator(responses()):
            self.requests.append(request)

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
) -> tuple[WaypointJoggingSession, FakeJoggingServer]:
    server = FakeJoggingServer(
        fault=fault,
        raise_exc=raise_exc,
        response_delay=response_delay,
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
    )

    patches = [
        patch(f"{_SESSION}.get_api_gateway", return_value=gateway),
        patch(f"{_SESSION}.get_cell", return_value="cell"),
        patch(f"{_SESSION}.get_controller_id", return_value="0@ur10e"),
    ]
    for p in patches:
        p.start()
    session._test_patches = patches  # type: ignore[attr-defined]  # torn down in _run
    return session, server


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
        waypoint.waypoint.root.kind == "JOINTS" for waypoint in message.waypoints
    )


def _is_pose_chunk(message: object) -> bool:
    """True for an ActionChunkRequest whose waypoints carry pose coordinates."""
    return isinstance(message, api.models.ActionChunkRequest) and all(
        waypoint.waypoint.root.kind == "POSE" for waypoint in message.waypoints
    )


def _joints(waypoint: object) -> list[float]:
    """Joint values of one waypoint, past the coordinates envelope."""
    return list(waypoint.waypoint.root.joints.root)


def _pose(waypoint: object) -> object:
    """Pose of one waypoint, past the coordinates envelope."""
    return waypoint.waypoint.root.pose


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_it_initializes_the_jogging_session_before_any_waypoints():
    """The very first message to the server is an InitializeActionChunksRequest."""
    session, server = _build_session()
    try:
        await session.start()
        await session.wait_ready()
        await _wait_until(lambda: len(server.requests) >= 1)
        assert isinstance(_inner(server.requests[0]), api.models.InitializeActionChunksRequest)
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_queued_joint_chunk_is_sent_as_a_timestamped_waypoint():
    """update_chunk(step) reaches the server as a joint-flavoured ActionChunkRequest."""
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
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
        assert session.scheduled_action_timestep == 7
        assert session.scheduled_waypoint_timestamps == (5_000,)
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_short_chunk_holds_its_final_target_to_fill_the_buffer():
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=100.0))
    target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    try:
        await session.start()
        await session.wait_ready()
        session.update_chunk(steps=[target], dt_ms=50.0, first_timestamp_ms=5_000)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        assert _is_joint_chunk(request)
        assert [_joints(waypoint) for waypoint in request.waypoints] == [target, target]
        assert [waypoint.timestamp for waypoint in request.waypoints] == [5_000, 5_050]
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_waypoints_already_in_the_past_are_trimmed_before_sending():
    """Only the still-reachable tail of a chunk is sent.

    A chunk is timestamped when the caller builds it but sent slightly later.
    Waypoints whose moment has passed are unreachable, and commanding them makes
    the server jump to catch up, which is heavy velocity ripple on the robot.
    The absolute time-to-position mapping of the surviving waypoints is
    unchanged, so trimming never distorts the trajectory.
    """
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
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
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_trimmed_chunk_reports_the_timestep_it_actually_starts_at():
    """``scheduled_action_timestep`` names the policy step ``steps[0]`` came from.

    Trimming drops leading waypoints, so the first waypoint sent is not the
    caller's step zero. Reporting the untrimmed timestep misattributes every
    plotted and logged waypoint to a step the robot never received.
    """
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
        steps = [[float(i)] * 6 for i in range(6)]
        session.update_chunk(steps=steps, dt_ms=50.0, first_timestamp_ms=0, action_timestep=100)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        skipped = len(steps) - len(request.waypoints)

        assert skipped > 0  # the premise: something was trimmed
        assert session.scheduled_action_timestep == 100 + skipped
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_trimmed_chunk_still_reports_where_each_caller_step_landed():
    """Trimming must stay invisible to anyone asking about their own steps.

    Callers place meaning on step indices — a policy queue reads the absolute
    timestamp of its own waypoint zero off the session and treats it as the
    immutable origin of the whole queue. Trimming drops leading waypoints, so
    indexing the *sent* request shifts that origin by however many were dropped,
    on every chunk, compounding.
    """
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
        steps = [[float(i)] * 6 for i in range(6)]
        session.update_chunk(steps=steps, dt_ms=50.0, first_timestamp_ms=0)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        assert _is_joint_chunk(request)
        skipped = len(steps) - len(request.waypoints)
        assert skipped > 0  # the premise: something was trimmed

        # Every caller step keeps the slot it was laid out on, sent or not.
        for step in range(len(steps)):
            assert session.scheduled_timestamp_for_step(step) == step * 50
        # A surviving step resolves to the timestamp it was really sent with.
        assert session.scheduled_timestamp_for_step(skipped) == request.waypoints[0].timestamp
        # And a step the caller never passed has no answer.
        assert session.scheduled_timestamp_for_step(len(steps)) is None
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_the_padded_tail_of_a_short_chunk_is_not_motion_anyone_waits_for():
    """``scheduled_until_server_ms`` ends at the caller's last step.

    A short chunk is padded to ``min_chunk_horizon_ms`` by repeating its final target,
    which gives the server room to brake. Treating that hold as commanded motion
    makes every caller waiting for the chunk to finish sit at the final target
    for the rest of the buffer first — at a 500ms buffer, most of the wait.
    """
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=200.0))
    target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    try:
        await session.start()
        await session.wait_ready()
        session.update_chunk(steps=[target, target], dt_ms=50.0, first_timestamp_ms=5_000)

        await _wait_until(lambda: len(server.requests) >= 2)
        request = _inner(server.requests[1])
        assert _is_joint_chunk(request)
        sent = [waypoint.timestamp for waypoint in request.waypoints]

        # The padding really was sent, so the server still gets its horizon.
        assert sent == [5_000, 5_050, 5_100, 5_150]
        assert session.scheduled_waypoint_timestamps == (5_000, 5_050, 5_100, 5_150)
        # But the commanded motion runs out at the caller's own last step.
        assert session.scheduled_until_server_ms == 5_050
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_fully_elapsed_chunk_is_dropped_instead_of_sent():
    """A chunk whose every waypoint has passed is not worth sending at all."""
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
        session.update_chunk(steps=[[1.0] * 6, [2.0] * 6], dt_ms=1.0, first_timestamp_ms=0)
        await _wait_until(lambda: session.scheduled_chunk_count == session.queued_chunk_count)

        waypoint_requests = [r for r in server.requests if _is_joint_chunk(_inner(r))]
        assert waypoint_requests == []
        # Nothing went on the wire...
        assert session.scheduled_waypoint_timestamps == ()
        # ...but the chunk still counts as scheduled. Callers wait for this to
        # reach what they queued, so skipping it silently strands them forever.
        assert session.scheduled_chunk_count == session.queued_chunk_count
        # Its steps keep the timestamps they were given, all of them in the past,
        # which is what lets a policy boundary read as already due.
        assert session.scheduled_timestamp_for_step(0) == 0
        assert session.scheduled_timestamp_for_step(1) == 1
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_live_updates_are_sent_without_waiting_for_the_next_response():
    session, server = _build_session(
        config=WaypointConfig(min_chunk_horizon_ms=0),
        response_delay=0.2,
    )
    first = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    stale = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    freshest = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
    try:
        await session.start()
        await session.wait_ready()

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
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_motion_error_response_surfaces_as_a_failure():
    """A server MOTION_ERROR turns into has_failed + a MotionError."""
    session, server = _build_session(fault=_motion_error("joint_limit"))
    try:
        await session.start()
        await _wait_until(lambda: session.has_failed)
        assert session.has_failed is True
        assert isinstance(session.failure_exception, MotionError)
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_dead_state_stream_surfaces_instead_of_timing_out_startup():
    """Startup depends on the state stream, so its death has to be reported.

    Server/client clock skew is calibrated from that stream, so without it
    ``wait_ready`` can only run out its timeout and complain about a missing
    calibration — which says nothing about the stream having died and buries the
    error. The caller should get the real cause, promptly.
    """
    session, server = _build_session()
    broken = OSError("state stream reset by peer")

    async def dying_stream(**_kw: object):
        raise broken
        yield  # pragma: no cover - generator marker

    session._motion_group.stream_state = MagicMock(side_effect=dying_stream)
    try:
        await session.start()
        with pytest.raises(RuntimeError, match="state stream reset by peer"):
            await session.wait_ready(timeout_s=2.0)
        assert session.has_failed is True
        assert session.failure_exception is broken
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_missing_endpoint_404_surfaces_as_jogging_not_supported():
    """An old gateway rejecting the websocket with HTTP 404 surfaces a typed error."""
    from websockets.exceptions import InvalidStatus

    not_found = InvalidStatus(SimpleNamespace(status_code=404))  # type: ignore[arg-type]
    session, server = _build_session(raise_exc=not_found)
    try:
        await session.start()
        await _wait_until(lambda: session.has_failed)
        assert isinstance(session.failure_exception, JoggingNotSupportedError)
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_the_state_stream_updates_the_observable_robot_state():
    """current_state reflects joints pushed by the server state stream."""
    moved = SimpleNamespace(joints=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    session, server = _build_session(states=[_stream_state(moved)])
    try:
        await session.start()
        await _wait_until(
            lambda: (
                session.current_state is not None
                and list(session.current_state.joints) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
            )
        )
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_is_running_reflects_the_jogging_state():
    """is_running follows the stream's RUNNING jogging state."""
    moved = SimpleNamespace(joints=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    session, server = _build_session(states=[_stream_state(moved, kind="RUNNING")])
    try:
        await session.start()
        assert session.is_running is False  # no execution state reported yet
        await _wait_until(lambda: session.is_running)
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


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
    session, server = _build_session(states=states)
    try:
        await session.start()
        assert session.is_at_standstill is False  # no state sampled yet
        await _wait_until(lambda: session.is_at_standstill)
        assert session.jogging_state == "RUNNING"  # never anything else
        assert session.standstill_ms > 0
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


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
    session, server = _build_session(states=moving)
    try:
        await session.start()
        await session.wait_ready()
        # Sample across the whole moving stretch; it must never read as settled.
        for _ in range(20):
            assert session.is_at_standstill is False
            await asyncio.sleep(0.003)
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_fired_stop_condition_ends_the_session_without_failing():
    """A stop condition that returns True stops the session as a normal end."""

    def workspace_limit(_ctx: object) -> bool:
        return True

    session, server = _build_session(stop_conditions=[workspace_limit])
    try:
        await session.start()
        await _wait_until(lambda: session.stop_condition_triggered is not None)
        assert session.stop_condition_triggered == "workspace_limit"
        assert session.has_failed is False
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


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
    server = FakeJoggingServer()

    async def state_stream(**_kw: object) -> AsyncGenerator[object, None]:
        # No jogger_session_timestamp_ms (ts_ms=0 is ignored), so server-time
        # tracks client-time and dt is used verbatim.
        while True:
            yield _stream_state(_initial_state(), kind="RUNNING")
            await asyncio.sleep(0.003)

    mg = MagicMock()
    mg.id = "0@ur10e"
    mg.get_state = AsyncMock(return_value=_initial_state())
    mg.stream_state = MagicMock(side_effect=state_stream)
    mg.active_tcp_name = AsyncMock(return_value="Flange")
    mg.tcp_names = AsyncMock(return_value=["Flange"])

    gateway = MagicMock()
    gateway.action_chunk_streaming_api.execute_action_chunks = server.execute_action_chunks

    estop = MagicMock()
    estop.start = AsyncMock()
    estop.stop = AsyncMock()

    dt_ms = 50.0
    chunk = [
        [10.0, 0.0, 300.0, 0.0, 0.0, 0.0],
        [20.0, 0.0, 300.0, 0.0, 0.0, 0.0],
        [30.0, 0.0, 300.0, 0.0, 0.0, 0.0],
    ]

    patches = [
        patch(f"{_SESSION}.get_api_gateway", return_value=gateway),
        patch(f"{_SESSION}.get_cell", return_value="cell"),
        patch(f"{_SESSION}.get_controller_id", return_value="0@ur10e"),
        patch("novapolicy.jogging.jogger.EstopMonitor", return_value=estop),
        patch("novapolicy.rerun._is_rerun_active", return_value=False),
    ]
    for p in patches:
        p.start()
    try:
        async with jog_tcp(mg, tcp="Flange") as jogger:
            jogger.set_chunk(chunk, dt_ms=dt_ms)
            await _wait_until(lambda: any(_is_pose_chunk(_inner(r)) for r in server.requests))

        pose_req = next(_inner(r) for r in server.requests if _is_pose_chunk(_inner(r)))
        waypoints = pose_req.waypoints

        # The chunk steps became pose waypoints, in order, starting at the first
        # step that is still reachable (earlier ones are trimmed as past). A
        # chunk shorter than the configured horizon is extended by holding its
        # final pose, so the robot is never handed a lone terminal waypoint.
        sent_positions = [[_pose(w).position.root[k] for k in range(3)] for w in waypoints]
        chunk_positions = [step[:3] for step in chunk]
        offset = chunk_positions.index(sent_positions[0])
        for sent, expected in zip(sent_positions, chunk_positions[offset:], strict=False):
            assert sent == expected
        for waypoint in waypoints:
            assert [_pose(waypoint).orientation.root[k] for k in range(3)] == chunk[0][3:6]

        # Timestamps: the layout the waypoint API expects.
        timestamps = [w.timestamp for w in waypoints]
        assert timestamps[0] >= 0  # absolute anchor on the session timeline
        assert timestamps == sorted(timestamps)  # monotonically increasing
        assert all(
            timestamps[i + 1] - timestamps[i] == int(dt_ms) for i in range(len(timestamps) - 1)
        )  # evenly spaced by dt_ms — server milliseconds are never rescaled
    finally:
        server.stop()
        for p in patches:
            p.stop()


@pytest.mark.asyncio
async def test_overlapping_joint_chunks_share_one_absolute_timeline():
    """Overlapping chunks place identical targets at identical absolute timestamps."""
    session, server = _build_session(mode="joint")

    def _joint_requests() -> list[object]:
        return [r for r in (_inner(x) for x in server.requests) if _is_joint_chunk(r)]

    try:
        await session.start()
        await session.wait_ready()

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
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


# ---------------------------------------------------------------------------
# drain — waypoints already accepted are owed to the caller
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_drain_waits_until_the_server_clock_reaches_the_last_waypoint():
    """ "Finished" is the server clock passing the last timestamp that was sent."""
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
        session.update_chunk(steps=[[1.0] * 6], dt_ms=50.0, first_timestamp_ms=5_000)
        await _wait_until(lambda: session.scheduled_until_server_ms == 5_000)

        # Server time is far short of the schedule, so a drain cannot finish.
        drain = asyncio.create_task(session.drain(timeout_s=0.05))
        assert await drain is False

        # Once the clock is past it, the drain returns immediately.
        session._clock.update(6_000)
        assert await session.drain(timeout_s=1.0) is True
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_drain_waits_for_acknowledgement_not_for_extrapolation():
    """A stalled link must not let a drain declare the motion finished.

    Estimated server time free-runs at wall-clock rate between state samples and
    is deliberately uncapped, so on a stalled link it sails straight past the
    schedule. Draining against it would report the path complete when the robot
    may not even have started it, and the stop that follows would truncate
    exactly the motion the drain exists to protect.
    """
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
        session.update_chunk(steps=[[1.0] * 6], dt_ms=50.0, first_timestamp_ms=5_000)
        await _wait_until(lambda: session.scheduled_until_server_ms == 5_000)

        # A link that acknowledged 100ms and then went quiet: the estimate
        # extrapolates a minute past the schedule while the acknowledgement
        # stays far short of it.
        session._clock.update(100)
        session._clock._last_server_wall -= 60.0
        assert session._clock.estimated_server_timestamp_ms > 5_000
        assert session.last_server_timestamp_ms == 100

        assert await session.drain(timeout_s=0.05) is False

        # It finishes as soon as the server actually acknowledges the schedule.
        session._clock.update(5_000)
        assert await session.drain(timeout_s=1.0) is True
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_drain_gives_up_when_a_stop_condition_fires_mid_drain():
    """A stop condition wants the robot stopped, not the rest of the path run."""
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
        session.update_chunk(steps=[[1.0] * 6], dt_ms=50.0, first_timestamp_ms=9_000_000)
        await _wait_until(lambda: session.scheduled_until_server_ms == 9_000_000)

        session._stop_condition = "force_exceeded"

        assert await session.drain(timeout_s=5.0) is False
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_drain_on_a_session_that_never_scheduled_anything_returns_at_once():
    """Nothing sent means nothing owed."""
    session, server = _build_session()
    try:
        await session.start()
        await session.wait_ready()

        assert await session.drain(timeout_s=0.01) is True
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_drain_waits_for_a_chunk_that_is_still_only_queued():
    """The caller's last target may not be timestamped yet when the drain starts.

    ``update_chunk`` only queues; the jogging task timestamps and sends it a tick
    later. Reading the schedule once at entry would finish the drain before that
    chunk ever went out, and ``stop`` would then cancel it mid-flight.
    """
    session, server = _build_session(config=WaypointConfig(min_chunk_horizon_ms=0))
    try:
        await session.start()
        await session.wait_ready()
        # Queue without letting the jogging task pick it up first.
        session.update_chunk(steps=[[1.0] * 6], dt_ms=50.0, first_timestamp_ms=5_000)
        assert session._pending_request is not None

        # Nothing is scheduled yet, so a snapshotting drain would return True here.
        assert await session.drain(timeout_s=0.05) is False
        # ...and it was the queued chunk that held it, not an empty schedule.
        assert session.scheduled_until_server_ms == 5_000
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()
