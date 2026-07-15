"""Behavioural tests for WaypointJoggingSession.

These drive the *real* async jogging + state-stream loops, substituting only
the NOVA SDK boundary: the motion group, the API gateway, and the small
``_sdk`` accessors. The fake gateway plays the role of the server — it feeds the
session a response stream and records the ``ExecuteWaypointJoggingRequest`` messages the
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
from types import SimpleNamespace
from typing import TYPE_CHECKING
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from nova import api
from nova.types import Pose
from novapolicy.jogging import jog_tcp
from novapolicy.jogging.waypoint_session import WaypointJoggingSession
from novapolicy.types import JoggingNotSupportedError, MotionError, WaypointConfig

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, Callable, Sequence

_SESSION = "novapolicy.jogging.waypoint_session"


# ---------------------------------------------------------------------------
# Fake server: stands in for api_gateway.jogging_api.execute_waypoint_jogging
# ---------------------------------------------------------------------------


def _ok() -> object:
    """A normal jogging response (no motion error)."""
    return SimpleNamespace(root=SimpleNamespace())


def _motion_error(message: str = "joint_limit") -> object:
    """A response that tells the session the server hit a motion error."""
    return SimpleNamespace(root=SimpleNamespace(kind="MOTION_ERROR", message=message))


class FakeJoggingServer:
    """Consumes the session's request generator and replays scripted responses.

    The session yields an ``InitializeJoggingRequest`` first, then one waypoint
    request per response it receives (once a chunk is queued). We keep emitting
    ``_ok()`` responses on a short cadence until stopped, recording every
    request the session produces.
    """

    def __init__(
        self, *, fault: object | None = None, raise_exc: BaseException | None = None
    ) -> None:
        self.requests: list[object] = []
        self._fault = fault
        self._raise_exc = raise_exc
        self._stop = asyncio.Event()

    async def execute_waypoint_jogging(
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
                await asyncio.sleep(0.003)

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
    )


def _build_session(
    *,
    fault: object | None = None,
    raise_exc: BaseException | None = None,
    states: Sequence[object] = (),
    stop_conditions: list[object] | None = None,
    mode: str = "joint",
    trace_trajectory: bool = False,
) -> tuple[WaypointJoggingSession, FakeJoggingServer]:
    server = FakeJoggingServer(fault=fault, raise_exc=raise_exc)

    async def state_stream(**_kw: object) -> AsyncGenerator[object, None]:
        for s in states:
            yield s
            await asyncio.sleep(0.003)
        while True:  # idle until cancelled on stop()
            await asyncio.sleep(0.01)

    mg = MagicMock()
    mg.id = "0@ur10e"
    mg.get_state = AsyncMock(return_value=_initial_state())
    mg.stream_state = MagicMock(side_effect=state_stream)
    mg.active_tcp_name = AsyncMock(return_value="Flange")
    mg.tcp_names = AsyncMock(return_value=["Flange"])

    gateway = MagicMock()
    gateway.jogging_api.execute_waypoint_jogging = server.execute_waypoint_jogging

    session = WaypointJoggingSession(
        motion_group=mg,
        config=WaypointConfig(),
        tcp="Flange",
        mode=mode,
        stop_conditions=stop_conditions,
        trace_trajectory=trace_trajectory,
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
    """Unwrap the ExecuteWaypointJoggingRequest envelope to the concrete message."""
    return getattr(request, "root", request)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_it_initializes_the_jogging_session_before_any_waypoints():
    """The very first message to the server is an InitializeJoggingRequest."""
    session, server = _build_session()
    try:
        await session.start()
        await session.wait_ready()
        await _wait_until(lambda: len(server.requests) >= 1)
        assert isinstance(_inner(server.requests[0]), api.models.InitializeJoggingRequest)
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_a_queued_joint_chunk_is_sent_as_a_timestamped_waypoint():
    """update_chunk(step) reaches the server as a JointWaypointsRequest."""
    session, server = _build_session(trace_trajectory=True)
    try:
        await session.start()
        await session.wait_ready()
        target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        session.update_chunk(
            steps=[target],
            dt_ms=50.0,
            anchor_ms=0,
            action_timestep=7,
        )

        await _wait_until(lambda: len(server.requests) >= 2)
        waypoint_req = _inner(server.requests[1])
        assert isinstance(waypoint_req, api.models.JointWaypointsRequest)
        assert len(waypoint_req.waypoints) == 1
        sent = waypoint_req.waypoints[0]
        assert list(sent.joints.root) == target
        assert sent.timestamp == 0  # absolute anchor at 0, single step
        request_trace = session.trajectory_trace["requests"]
        assert request_trace == [
            {
                "sequence": 1,
                "action_timestep": 7,
                "policy_dt_ms": 50.0,
                "anchor_ms": 0,
                "anchor_offset_steps": 0,
                "server_timestamp_ms": None,
                "server_dt_ms": None,
                "server_sample_ms": 0,
                "timestamps_ms": [0],
                "steps": [target],
            }
        ]
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
    ``PoseWaypointsRequest`` whose waypoints carry the chunk's poses and
    timestamps laid out exactly as the waypoint API expects: absolute,
    non-negative, strictly increasing, and spaced by ``dt_ms``.

    This drives the real session end-to-end (only the API gateway, estop and
    Rerun are stubbed), so the timestamps asserted here are the actual values
    the server would receive.
    """
    server = FakeJoggingServer()

    async def state_stream(**_kw: object) -> AsyncGenerator[object, None]:
        # No jogger_session_timestamp_ms (ts_ms=0 is ignored), so the clock's
        # speed_ratio stays 1.0 and server-time == client-time: dt is unscaled.
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
    gateway.jogging_api.execute_waypoint_jogging = server.execute_waypoint_jogging

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
            jogger.set_target(chunk, dt_ms=dt_ms)
            await _wait_until(
                lambda: any(
                    isinstance(_inner(r), api.models.PoseWaypointsRequest) for r in server.requests
                )
            )

        pose_req = next(
            _inner(r)
            for r in server.requests
            if isinstance(_inner(r), api.models.PoseWaypointsRequest)
        )
        waypoints = pose_req.waypoints

        # Every chunk step became one pose waypoint, in order.
        assert len(waypoints) == len(chunk)
        for sent, expected in zip(waypoints, chunk, strict=True):
            assert [sent.pose.position.root[k] for k in range(3)] == expected[:3]
            assert [sent.pose.orientation.root[k] for k in range(3)] == expected[3:6]

        # Timestamps: the layout the waypoint API expects.
        timestamps = [w.timestamp for w in waypoints]
        assert timestamps[0] >= 0  # absolute anchor on the session timeline
        assert timestamps == sorted(timestamps)  # monotonically increasing
        assert all(
            timestamps[i + 1] - timestamps[i] == int(dt_ms) for i in range(len(timestamps) - 1)
        )  # evenly spaced by dt_ms (speed_ratio == 1)
    finally:
        server.stop()
        for p in patches:
            p.stop()


@pytest.mark.asyncio
async def test_overlapping_tcp_chunks_share_one_absolute_timeline():
    """Consecutive overlapping TCP chunks place identical poses at identical
    absolute timestamps -- the property that lets the server stitch a per-tick
    resend stream into one trajectory with no seam jump.

    The chunked path anchors every chunk at an *absolute* session timestamp
    (not a fresh ``now + dt`` each tick). When the next tick advances the anchor
    by exactly one ``dt`` and the caller shifts the chunk content by one step,
    the overlapping region must coincide in both timestamp and pose. If chunks
    were instead re-sequenced from "now" every tick, the overlap would land on
    different timestamps and the server would keep restarting the trajectory.
    """
    session, server = _build_session(mode="cartesian")

    def _pose_requests() -> list[object]:
        return [
            r
            for r in (_inner(x) for x in server.requests)
            if isinstance(r, api.models.PoseWaypointsRequest)
        ]

    try:
        await session.start()
        await session.wait_ready()

        dt = 50.0
        # Tick 1: anchor at 100ms, poses advancing along +x.
        poses_a = [[float(x), 0.0, 300.0, 0.0, 0.0, 0.0] for x in (10, 20, 30, 40, 50)]
        session.update_chunk(steps=poses_a, dt_ms=dt, anchor_ms=100)
        await _wait_until(lambda: len(_pose_requests()) >= 1)

        # Tick 2, one dt later: anchor advances by dt, content shifts by one step
        # -> the two chunks overlap by four waypoints.
        poses_b = [[float(x), 0.0, 300.0, 0.0, 0.0, 0.0] for x in (20, 30, 40, 50, 60)]
        session.update_chunk(steps=poses_b, dt_ms=dt, anchor_ms=150)
        await _wait_until(lambda: len(_pose_requests()) >= 2)

        first, second = _pose_requests()[0], _pose_requests()[1]
        a_by_ts = {
            w.timestamp: tuple(w.pose.position.root[k] for k in range(3)) for w in first.waypoints
        }
        b_by_ts = {
            w.timestamp: tuple(w.pose.position.root[k] for k in range(3)) for w in second.waypoints
        }

        # The two chunks share four absolute timestamps...
        overlap = set(a_by_ts) & set(b_by_ts)
        assert overlap == {150, 200, 250, 300}
        # ...and at every shared timestamp they command the *same* pose, so there
        # is no discontinuity where one chunk hands off to the next.
        for ts in overlap:
            assert a_by_ts[ts] == b_by_ts[ts]
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()


@pytest.mark.asyncio
async def test_overlapping_joint_chunks_share_one_absolute_timeline():
    """The joint-mode counterpart: overlapping joint chunks also place identical
    joint targets at identical absolute timestamps.

    Same per-tick resend as the TCP case, but the joint path emits
    ``JointWaypointsRequest`` messages. The absolute-timeline guarantee is
    mode-independent, so the overlap must coincide here too.
    """
    session, server = _build_session(mode="joint")

    def _joint_requests() -> list[object]:
        return [
            r
            for r in (_inner(x) for x in server.requests)
            if isinstance(r, api.models.JointWaypointsRequest)
        ]

    try:
        await session.start()
        await session.wait_ready()

        dt = 50.0
        # Tick 1: anchor at 100ms, j0 advancing.
        joints_a = [[v, 0.0, 0.0, 0.0, 0.0, 0.0] for v in (0.1, 0.2, 0.3, 0.4, 0.5)]
        session.update_chunk(steps=joints_a, dt_ms=dt, anchor_ms=100)
        await _wait_until(lambda: len(_joint_requests()) >= 1)

        # Tick 2, one dt later: anchor +dt, content shifts one step -> 4 overlap.
        joints_b = [[v, 0.0, 0.0, 0.0, 0.0, 0.0] for v in (0.2, 0.3, 0.4, 0.5, 0.6)]
        session.update_chunk(steps=joints_b, dt_ms=dt, anchor_ms=150)
        await _wait_until(lambda: len(_joint_requests()) >= 2)

        first, second = _joint_requests()[0], _joint_requests()[1]
        a_by_ts = {w.timestamp: tuple(w.joints.root) for w in first.waypoints}
        b_by_ts = {w.timestamp: tuple(w.joints.root) for w in second.waypoints}

        overlap = set(a_by_ts) & set(b_by_ts)
        assert overlap == {150, 200, 250, 300}
        for ts in overlap:
            assert a_by_ts[ts] == b_by_ts[ts]
    finally:
        server.stop()
        await session.stop()
        for p in session._test_patches:  # type: ignore[attr-defined]
            p.stop()
