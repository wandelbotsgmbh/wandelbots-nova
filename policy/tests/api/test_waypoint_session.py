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
from policy.jogging.waypoint_session import WaypointJoggingSession
from policy.types import MotionError, WaypointConfig

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator, Callable, Sequence

_SESSION = "policy.jogging.waypoint_session"


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

    def __init__(self, *, fault: object | None = None) -> None:
        self.requests: list[object] = []
        self._fault = fault
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


def _stream_state(state: object, *, ts_ms: int = 0) -> object:
    """One MotionGroupState as yielded by ``stream_state``."""
    details = SimpleNamespace(jogger_session_timestamp_ms=ts_ms, state=None)
    return SimpleNamespace(
        joint_position=list(state.joints),
        tcp_pose=None,  # keep the initial pose; avoids constructing a Pose here
        tcp="Flange",
        execute=SimpleNamespace(details=details),
    )


def _build_session(
    *,
    fault: object | None = None,
    states: Sequence[object] = (),
    stop_conditions: list[object] | None = None,
) -> tuple[WaypointJoggingSession, FakeJoggingServer]:
    server = FakeJoggingServer(fault=fault)

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
        mode="joint",
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
    session, server = _build_session()
    try:
        await session.start()
        await session.wait_ready()
        target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        session.update_chunk(steps=[target], dt_ms=50.0, first_timestamp_ms=0)

        await _wait_until(lambda: len(server.requests) >= 2)
        waypoint_req = _inner(server.requests[1])
        assert isinstance(waypoint_req, api.models.JointWaypointsRequest)
        assert len(waypoint_req.waypoints) == 1
        sent = waypoint_req.waypoints[0]
        assert list(sent.joints.root) == target
        assert sent.timestamp == 0  # absolute anchor at 0, single step
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
