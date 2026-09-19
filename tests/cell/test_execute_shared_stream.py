"""``MotionGroup._execute`` over the shared state stream.

Asserts the structural properties that remove the end-of-motion stall: one
state websocket per execution regardless of how many consumers it has (relay
and cursor), teardown by deregistration instead of cancellation, and the
websocket closed by the shared stream's pump once the execution is over.

The ``executeTrajectory`` protocol fake mirrors the one proven in
``test_trajectory_executor_session.py``; states flow through a real
``MotionGroupStateStreamRegistry`` over a :class:`FakeUpstream`.
"""

import asyncio
from collections.abc import AsyncIterator
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.cell.motion_group import MotionGroup
from nova.cell.state_stream import (
    MotionGroupStateStreamRegistry,
    SharedMotionGroupStateStream,
    StateSubscription,
)
from nova.core.gateway import ApiGateway
from tests.cell.multi_group_doubles import ended_state, running_state, state
from tests.cell.test_state_stream import FakeUpstream

pytestmark = pytest.mark.asyncio


def _joint_trajectory() -> api.models.JointTrajectory:
    return api.models.JointTrajectory(
        joint_positions=[[0.0] * 6] * 3, times=[0.0, 1.0, 2.0], locations=[0.0, 1.0, 2.0]
    )


def _with_tcp(motion_group_state: api.models.MotionGroupState) -> api.models.MotionGroupState:
    """The relay converts execute frames to MotionStates, which needs a TCP."""
    return motion_group_state.model_copy(
        update={
            "tcp": "Flange",
            "tcp_pose": api.models.Pose(position=[0.0] * 3, orientation=[0.0] * 3),
        }
    )


class _ExecuteProtocolFake:
    """The bidirectional ``executeTrajectory`` protocol with FIFO acks.

    Feeds trajectory states into ``upstream`` as the protocol advances, the way
    a controller starts reporting once it is commanded.
    """

    def __init__(self, upstream: FakeUpstream, fail_after_start: Exception | None = None):
        self._upstream = upstream
        self._fail_after_start = fail_after_start

    async def __call__(self, cell, controller, client_request_generator):
        responses: asyncio.Queue = asyncio.Queue()

        async def response_stream() -> AsyncIterator[api.models.ExecuteTrajectoryResponse]:
            while True:
                yield await responses.get()

        # A real motion-group stream is always live; an idle state satisfies
        # the cursor's startup handshake before any trajectory progress.
        self._upstream.feed(_with_tcp(state(True)))
        async for request in client_request_generator(response_stream()):
            if isinstance(request, api.models.InitializeMovementRequest):
                responses.put_nowait(api.models.InitializeMovementResponse())
            elif isinstance(request, api.models.StartMovementRequest):
                responses.put_nowait(api.models.StartMovementResponse())
                self._upstream.feed(_with_tcp(running_state(0.5)))
                if self._fail_after_start is not None:
                    self._upstream.fail(self._fail_after_start)
                else:
                    self._upstream.feed(_with_tcp(ended_state(2.0)))


def _motion_group(upstream: FakeUpstream, fail_after_start: Exception | None = None) -> MotionGroup:
    gateway = MagicMock(spec=ApiGateway)
    registry = MotionGroupStateStreamRegistry(
        open_stream=lambda cell, controller_id, motion_group_id, rate: upstream.open(rate)
    )
    gateway.motion_group_state_stream = registry.stream
    gateway.trajectory_execution_api = MagicMock()
    gateway.trajectory_execution_api.execute_trajectory = _ExecuteProtocolFake(
        upstream, fail_after_start
    )
    motion_group = MotionGroup(
        api_client=gateway, cell="cell", controller_id="ctrl", motion_group_id="0@ctrl"
    )
    motion_group._load_planned_motion = AsyncMock(return_value="traj-1")
    return motion_group


def _flatten(error: BaseException) -> list[BaseException]:
    if isinstance(error, BaseExceptionGroup):
        return [leaf for inner in error.exceptions for leaf in _flatten(inner)]
    return [error]


async def test_execute_uses_exactly_one_state_socket(monkeypatch):
    upstream = FakeUpstream()
    motion_group = _motion_group(upstream)

    cancelled_anexts: list[BaseException] = []
    original_anext = StateSubscription.__anext__

    async def recording_anext(self):
        try:
            return await original_anext(self)
        except asyncio.CancelledError as error:
            cancelled_anexts.append(error)
            raise

    monkeypatch.setattr("nova.cell.state_stream.StateSubscription.__anext__", recording_anext)

    motion_states = []
    async with asyncio.timeout(5):
        async for motion_state in motion_group.stream_execute(
            _joint_trajectory(), tcp=None, actions=[]
        ):
            motion_states.append(motion_state)

    assert upstream.open_rates == [None], "relay and cursor must share one websocket"
    assert len(motion_states) == 2, "the running and the ended frame are relayed"
    # The idle handshake frame has no execute block and is filtered out.

    # Teardown is deregistration, never cancellation into a websocket close.
    assert cancelled_anexts == []
    await asyncio.wait_for(upstream.closed.wait(), 1.0)
    assert upstream.aclose_count == 1, "the pump closes the websocket once, when refcount hits 0"


async def test_execute_surfaces_a_state_stream_failure_mid_operation():
    upstream = FakeUpstream()
    motion_group = _motion_group(upstream, fail_after_start=RuntimeError("state stream died"))

    with pytest.raises(BaseException) as excinfo:
        async with asyncio.timeout(5):
            async for _ in motion_group.stream_execute(_joint_trajectory(), tcp=None, actions=[]):
                pass

    leaves = _flatten(excinfo.value)
    assert any(
        isinstance(error, RuntimeError) and "state stream died" in str(error) for error in leaves
    ), f"expected the upstream error to surface, got {leaves!r}"

    # The failed socket is closed and a later execution would get a fresh one.
    await asyncio.wait_for(upstream.closed.wait(), 1.0)


async def test_cursor_subscription_is_released_when_the_monitor_ends():
    """The cursor monitor acloses its shared-stream subscription in its finally.

    Covered end-to-end by test_execute_uses_exactly_one_state_socket (the
    refcount only reaches 0 because both subscriptions deregister); this pins
    the seam directly: a cursor fed by a subscription releases it on detach.
    """
    upstream = FakeUpstream()
    shared = SharedMotionGroupStateStream(open_stream=upstream.open, name="cell/ctrl/0@ctrl")

    from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor

    cursor = TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=lambda: shared.subscribe(),
        joint_trajectory=_joint_trajectory(),
        detach_on_standstill=True,
        emit_motion_events=False,
    )

    async def responses() -> AsyncIterator[api.models.ExecuteTrajectoryResponse]:
        yield api.models.InitializeMovementResponse()
        yield api.models.StartMovementResponse()

    operation = cursor.forward()
    upstream.feed(state(True))
    upstream.feed(running_state(0.5))
    upstream.feed(ended_state(2.0))

    async with asyncio.timeout(5):
        async for _ in cursor.cntrl(responses()):
            pass
        result = await operation

    assert result.final_location == 2.0
    await asyncio.wait_for(upstream.closed.wait(), 1.0)
    assert upstream.aclose_count == 1
