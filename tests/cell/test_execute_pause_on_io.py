"""``MotionGroup.execute()`` through a controller-side IO pause.

End to end over the test doubles: the fake controller acks the start, reports RUNNING,
then holds a ``PAUSED_ON_IO`` (level-based, never END_OF_TRAJECTORY) until a second start
arrives; the bus IO reads "hold" once, then NATS pushes the release. ``execute()`` must
block through the pause and return only after the trajectory ended, having sent exactly
two starts that both carry the pause condition — without polling the API.
"""

import asyncio
import json
from collections.abc import AsyncIterator
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.cell.motion_group import MotionGroup
from nova.cell.state_stream import MotionGroupStateStreamRegistry
from nova.core.gateway import ApiGateway
from tests.cell.fake_nats import FakeNats
from tests.cell.multi_group_doubles import ended_state, execute_detail, running_state, state
from tests.cell.test_execute_shared_stream import _joint_trajectory, _with_tcp
from tests.cell.test_state_stream import FakeUpstream

pytestmark = pytest.mark.asyncio


def paused_on_io_state(location: float) -> api.models.MotionGroupState:
    return state(True, execute_detail(location, api.models.TrajectoryPausedOnIO()))


class _PausingController:
    """executeTrajectory fake: pauses on IO after the first start, ends after the second."""

    def __init__(self, upstream: FakeUpstream):
        self._upstream = upstream
        self.starts: list[api.models.StartMovementRequest] = []

    async def __call__(self, cell, controller, client_request_generator):
        responses: asyncio.Queue = asyncio.Queue()

        async def response_stream() -> AsyncIterator[api.models.ExecuteTrajectoryResponse]:
            while True:
                yield await responses.get()

        self._upstream.feed(_with_tcp(state(True)))
        async for request in client_request_generator(response_stream()):
            if isinstance(request, api.models.InitializeMovementRequest):
                responses.put_nowait(api.models.InitializeMovementResponse())
            elif isinstance(request, api.models.StartMovementRequest):
                self.starts.append(request)
                responses.put_nowait(api.models.StartMovementResponse())
                if len(self.starts) == 1:
                    self._upstream.feed(_with_tcp(running_state(0.5)))
                    for _ in range(3):
                        self._upstream.feed(_with_tcp(paused_on_io_state(1.0)))
                else:
                    self._upstream.feed(_with_tcp(paused_on_io_state(1.0)))  # stale
                    self._upstream.feed(_with_tcp(running_state(1.5)))
                    self._upstream.feed(_with_tcp(ended_state(2.0)))


def _pause_condition() -> api.models.PauseOnIO:
    return api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="hold", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.BUS_IO,
    )


async def test_execute_blocks_through_an_io_pause_and_completes_after_the_signal_clears():
    upstream = FakeUpstream()
    fake = _PausingController(upstream)
    gateway = MagicMock(spec=ApiGateway)
    registry = MotionGroupStateStreamRegistry(
        open_stream=lambda cell, controller_id, motion_group_id, rate: upstream.open(rate)
    )
    gateway.motion_group_state_stream = registry.stream
    gateway.trajectory_execution_api = MagicMock()
    gateway.trajectory_execution_api.execute_trajectory = fake
    # The one initial read says "hold"; the release arrives as a NATS push.
    gateway.bus_ios_api = MagicMock()
    gateway.bus_ios_api.get_bus_io_values = AsyncMock(
        return_value=[api.models.IOBooleanValue(io="hold", value=True)]
    )
    gateway.bus_ios_api.get_bus_io_state = AsyncMock(
        return_value=api.models.BusIOsState(
            state=api.models.BusIOsStateEnum.BUS_IOS_STATE_CONNECTED
        )
    )
    nats = FakeNats()
    motion_group = MotionGroup(
        api_client=gateway,
        cell="cell",
        controller_id="ctrl",
        motion_group_id="0@ctrl",
        nats_client=nats,
    )
    motion_group._load_planned_motion = AsyncMock(return_value="traj-1")

    async def release_when_waited_for():
        # Wait until the release watcher subscribed to the values subject.
        while "nova.v2.cells.cell.bus-ios.ios" not in nats.subjects():
            await asyncio.sleep(0)
        await asyncio.sleep(0)
        await nats.publish(
            "nova.v2.cells.cell.bus-ios.ios",
            json.dumps([{"io": "hold", "value": False, "value_type": "boolean"}]).encode(),
        )

    async with asyncio.timeout(10):
        async with asyncio.TaskGroup() as tg:
            tg.create_task(release_when_waited_for())
            await motion_group.execute(
                _joint_trajectory(), tcp=None, actions=[], pause_on_io=_pause_condition()
            )

    assert len(fake.starts) == 2, "one start, then one resume after the signal cleared"
    assert all(s.pause_on_io == _pause_condition() for s in fake.starts)
    assert gateway.bus_ios_api.get_bus_io_values.await_count == 1, "one initial read, no polling"
    await asyncio.wait_for(upstream.closed.wait(), 1.0)
