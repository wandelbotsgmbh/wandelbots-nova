"""``MotionGroup.execute()`` through a controller-side IO pause.

End to end over the test doubles: the fake controller acks the start, reports RUNNING,
then holds a ``PAUSED_ON_IO`` (level-based, never END_OF_TRAJECTORY) until a second start
arrives; the fake bus IO reports the signal set, then cleared. ``execute()`` must block
through the pause and return only after the trajectory ended, having sent exactly two
starts that both carry the pause condition.
"""

import asyncio
from collections.abc import AsyncIterator
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.cell.motion_group import MotionGroup
from nova.cell.state_stream import MotionGroupStateStreamRegistry
from nova.core.gateway import ApiGateway
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
    # The pause signal reads as set twice, then cleared.
    gateway.bus_ios_api = MagicMock()
    gateway.bus_ios_api.get_bus_io_values = AsyncMock(
        side_effect=[
            [api.models.IOBooleanValue(io="hold", value=True)],
            [api.models.IOBooleanValue(io="hold", value=True)],
            [api.models.IOBooleanValue(io="hold", value=False)],
        ]
    )
    motion_group = MotionGroup(
        api_client=gateway, cell="cell", controller_id="ctrl", motion_group_id="0@ctrl"
    )
    motion_group._load_planned_motion = AsyncMock(return_value="traj-1")

    async with asyncio.timeout(10):
        await motion_group.execute(
            _joint_trajectory(), tcp=None, actions=[], pause_on_io=_pause_condition()
        )

    assert len(fake.starts) == 2, "one start, then one resume after the signal cleared"
    assert all(s.pause_on_io == _pause_condition() for s in fake.starts)
    assert gateway.bus_ios_api.get_bus_io_values.await_count == 3
    await asyncio.wait_for(upstream.closed.wait(), 1.0)
