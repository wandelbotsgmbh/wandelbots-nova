"""``TrajectoryExecutor.execute`` with one ``pause_on_io`` for the whole ensemble.

A multi-group motion pauses as a whole on one signal: every group's start carries the
same condition, and once it clears ``execute()`` starts all groups again through the
barrier, so they set off together from where they stopped. With
``resume_on_io_release=False`` the condition is a stop condition and ``execute()``
returns where the groups stopped.
"""

import asyncio
import json
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from nova import api
from nova.cell.io_condition import IOConditionUnavailable, motion_enable_signal
from nova.cell.trajectory_executor import TrajectoryExecutor
from tests.cell.fake_nats import FakeNats
from tests.cell.multi_group_doubles import (
    ended_state,
    execute_detail,
    motion_group,
    multi_trajectory,
    running_state,
    state,
    sync_driver,
)
from tests.cell.test_trajectory_executor_session import _FakeGateway

pytestmark = pytest.mark.asyncio

VALUES = "nova.v2.cells.cell.bus-ios.ios"


def _wait_for_io(location: float, at_milliseconds: int) -> api.models.MotionGroupState:
    return state(True, execute_detail(location, api.models.TrajectoryWaitForIO()), at_milliseconds)


def _paused_on_io(location: float, at_milliseconds: int) -> api.models.MotionGroupState:
    return state(True, execute_detail(location, api.models.TrajectoryPausedOnIO()), at_milliseconds)


class _Signal:
    """The bus IO ``enable``: read over REST once per wait, changes pushed over NATS."""

    def __init__(self, gateway: _FakeGateway, nats: FakeNats, enabled: bool):
        self.enabled = enabled
        self._nats = nats
        gateway.bus_ios_api = SimpleNamespace(
            get_bus_io_values=AsyncMock(
                side_effect=lambda **_: [api.models.IOBooleanValue(io="enable", value=self.enabled)]
            ),
            get_bus_io_state=AsyncMock(
                return_value=SimpleNamespace(
                    state=api.models.BusIOsStateEnum.BUS_IOS_STATE_CONNECTED
                )
            ),
        )

    async def set(self, enabled: bool) -> None:
        self.enabled = enabled
        payload = [{"io": "enable", "value": enabled, "value_type": "boolean"}]
        await self._nats.publish(VALUES, json.dumps(payload).encode())


def _executor(gateway: _FakeGateway, nats: FakeNats, controllers=("ctrl", "ctrl")):
    queues = {"a": asyncio.Queue(), "b": asyncio.Queue()}
    groups = {}
    for (name, queue), controller in zip(queues.items(), controllers):
        group = motion_group(gateway, queue, controller=controller, trajectory_id=f"traj-{name}")
        group._nats_client = nats
        groups[name] = group
    return TrajectoryExecutor(groups, sync=sync_driver(gateway)), queues


async def _run_to_the_pause(gateway: _FakeGateway, queues: dict[str, asyncio.Queue]) -> None:
    await gateway.reached("start", 2)
    for queue in queues.values():
        queue.put_nowait(_wait_for_io(0.0, at_milliseconds=10))
    await gateway.reached("write", 2)
    for queue in queues.values():
        queue.put_nowait(running_state(0.5, at_milliseconds=20))
        queue.put_nowait(_paused_on_io(1.0, at_milliseconds=30))


async def _wait_until_subscribed(nats: FakeNats) -> None:
    async with asyncio.timeout(5):
        while VALUES not in nats.subjects():
            await asyncio.sleep(0)


async def test_execute_resumes_every_group_through_the_barrier_once_the_signal_clears():
    gateway = _FakeGateway()
    nats = FakeNats()
    signal = _Signal(gateway, nats, enabled=True)
    executor, queues = _executor(gateway, nats)
    condition = motion_enable_signal("enable")

    execute = asyncio.create_task(
        executor.execute(multi_trajectory("a", "b"), pause_on_io=condition)
    )
    signal.enabled = False  # the PLC drops the enable while the groups move
    await _run_to_the_pause(gateway, queues)
    await _wait_until_subscribed(nats)
    assert not execute.done(), "execute() must wait while the ensemble is paused"
    assert gateway._counts["start"] == 2

    await signal.set(True)
    await gateway.reached("start", 4)
    for queue in queues.values():
        queue.put_nowait(_wait_for_io(1.0, at_milliseconds=40))
    await gateway.reached("write", 4)
    for queue in queues.values():
        queue.put_nowait(running_state(1.5, at_milliseconds=50))
        queue.put_nowait(ended_state(2.0, at_milliseconds=60))
    await asyncio.wait_for(execute, timeout=5)

    for trajectory in ("traj-a", "traj-b"):
        starts = gateway.start_requests[trajectory]
        assert len(starts) == 2
        assert all(start.pause_on_io == condition for start in starts)
        assert all(start.start_on_io is not None for start in starts), "both through the barrier"
    assert gateway.trigger_writes == [False, True, False, True]


async def test_execute_without_resume_returns_where_the_groups_stopped():
    gateway = _FakeGateway()
    nats = FakeNats()
    _Signal(gateway, nats, enabled=False)
    executor, queues = _executor(gateway, nats)

    execute = asyncio.create_task(
        executor.execute(
            multi_trajectory("a", "b"),
            pause_on_io=motion_enable_signal("enable"),
            resume_on_io_release=False,
        )
    )
    await _run_to_the_pause(gateway, queues)
    await asyncio.wait_for(execute, timeout=5)

    assert gateway._counts["start"] == 2, "a stop condition is not resumed"
    gateway.bus_ios_api.get_bus_io_values.assert_not_awaited()


async def test_a_controller_io_condition_cannot_pause_groups_on_different_controllers():
    gateway = _FakeGateway()
    executor, _ = _executor(gateway, FakeNats(), controllers=("ctrl-a", "ctrl-b"))

    with pytest.raises(ValueError, match="one signal"):
        await executor.execute(
            multi_trajectory("a", "b"),
            pause_on_io=motion_enable_signal("enable", api.models.IOOrigin.CONTROLLER),
        )
    assert gateway._counts["init"] == 0


async def test_a_bus_io_condition_without_nats_fails_before_anything_moves():
    gateway = _FakeGateway()
    executor, _ = _executor(gateway, FakeNats(connected=False))

    with pytest.raises(IOConditionUnavailable):
        await executor.execute(
            multi_trajectory("a", "b"), pause_on_io=motion_enable_signal("enable")
        )
    assert gateway._counts["init"] == 0
