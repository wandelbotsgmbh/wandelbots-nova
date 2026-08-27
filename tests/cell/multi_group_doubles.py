"""Doubles shared by the multi-motion-group execution tests.

Nothing here emulates the ``executeTrajectory`` protocol: these are the api
model builders, the motion-group double and the gateway's IO surface. The
protocol fake sits with the tests that need it, in
``test_trajectory_executor_session.py``.
"""

import asyncio
from datetime import datetime, timedelta, timezone
from unittest.mock import AsyncMock, MagicMock

from nova import api
from nova.actions.io import io_write
from nova.cell.multi_trajectory_cursor import IOSyncDriver

BASE_TIME = datetime(2026, 1, 1, tzinfo=timezone.utc)


def multi_trajectory(*keys: str) -> api.models.MultiJointTrajectory:
    return api.models.MultiJointTrajectory(
        joint_positions_by_motion_group_key={key: [[0.0] * 6] * 3 for key in keys},
        times=[0.0, 1.0, 2.0],
        locations=[0.0, 1.0, 2.0],
    )


def state(
    standstill: bool, execute: api.models.Execute | None = None, at_milliseconds: int = 0
) -> api.models.MotionGroupState:
    return api.models.MotionGroupState(
        timestamp=BASE_TIME + timedelta(milliseconds=at_milliseconds),
        sequence_number=1,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=[0.0] * 6,
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(limit_reached=[False] * 6),
        standstill=standstill,
        execute=execute,
        description_revision=1,
    )


def execute_detail(location: float, trajectory_state=None) -> api.models.Execute:
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-1",
            location=location,
            state=trajectory_state or api.models.TrajectoryRunning(time_to_end=0),
        ),
    )


def wait_for_io_state(at_milliseconds: int = 0) -> api.models.MotionGroupState:
    return state(True, execute_detail(0.0, api.models.TrajectoryWaitForIO()), at_milliseconds)


def running_state(location: float, at_milliseconds: int = 0) -> api.models.MotionGroupState:
    return state(False, execute_detail(location), at_milliseconds)


def ended_state(location: float, at_milliseconds: int = 0) -> api.models.MotionGroupState:
    return state(True, execute_detail(location, api.models.TrajectoryEnded()), at_milliseconds)


def paused_state(location: float, at_milliseconds: int = 0) -> api.models.MotionGroupState:
    return state(
        True, execute_detail(location, api.models.TrajectoryPausedByUser()), at_milliseconds
    )


def watch_condition(io: str, value: bool = True) -> api.models.StartOnIO:
    return api.models.StartOnIO(
        io=api.models.IOBooleanValue(io=io, value=value),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )


def sync_driver(api_client, groups: tuple[str, ...] = ("a", "b")) -> IOSyncDriver:
    return IOSyncDriver(
        clear=io_write("sync-io", False, device_id="controller-a"),
        release=io_write("sync-io", True, device_id="controller-a"),
        watch={group: watch_condition("sync-io") for group in groups},
        api_client=api_client,
        cell="cell",
    )


class IOGateway:
    """The gateway's IO surface, recording what the sync driver writes."""

    def __init__(self):
        self.io_writes: list[tuple[str, bool]] = []

        self.controller_ios_api = MagicMock()
        self.controller_ios_api.set_output_values = AsyncMock(side_effect=self._record_write)
        self.controller_ios_api.wait_for_io_event = AsyncMock()

    @property
    def trigger_writes(self) -> list[bool]:
        return [value for _, value in self.io_writes]

    async def _record_write(self, cell, controller, io_value):
        self.io_writes.append((io_value[0].io, io_value[0].value))


def motion_group(
    api_client,
    states: asyncio.Queue | None = None,
    controller: str = "controller-a",
    trajectory_id: str = "traj-1",
    cell: str = "cell",
) -> MagicMock:
    """A motion group as the executor sees it: an address, a loader and a stream."""
    queue = states if states is not None else asyncio.Queue()

    async def stream():
        # A real motion-group stream is always live; an idle state satisfies the
        # cursor's startup handshake before any trajectory progress arrives.
        yield state(True)
        while True:
            yield await queue.get()

    group = MagicMock()
    group._cell = cell
    group._controller_id = controller
    group._api_client = api_client
    group._load_planned_motion = AsyncMock(return_value=trajectory_id)
    group.stream_state = stream
    return group
