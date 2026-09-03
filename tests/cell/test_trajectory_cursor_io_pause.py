"""Cursor behaviour on a controller-side IO pause (``pause_on_io`` → ``PAUSED_ON_IO``).

Measured wire behaviour (docs/architecture/incoming/pause-on-signal-evaluation.md): the
controller reports ``PAUSED_ON_IO`` level-based for as long as it holds the pause, never
resumes by itself, and honours a new ``StartMovementRequest`` only once the condition has
cleared. Between init and the first start it reports the parked ``PAUSED_BY_USER``; a
condition that is already true at the start yields ``PAUSED_ON_IO`` without any motion.

These tests pin the cursor's side of that contract: an IO pause completes the movement
operation *as paused* (not as reached target), keeps the cursor attached, and the stale
IO-pause frames the controller still publishes right after a resume do not complete the
resumed operation.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from typing import AsyncIterator

import pytest

from nova import api
from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor

pytestmark = pytest.mark.asyncio


def _joint_trajectory() -> api.models.JointTrajectory:
    return api.models.JointTrajectory(
        joint_positions=[[0.0] * 6] * 4, times=[0.0, 1.0, 2.0, 3.0], locations=[0.0, 1.0, 2.0, 3.0]
    )


def _state(
    standstill: bool, execute: api.models.Execute | None = None
) -> api.models.MotionGroupState:
    return api.models.MotionGroupState(
        timestamp=datetime.now(timezone.utc),
        sequence_number=1,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=[0.0] * 6,
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(limit_reached=[False] * 6),
        standstill=standstill,
        execute=execute,
        description_revision=1,
    )


def _execute(trajectory_state, location: float) -> api.models.Execute:
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-1", location=location, state=trajectory_state
        ),
    )


def running(location: float) -> api.models.MotionGroupState:
    return _state(False, _execute(api.models.TrajectoryRunning(time_to_end=1000), location))


def paused_on_io(location: float, standstill: bool = True) -> api.models.MotionGroupState:
    return _state(standstill, _execute(api.models.TrajectoryPausedOnIO(), location))


def parked(location: float = 0.0) -> api.models.MotionGroupState:
    """The pre-start PAUSED_BY_USER the controller publishes after initialization."""
    return _state(True, _execute(api.models.TrajectoryPausedByUser(), location))


def ended(location: float) -> api.models.MotionGroupState:
    return _state(True, _execute(api.models.TrajectoryEnded(), location))


class _Frames:
    """A never-ending state stream the test feeds frame by frame."""

    def __init__(self):
        self._queue: asyncio.Queue[api.models.MotionGroupState] = asyncio.Queue()

    def feed(self, *states: api.models.MotionGroupState) -> None:
        for state in states:
            self._queue.put_nowait(state)

    async def stream(self) -> AsyncIterator[api.models.MotionGroupState]:
        while True:
            yield await self._queue.get()


async def _responses() -> AsyncIterator[api.models.ExecuteTrajectoryResponse]:
    yield api.models.InitializeMovementResponse()
    while True:
        yield api.models.StartMovementResponse()
        await asyncio.sleep(0)


def _pause_condition() -> api.models.PauseOnIO:
    return api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="hold", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.BUS_IO,
    )


def _one_shot_cursor(frames: _Frames) -> TrajectoryCursor:
    return TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=frames.stream(),
        joint_trajectory=_joint_trajectory(),
        initial_location=0.0,
        detach_on_standstill=True,
        emit_motion_events=False,
        pause_on_io=_pause_condition(),
    )


async def _drive(cursor: TrajectoryCursor) -> tuple[asyncio.Task, list]:
    requests: list = []

    async def consume():
        async for request in cursor.cntrl(_responses()):
            requests.append(request)

    return asyncio.create_task(consume()), requests


async def _settle(rounds: int = 20) -> None:
    for _ in range(rounds):
        await asyncio.sleep(0)


async def test_io_pause_completes_the_movement_as_paused_and_keeps_the_cursor_attached():
    frames = _Frames()
    frames.feed(_state(True), parked())
    cursor = _one_shot_cursor(frames)
    operation = cursor.forward()
    consumer, requests = await _drive(cursor)
    try:
        frames.feed(running(0.5), paused_on_io(1.0, standstill=False), paused_on_io(1.0))

        async with asyncio.timeout(5):
            result = await operation

        assert result.paused_on_io is True
        assert result.error is None
        assert result.final_location == 1.0
        await _settle()
        # `paused`, not `ended`: even the one-shot cursor stays attached so the
        # movement can be resumed with another start.
        assert not consumer.done()
        assert not cursor._stop_event.is_set()
        starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
        assert len(starts) == 1 and starts[0].pause_on_io == _pause_condition()
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await consumer


async def test_io_pause_before_any_motion_completes_the_operation():
    """The condition already holds at the start: the controller reports PAUSED_ON_IO
    at the start location without ever running. Unlike the parked PAUSED_BY_USER,
    this must resolve the movement — nothing else ever will."""
    frames = _Frames()
    frames.feed(_state(True), parked())
    cursor = _one_shot_cursor(frames)
    operation = cursor.forward()
    consumer, _ = await _drive(cursor)
    try:
        frames.feed(parked(), paused_on_io(0.0))

        async with asyncio.timeout(5):
            result = await operation

        assert result.paused_on_io is True
        assert result.final_location == 0.0
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await consumer


async def test_stale_io_pause_frames_after_a_resume_do_not_complete_the_resumed_operation():
    frames = _Frames()
    frames.feed(_state(True), parked())
    cursor = _one_shot_cursor(frames)
    first = cursor.forward()
    consumer, requests = await _drive(cursor)
    try:
        frames.feed(running(0.5), paused_on_io(1.0))
        async with asyncio.timeout(5):
            assert (await first).paused_on_io

        # Resume: the controller keeps re-publishing the pause it was started out
        # of until it has processed the new start.
        resumed = cursor.forward()
        frames.feed(paused_on_io(1.0), paused_on_io(1.0), paused_on_io(1.0))
        await _settle()
        assert not resumed.done(), "stale PAUSED_ON_IO frames completed the resumed operation"

        # ...then it runs, and a later, genuine IO pause does complete it.
        frames.feed(running(1.5), paused_on_io(2.0))
        async with asyncio.timeout(5):
            result = await resumed
        assert result.paused_on_io is True
        assert result.final_location == 2.0

        # A final resume that reaches the end completes normally and detaches
        # the one-shot cursor.
        final = cursor.forward()
        frames.feed(paused_on_io(2.0), running(2.5), ended(3.0), _state(True))
        async with asyncio.timeout(5):
            result = await final
            await consumer
        assert result.paused_on_io is False
        assert result.final_location == 3.0
        starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
        assert len(starts) == 3
        assert all(s.pause_on_io == _pause_condition() for s in starts)
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await consumer


async def test_stale_end_frames_after_an_intermediate_stop_do_not_complete_the_next_move():
    """An intermediate ``forward_to`` target is reported as END_OF_TRAJECTORY and kept
    re-published (level-based) until the controller processes the next start — measured
    on 26.7.0; the multi-group session test showed the next ``forward()`` resolving at
    the intermediate location. The same stale-frame rule as for IO pauses applies."""
    frames = _Frames()
    frames.feed(_state(True), parked())
    # A session cursor (as the multi-group executor builds it): no auto-detach.
    cursor = TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=frames.stream(),
        joint_trajectory=_joint_trajectory(),
        initial_location=0.0,
        detach_on_standstill=False,
        emit_motion_events=False,
    )
    consumer, _ = await _drive(cursor)
    try:
        first = cursor.forward_to(1.0)
        frames.feed(running(0.5), ended(1.0))
        async with asyncio.timeout(5):
            result = await first
        assert result.final_location == 1.0

        second = cursor.forward()
        frames.feed(ended(1.0), ended(1.0), ended(1.0))
        await _settle()
        assert not second.done(), "stale END_OF_TRAJECTORY frames completed the next move"

        frames.feed(running(1.5), ended(3.0), _state(True))
        async with asyncio.timeout(5):
            result = await second
        assert result.final_location == 3.0
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await consumer


async def test_a_start_at_the_real_end_is_still_concluded_by_the_repeated_end_frame():
    """With nowhere left to move, the re-published END is the honest answer, not a
    stale one: the operation must not hang."""
    frames = _Frames()
    frames.feed(_state(True), parked())
    cursor = TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=frames.stream(),
        joint_trajectory=_joint_trajectory(),
        initial_location=0.0,
        detach_on_standstill=False,
        emit_motion_events=False,
    )
    consumer, _ = await _drive(cursor)
    try:
        first = cursor.forward()
        frames.feed(running(1.5), ended(3.0))
        async with asyncio.timeout(5):
            assert (await first).final_location == 3.0

        again = cursor.forward()
        frames.feed(ended(3.0), ended(3.0))
        async with asyncio.timeout(5):
            assert (await again).final_location == 3.0
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await consumer
