"""Cursor behaviour against both execute-state publishing modes of the controller.

Two wire behaviours exist for ``MotionGroupState.execute`` (see
nova/cell/movement_controller/README.md, "How the controller publishes
execution state"):

- **Current controllers** drop the ``execute`` block the instant the robot
  settles (robotics/wbr ``MotionPointGenerator`` removes the provider on
  END_OF_TRAJECTORY/USER_PAUSED). The terminal ``TrajectoryEnded`` frame can be
  followed only by *bare standstill* frames.
- **Level-based publishing** (robotics/wbr!2262): ``execute`` persists from
  ``InitializeMovementRequest`` until stop/teardown. ``END_OF_TRAJECTORY`` is
  re-published every step after the end, and ``PAUSED_BY_USER`` is published
  persistently — including **between initialize and the actual motion start**.

These tests pin the cursor's completion logic against synthetic streams of both
shapes: no hang when the execute block vanishes at settle, no premature
"paused" completion from the pre-start PAUSED_BY_USER window, recovery from the
stale pause frame and standstill jitter at motion start, and a clear failure
when the controller contradicts the tracked execution.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from typing import AsyncIterator

import pytest

from nova import api
from nova.cell.movement_controller.trajectory_cursor import (
    OperationType,
    TrajectoryCursor,
    _frame_shows_motion,
)
from nova.exceptions import UnexpectedTrajectoryState

pytestmark = pytest.mark.asyncio

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_NUM_SAMPLES = 4  # locations 0.0 .. 3.0


def _joint_trajectory() -> api.models.JointTrajectory:
    return api.models.JointTrajectory(
        joint_positions=[[0.0] * 6] * _NUM_SAMPLES,
        times=[float(i) for i in range(_NUM_SAMPLES)],
        locations=[float(i) for i in range(_NUM_SAMPLES)],
    )


def _state(
    standstill: bool, execute: api.models.Execute | None = None, sequence_number: int = 1
) -> api.models.MotionGroupState:
    return api.models.MotionGroupState(
        timestamp=datetime.now(timezone.utc),
        sequence_number=sequence_number,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=[0.0] * 6,
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(limit_reached=[False] * 6),
        standstill=standstill,
        execute=execute,
        description_revision=1,
    )


def _execute(
    trajectory_state: (
        api.models.TrajectoryRunning
        | api.models.TrajectoryEnded
        | api.models.TrajectoryPausedByUser
    ),
    location: float,
) -> api.models.Execute:
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-1", location=location, state=trajectory_state
        ),
    )


def _running(location: float) -> api.models.Execute:
    return _execute(api.models.TrajectoryRunning(time_to_end=1000), location)


def _ended(location: float) -> api.models.Execute:
    return _execute(api.models.TrajectoryEnded(), location)


def _paused(location: float) -> api.models.Execute:
    return _execute(api.models.TrajectoryPausedByUser(), location)


async def _stream_then_block(
    states: list[api.models.MotionGroupState],
) -> AsyncIterator[api.models.MotionGroupState]:
    """Yield the given frames, then stay open forever.

    An endless tail is essential: with a finite stream, stream exhaustion tears
    the cursor down and masks a completion-detection hang. Every test here must
    conclude through the completion logic itself, under a timeout.
    """
    for state in states:
        yield state
    await asyncio.Future()


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
    yield api.models.StartMovementResponse()
    await asyncio.Future()


def _cursor(
    stream: AsyncIterator[api.models.MotionGroupState], *, detach_on_standstill: bool
) -> TrajectoryCursor:
    return TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=stream,
        joint_trajectory=_joint_trajectory(),
        initial_location=0.0,
        detach_on_standstill=detach_on_standstill,
        emit_motion_events=False,
    )


def _one_shot_cursor(
    states: list[api.models.MotionGroupState], *, detach_on_standstill: bool = True
) -> TrajectoryCursor:
    """A cursor configured the way ``move_forward`` configures it (one-shot).

    ``detach_on_standstill=False`` gives the interactive (session) shape instead.
    """
    return _cursor(_stream_then_block(states), detach_on_standstill=detach_on_standstill)


async def _drive(cursor: TrajectoryCursor) -> asyncio.Task:
    """Consume the cursor's protocol generator in the background."""

    async def consume():
        async for _request in cursor.cntrl(_responses()):
            pass

    return asyncio.create_task(consume())


async def _settle(rounds: int = 20) -> None:
    for _ in range(rounds):
        await asyncio.sleep(0)


# ---------------------------------------------------------------------------
# Current controllers: execute block vanishes at settle
# ---------------------------------------------------------------------------


async def test_edge_then_bare_standstill_completes_and_detaches():
    """The measured 50 ms-stream shape: TrajectoryEnded seen while decelerating,
    then only bare standstill frames. Previously this hung forever in `ending`
    (the machine discarded frames without an execute block)."""
    cursor = _one_shot_cursor(
        [
            _state(False, _running(0.5)),
            _state(False, _running(2.5)),
            _state(False, _ended(3.0)),  # edge caught while still decelerating
            _state(True),  # execute block already dropped by the controller
            _state(True),
        ]
    )
    operation = cursor.forward()
    consumer = await _drive(cursor)

    async with asyncio.timeout(5):
        result = await operation
        await consumer  # one-shot detach on ended → the protocol loop finishes

    assert result.error is None
    assert result.final_location == 3.0


async def test_pause_edge_then_bare_standstill_completes_the_operation_as_paused():
    """The pause twin of the vanishing terminal state (pinned server-side by
    robotics/wbr!2322): on current controllers the paused trajectory is visible
    at standstill for a single control cycle before the execute block drops.
    A pause observed while decelerating, followed only by bare standstill
    frames, must still conclude the running operation — previously the machine
    hung in `pausing` forever. Interactive cursor: a pause nobody requested
    fails a one-shot execution (see below)."""
    cursor = _one_shot_cursor(
        [
            _state(False, _running(0.5)),
            _state(False, _running(1.0)),
            # pause takes effect while still decelerating…
            _state(False, _paused(1.2)),
            # …and the execute block is gone by the time standstill is reached
            _state(True),
            _state(True),
        ],
        detach_on_standstill=False,
    )
    operation = cursor.forward()
    consumer = await _drive(cursor)

    try:
        async with asyncio.timeout(5):
            result = await operation
        assert result.error is None
        assert result.final_location == 1.2
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await asyncio.gather(consumer, return_exceptions=True)


# ---------------------------------------------------------------------------
# Level-based publishing (robotics/wbr!2262)
# ---------------------------------------------------------------------------


async def test_pre_start_paused_frames_do_not_complete_a_forward_operation():
    """With level-based execute state the controller publishes
    PAUSED_BY_USER + standstill from initialize until the motion actually
    starts. Those frames must not resolve the forward operation — it completes
    at the trajectory end, not at location 0."""
    cursor = _one_shot_cursor(
        [
            # parked between initialize and motion start
            _state(True, _paused(0.0)),
            _state(True, _paused(0.0)),
            _state(True, _paused(0.0)),
            # the motion runs
            _state(False, _running(1.0)),
            _state(False, _running(2.0)),
            # level-based completion: END re-published every step
            _state(True, _ended(3.0)),
            _state(True, _ended(3.0)),
            _state(True, _ended(3.0)),
        ]
    )
    operation = cursor.forward()
    consumer = await _drive(cursor)

    async with asyncio.timeout(5):
        result = await operation
        await consumer

    assert result.error is None
    assert result.final_location == 3.0, (
        "the forward operation must complete at the trajectory end, not be "
        "resolved by the pre-start PAUSED_BY_USER frames"
    )


async def test_pause_after_motion_still_completes_the_operation_as_paused():
    """Guarding the pre-start window must not break real pauses: once the
    operation was seen running, a persistent paused state concludes it.
    Interactive cursor — an external pause is resumable there."""
    cursor = _one_shot_cursor(
        [
            _state(False, _running(0.5)),
            _state(False, _running(1.0)),
            # pause requested elsewhere: level-based publishing repeats it every step
            _state(True, _paused(1.2)),
            _state(True, _paused(1.2)),
        ],
        detach_on_standstill=False,
    )
    operation = cursor.forward()
    consumer = await _drive(cursor)

    try:
        async with asyncio.timeout(5):
            result = await operation
        assert result.error is None
        assert result.paused_on_io is False
        assert result.final_location == 1.2
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await asyncio.gather(consumer, return_exceptions=True)


# ---------------------------------------------------------------------------
# Motion start artefacts (2026-09-16 capture, trajectory 21a04c31)
# ---------------------------------------------------------------------------


def _start_lag_frames() -> list[api.models.MotionGroupState]:
    """F007–F055 of the capture, scaled to this 3-location trajectory: parked frames,
    `standstill` dropping one cycle before the discriminator, RUNNING with a
    standstill flicker while the location ramps up, then the motion to the end."""
    return (
        [_state(True, _paused(0.0))] * 22
        + [_state(False, _paused(0.0))] * 2
        + [_state(False, _running(loc)) for loc in (0.0, 0.0001, 0.0005, 0.0014, 0.0027, 0.0054)]
        + [_state(True, _running(loc)) for loc in (0.0065, 0.0091, 0.012, 0.0155, 0.0193, 0.0235)]
        + [_state(False, _running(loc)) for loc in (0.0258, 1.0, 2.0, 2.9)]
        + [_state(False, _ended(3.0))]
        + [_state(True, _ended(3.0))] * 2
    )


async def test_stale_pause_and_standstill_jitter_at_start_complete_at_the_end():
    """The hang: the machine went `pausing` on the stale PAUSED_BY_USER frame and
    `paused` on the standstill flicker, the FORWARD operation resolved at
    location 0.0065 and nothing ever detached the cursor."""
    cursor = _one_shot_cursor(_start_lag_frames())
    operation = cursor.forward()
    consumer = await _drive(cursor)

    async with asyncio.timeout(5):
        result = await operation
        await consumer

    assert result.error is None
    assert result.paused_on_io is False
    assert result.final_location == 3.0


# ---------------------------------------------------------------------------
# Contradicting frames fail the execution instead of stranding it
# ---------------------------------------------------------------------------


async def test_running_after_ended_without_a_start_fails_the_execution():
    """A RUNNING frame while the machine is `ended` and no operation asked for a
    start: the robot moves without this cursor. The protocol loop raises."""
    cursor = _one_shot_cursor(
        [
            _state(False, _running(1.0)),
            _state(True, _ended(3.0)),
            _state(True, _ended(3.0)),
            _state(False, _running(2.5)),
        ],
        detach_on_standstill=False,
    )
    operation = cursor.forward()
    consumer = await _drive(cursor)

    async with asyncio.timeout(5):
        result = await operation
        assert result.final_location == 3.0
        with pytest.raises(UnexpectedTrajectoryState) as info:
            await consumer

    assert info.value.machine_state == "ended"
    assert info.value.frame is not None
    assert "TrajectoryRunning" in str(info.value)


async def test_external_user_pause_during_one_shot_execution_fails():
    """One-shot execution has no resume path; a PAUSED_BY_USER that this cursor did
    not request would leave execute() waiting forever. It fails instead."""
    cursor = _one_shot_cursor(
        [
            _state(False, _running(0.5)),
            _state(False, _running(1.0)),
            _state(True, _paused(1.2)),
            _state(True, _paused(1.2)),
        ]
    )
    operation = cursor.forward()
    consumer = await _drive(cursor)

    async with asyncio.timeout(5):
        with pytest.raises(UnexpectedTrajectoryState):
            await operation
        with pytest.raises(UnexpectedTrajectoryState) as info:
            await consumer

    assert "one-shot" in str(info.value)
    assert "1.2" in str(info.value)


async def test_pause_requested_by_the_cursor_completes_in_one_shot_mode():
    """The tuner drives a one-shot cursor and pauses through the cursor itself; a
    pause the cursor requested is not a contradiction."""
    frames = _Frames()
    frames.feed(_state(True, _paused(0.0)), _state(False, _running(0.5)))
    cursor = _cursor(frames.stream(), detach_on_standstill=True)
    cursor.forward()
    consumer = await _drive(cursor)
    try:
        await _settle()
        pause = cursor.pause()
        assert pause is not None
        frames.feed(_state(True, _paused(0.8)), _state(True, _paused(0.8)))

        async with asyncio.timeout(5):
            result = await pause

        assert result.error is None
        assert result.operation_type is OperationType.PAUSE
        assert result.final_location == 0.8
        await _settle()
        assert not consumer.done()
    finally:
        cursor.detach()
        async with asyncio.timeout(5):
            await asyncio.gather(consumer, return_exceptions=True)


# ---------------------------------------------------------------------------
# Unit level: motion evidence
# ---------------------------------------------------------------------------


async def test_frame_shows_motion():
    assert _frame_shows_motion(_state(False))
    assert _frame_shows_motion(_state(False, _running(1.0)))
    # RUNNING detail counts as motion evidence even with standstill=True
    assert _frame_shows_motion(_state(True, _running(1.0)))
    # mere presence of an execute block is NOT motion evidence (wbr!2262
    # publishes it persistently from initialize on)
    assert not _frame_shows_motion(_state(True, _paused(0.0)))
    assert not _frame_shows_motion(_state(True, _ended(3.0)))
    assert not _frame_shows_motion(_state(True))
