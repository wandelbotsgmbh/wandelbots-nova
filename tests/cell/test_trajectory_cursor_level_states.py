"""Cursor behaviour against both execute-state publishing modes of the controller.

Two wire behaviours exist for ``MotionGroupState.execute`` (see
docs/architecture/incoming/execution-completion-detection-research.md):

- **Current controllers** drop the ``execute`` block the instant the robot
  settles (robotics/wbr ``MotionPointGenerator`` removes the provider on
  END_OF_TRAJECTORY/USER_PAUSED). The terminal ``TrajectoryEnded`` frame can be
  followed only by *bare standstill* frames.
- **Level-based publishing** (robotics/wbr!2262): ``execute`` persists from
  ``InitializeMovementRequest`` until stop/teardown. ``END_OF_TRAJECTORY`` is
  re-published every step after the end, and ``PAUSED_BY_USER`` is published
  persistently — including **between initialize and the actual motion start**.

These tests pin the cursor's completion logic against synthetic streams of both
shapes: no hang when the execute block vanishes at settle, and no premature
"paused" completion from the pre-start PAUSED_BY_USER window.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from typing import AsyncIterator

import pytest

from nova import api
from nova.cell.movement_controller.trajectory_cursor import (
    OperationHandler,
    OperationState,
    OperationType,
    TrajectoryCursor,
    _frame_shows_motion,
)

pytestmark = pytest.mark.asyncio

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_NUM_SAMPLES = 4  # locations 0.0 .. 3.0


def _joint_trajectory() -> api.models.JointTrajectory:
    return api.models.JointTrajectory(
        joint_positions=[api.models.Joints([0.0] * 6)] * _NUM_SAMPLES,
        times=[float(i) for i in range(_NUM_SAMPLES)],
        locations=[api.models.Location(root=float(i)) for i in range(_NUM_SAMPLES)],
    )


def _state(
    standstill: bool, execute: api.models.Execute | None = None, sequence_number: int = 1
) -> api.models.MotionGroupState:
    return api.models.MotionGroupState(
        timestamp=datetime.now(timezone.utc),
        sequence_number=sequence_number,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=api.models.Joints(root=[0.0] * 6),
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
            trajectory="traj-1", location=api.models.Location(root=location), state=trajectory_state
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


async def _responses() -> AsyncIterator[api.models.ExecuteTrajectoryResponse]:
    yield api.models.ExecuteTrajectoryResponse(root=api.models.InitializeMovementResponse())
    yield api.models.ExecuteTrajectoryResponse(root=api.models.StartMovementResponse())
    await asyncio.Future()


def _one_shot_cursor(states: list[api.models.MotionGroupState]) -> TrajectoryCursor:
    """A cursor configured the way ``move_forward`` configures it (one-shot)."""
    return TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=_stream_then_block(states),
        joint_trajectory=_joint_trajectory(),
        initial_location=0.0,
        detach_on_standstill=True,
        emit_motion_events=False,
    )


async def _drive(cursor: TrajectoryCursor) -> asyncio.Task:
    """Consume the cursor's protocol generator in the background."""

    async def consume():
        async for _request in cursor.cntrl(_responses()):
            pass

    return asyncio.create_task(consume())


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
        # one-shot detach on ended → the protocol loop finishes
        await asyncio.gather(consumer)

    assert result.error is None
    assert result.final_location == 3.0


async def test_pause_edge_then_bare_standstill_completes_the_operation_as_paused():
    """The pause twin of the vanishing terminal state (pinned server-side by
    robotics/wbr!2322): on current controllers the paused trajectory is visible
    at standstill for a single control cycle before the execute block drops.
    A pause observed while decelerating, followed only by bare standstill
    frames, must still conclude the running operation — previously the machine
    hung in `pausing` forever."""
    cursor = _one_shot_cursor(
        [
            _state(False, _running(0.5)),
            _state(False, _running(1.0)),
            # pause takes effect while still decelerating…
            _state(False, _paused(1.2)),
            # …and the execute block is gone by the time standstill is reached
            _state(True),
            _state(True),
        ]
    )
    operation = cursor.forward()
    consumer = await _drive(cursor)

    try:
        async with asyncio.timeout(5):
            result = await operation
        assert result.error is None
        assert result.final_location == 1.2
    finally:
        # paused is not ended: one-shot does not detach on a pause
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
        await asyncio.gather(consumer)

    assert result.error is None
    assert result.final_location == 3.0, (
        "the forward operation must complete at the trajectory end, not be "
        "resolved by the pre-start PAUSED_BY_USER frames"
    )


async def test_pause_after_motion_still_completes_the_operation_as_paused():
    """Guarding the pre-start window must not break real pauses: once the
    operation was seen running, a persistent paused state concludes it."""
    cursor = _one_shot_cursor(
        [
            _state(False, _running(0.5)),
            _state(False, _running(1.0)),
            # pause requested elsewhere (pause_on_io, another client, …):
            # level-based publishing repeats it every step
            _state(True, _paused(1.2)),
            _state(True, _paused(1.2)),
        ]
    )
    operation = cursor.forward()
    consumer = await _drive(cursor)

    try:
        async with asyncio.timeout(5):
            result = await operation
        assert result.error is None
        assert result.final_location == 1.2
    finally:
        # paused is not ended: one-shot does not detach on a pause
        cursor.detach()
        async with asyncio.timeout(5):
            await asyncio.gather(consumer, return_exceptions=True)


# ---------------------------------------------------------------------------
# Unit level: motion evidence and pause-completion gating
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


class TestMayCompleteAsPaused:
    def _handler(self, operation_type: OperationType) -> OperationHandler:
        handler = OperationHandler()
        handler.start(
            operation_type,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )
        return handler

    async def test_no_operation(self):
        assert not OperationHandler().may_complete_as_paused()

    async def test_pause_operation_may_always_complete_as_paused(self):
        handler = self._handler(OperationType.PAUSE)
        assert handler.may_complete_as_paused()

    async def test_forward_operation_before_motion_may_not(self):
        handler = self._handler(OperationType.FORWARD)
        handler.set_commanded()
        assert not handler.may_complete_as_paused()

    async def test_forward_operation_after_motion_may(self):
        handler = self._handler(OperationType.FORWARD)
        handler.set_commanded()
        handler.set_running()
        assert handler.current_operation.operation_state is OperationState.RUNNING
        assert handler.may_complete_as_paused()
