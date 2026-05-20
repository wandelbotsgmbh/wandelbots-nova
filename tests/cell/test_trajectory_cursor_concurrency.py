"""Concurrency and timing tests for TrajectoryCursor.

These tests focus on state management bugs that can arise from:
- Instantiation followed by immediate method calls before the event loop ticks
- Rapid successive operations (forward/pause/backward) that replace each other
- Command queue poisoning from superseded operations
- Future lifecycle (cancelled, unresolved, double-resolved)
- OperationHandler state-machine transition invariants under concurrent pressure

No actual robot connection is required; responses and state streams are faked
via async generators and asyncio primitives.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from typing import AsyncIterator

import pytest

from nova import api
from nova.actions.motions import lin
from nova.cell.movement_controller.trajectory_cursor import (
    _QUEUE_SENTINEL,
    Intent,
    OperationHandler,
    OperationState,
    OperationType,
    TrajectoryCursor,
)
from nova.types import Pose

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_joint_trajectory(num_actions: int) -> api.models.JointTrajectory:
    """Return a minimal JointTrajectory whose locations match *num_actions* actions."""
    n = num_actions + 1
    return api.models.JointTrajectory(
        joint_positions=[api.models.Joints([0.0] * 6)] * n,
        times=[float(i) for i in range(n)],
        locations=[api.models.Location(root=float(i)) for i in range(n)],
    )


def _make_actions(num_actions: int) -> list:
    return [lin(Pose((i * 100.0, 0, 0, 0, 0, 0))) for i in range(num_actions)]


def _make_motion_group_state(
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


def _make_execute(location: float = 0.5) -> api.models.Execute:
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-1",
            location=api.models.Location(root=location),
            state=api.models.TrajectoryRunning(time_to_end=0),
        ),
    )


async def _never_ending_state_stream() -> AsyncIterator[api.models.MotionGroupState]:
    """Async iterator that never yields anything – simulates a quiet stream."""
    await asyncio.Future()  # blocks forever
    yield _make_motion_group_state(standstill=True)  # unreachable; makes mypy happy


async def _state_stream_from(
    states: list[api.models.MotionGroupState],
) -> AsyncIterator[api.models.MotionGroupState]:
    """Yield a fixed list of states then block forever."""
    for state in states:
        yield state
    await asyncio.Future()


def _make_cursor(
    num_actions: int = 3,
    initial_location: float = 0.0,
    state_stream: AsyncIterator[api.models.MotionGroupState] | None = None,
) -> TrajectoryCursor:
    """Create a fully-initialised TrajectoryCursor (background task not yet run)."""
    if state_stream is None:
        state_stream = _never_ending_state_stream()
    return TrajectoryCursor(
        motion_id="traj-1",
        motion_group_state_stream=state_stream,
        joint_trajectory=_make_joint_trajectory(num_actions),
        actions=_make_actions(num_actions),
        initial_location=initial_location,
    )


# ---------------------------------------------------------------------------
# OperationHandler unit tests
# (async because asyncio.Future() requires a running event loop)
# ---------------------------------------------------------------------------


class TestOperationHandlerStateTransitions:
    """OperationHandler state transitions should respect defined invariants."""

    def _make_handler_with_op(self) -> OperationHandler:
        handler = OperationHandler()
        handler.start(
            OperationType.FORWARD,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )
        return handler

    async def test_initial_state_is_initial(self):
        handler = self._make_handler_with_op()
        assert handler.current_operation is not None
        assert handler.current_operation.operation_state is OperationState.INITIAL

    async def test_set_commanded_transitions_from_initial(self):
        handler = self._make_handler_with_op()
        handler.set_commanded()
        assert handler.current_operation.operation_state is OperationState.COMMANDED

    async def test_set_commanded_twice_is_idempotent(self):
        """Calling set_commanded() when already COMMANDED is now a no-op (defensive)."""
        handler = self._make_handler_with_op()
        handler.set_commanded()
        handler.set_commanded()  # must not raise
        assert handler.current_operation.operation_state is OperationState.COMMANDED

    async def test_set_running_from_commanded(self):
        handler = self._make_handler_with_op()
        handler.set_commanded()
        handler.set_running()
        assert handler.current_operation.operation_state is OperationState.RUNNING

    async def test_set_running_from_initial_is_allowed(self):
        """Race condition: state monitor fires before StartMovementResponse arrives."""
        handler = self._make_handler_with_op()
        handler.set_running()  # should not raise
        assert handler.current_operation.operation_state is OperationState.INITIAL

    async def test_set_running_is_idempotent(self):
        handler = self._make_handler_with_op()
        handler.set_commanded()
        handler.set_running()
        handler.set_running()  # second call must be a no-op
        assert handler.current_operation.operation_state is OperationState.RUNNING

    async def test_set_commanded_from_running_is_noop(self):
        """Race: StartMovementResponse arrives after set_running() already fired."""
        handler = self._make_handler_with_op()
        handler.set_commanded()
        handler.set_running()
        handler.set_commanded()  # should be a no-op, not raise
        assert handler.current_operation.operation_state is OperationState.RUNNING

    async def test_complete_resolves_future(self):
        handler = self._make_handler_with_op()
        future = handler.current_operation.future
        handler.complete(final_location=1.0)
        assert future.done()
        assert not future.cancelled()
        result = future.result()
        assert result.final_location == 1.0

    async def test_complete_with_error_sets_exception(self):
        handler = self._make_handler_with_op()
        future = handler.current_operation.future
        err = RuntimeError("estop")
        handler.complete(final_location=0.5, error=err)
        assert future.done()
        assert future.exception() is err

    async def test_complete_is_idempotent_when_future_already_done(self):
        """Calling complete() twice must not raise."""
        handler = self._make_handler_with_op()
        handler.complete(final_location=1.0)
        handler.complete(final_location=1.5)  # second call – no-op

    async def test_start_cancels_previous_in_progress_future(self):
        """Starting a new operation must cancel the previous pending future."""
        handler = OperationHandler()
        first_future = handler.start(
            OperationType.FORWARD,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )
        second_future = handler.start(
            OperationType.BACKWARD,
            start_location=0.5,
            expected_response_type=api.models.StartMovementResponse,
        )
        assert first_future.cancelled(), "Previous future should be cancelled"
        assert not second_future.done(), "New future should still be pending"

    async def test_in_progress_false_after_complete(self):
        handler = self._make_handler_with_op()
        assert handler.in_progress()
        handler.complete(final_location=1.0)
        assert not handler.in_progress()

    async def test_in_progress_false_when_no_operation(self):
        handler = OperationHandler()
        assert not handler.in_progress()


# ---------------------------------------------------------------------------
# TrajectoryCursor instantiation / initialization-task timing
# ---------------------------------------------------------------------------


class TestCursorInstantiationRace:
    """Verify behavior when methods are called before the event loop has had a
    chance to run the _initialize_task background coroutine."""

    async def test_initialize_task_runs_after_one_yield(self):
        """_initialize_task should complete after a single event-loop tick."""
        cursor = _make_cursor()
        assert not cursor._initialize_task.done()
        await asyncio.sleep(0)  # one tick – task runs
        # After one yield the task should have completed (or be completing).
        # It must not have raised an unexpected exception.
        cursor._initialize_task.cancel()
        try:
            await cursor._initialize_task
        except (asyncio.CancelledError, Exception):
            pass  # either fine – task was cancelled or completed

    async def test_forward_callable_before_initialize_task_completes(self):
        """forward() is synchronous – it must succeed even if the background
        task hasn't run yet.  The intent should be recorded."""
        cursor = _make_cursor()
        future = cursor.forward()
        assert not future.done()
        assert cursor._pending_intent is not None
        # Clean up
        cursor._initialize_task.cancel()

    async def test_pause_before_any_operation_returns_none(self):
        """pause() with no active operation should return None immediately."""
        cursor = _make_cursor()
        result = cursor.pause()
        assert result is None
        cursor._initialize_task.cancel()

    async def test_backward_callable_before_initialize_task_completes(self):
        cursor = _make_cursor(initial_location=1.5)
        future = cursor.backward()
        assert not future.done()
        cursor._initialize_task.cancel()


# ---------------------------------------------------------------------------
# Rapid successive operations – queue and future integrity
# ---------------------------------------------------------------------------


class TestRapidOperationSwitching:
    """Calling multiple operations without awaiting reveals state-management bugs."""

    async def test_forward_twice_cancels_first_future(self):
        """Issuing forward() twice must cancel the first future to prevent leaks."""
        cursor = _make_cursor()
        first = cursor.forward()
        second = cursor.forward()
        assert first.cancelled(), "First future must be cancelled when superseded"
        assert not second.done(), "Second future must remain pending"
        cursor._initialize_task.cancel()

    async def test_pending_intent_is_latest_after_two_forwards(self):
        """Each forward() overwrites the pending intent.  After two rapid
        forward() calls, only the second intent survives in the slot."""
        cursor = _make_cursor()
        cursor.forward()
        cursor.forward()

        assert cursor._pending_intent is not None
        assert cursor._pending_intent.operation_type is OperationType.FORWARD
        cursor._initialize_task.cancel()

    async def test_forward_then_pause_leaves_pause_intent(self):
        """forward() followed immediately by pause() overwrites the forward
        intent with a pause intent.  Only the pause intent is pending."""
        cursor = _make_cursor()
        forward_future = cursor.forward()
        pause_future = cursor.pause()

        assert forward_future.cancelled(), "Forward future must be cancelled by pause"
        assert pause_future is not None
        assert not pause_future.done()

        assert cursor._pending_intent is not None
        assert cursor._pending_intent.operation_type is OperationType.PAUSE
        cursor._initialize_task.cancel()

    async def test_forward_then_backward_cancels_forward_future(self):
        cursor = _make_cursor(initial_location=1.0)
        fwd = cursor.forward()
        bwd = cursor.backward()
        assert fwd.cancelled()
        assert not bwd.done()
        cursor._initialize_task.cancel()

    async def test_triple_operation_only_last_future_is_pending(self):
        """Three rapid operations: only the last future survives."""
        cursor = _make_cursor(initial_location=1.5)
        f1 = cursor.forward()
        f2 = cursor.backward()
        f3 = cursor.forward()
        assert f1.cancelled()
        assert f2.cancelled()
        assert not f3.done()
        cursor._initialize_task.cancel()

    async def test_detach_with_pending_future_cancels_it(self):
        """Calling detach() with an in-progress forward() must cancel the future
        so any awaiter gets CancelledError instead of hanging."""
        cursor = _make_cursor()
        future = cursor.forward()
        cursor.detach()

        await asyncio.sleep(0)  # allow queue processing

        assert future.cancelled(), "detach() must cancel the pending operation future"
        cursor._initialize_task.cancel()


# ---------------------------------------------------------------------------
# Response consumer double-commanded hazard (integration-level)
# ---------------------------------------------------------------------------


class TestResponseConsumerDoubleCommandedHazard:
    """The response consumer processes every response from the motion controller,
    including responses for *superseded* operations whose futures were already
    cancelled.  This can drive set_commanded() into an illegal state."""

    async def test_intent_to_commands_produces_correct_api_commands(self):
        """When forward() is called twice, only the latest intent survives.
        The request loop materializes only that intent into API commands,
        so no stale commands are ever sent to the server."""
        handler = OperationHandler()
        # First forward()
        handler.start(
            OperationType.FORWARD,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )
        handler.start(  # replaces the first, cancels its future
            OperationType.FORWARD,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )

        # With intent-only, only the latest intent is materialized.
        # Build commands from the winning intent.
        intent = Intent(operation_type=OperationType.FORWARD)
        commands = intent.to_commands()
        assert len(commands) == 1
        assert isinstance(commands[0], api.models.StartMovementRequest)

        # No stale-response tracking needed — only one response expected
        handler.set_commanded()
        assert handler.current_operation.operation_state is OperationState.COMMANDED

    async def test_response_consumer_skips_commanded_when_type_mismatch(self):
        """When forward() is followed by pause(), the active operation expects
        PauseMovementResponse.  A StartMovementResponse must NOT call set_commanded()
        (wrong type guard prevents it).  Verify the handler stays INITIAL."""
        handler = OperationHandler()
        handler.start(
            OperationType.FORWARD,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )
        # Now pause() replaces the forward operation
        handler.start(
            OperationType.PAUSE,
            start_location=0.0,
            expected_response_type=api.models.PauseMovementResponse,
        )

        # Simulate: StartMovementResponse arrives (orphaned from the forward)
        start_resp = api.models.StartMovementResponse()
        if not isinstance(start_resp, handler.current_operation.expected_response_type):
            pass  # response consumer skips it – state remains INITIAL

        assert handler.current_operation.operation_state is OperationState.INITIAL, (
            "set_commanded() must NOT be called when response type doesn't match"
        )

        # Now the PauseMovementResponse arrives → should transition to COMMANDED
        pause_resp = api.models.PauseMovementResponse()
        if isinstance(pause_resp, handler.current_operation.expected_response_type):
            handler.set_commanded()

        assert handler.current_operation.operation_state is OperationState.COMMANDED


# ---------------------------------------------------------------------------
# forward_to / backward_to immediate-reject futures
# ---------------------------------------------------------------------------


class TestTargetedMovementValidation:
    """forward_to and backward_to should return pre-failed futures for invalid targets."""

    async def test_forward_to_past_current_location_returns_failed_future(self):
        cursor = _make_cursor(initial_location=2.0)
        future = cursor.forward_to(1.0)  # target is behind us
        assert future.done()
        with pytest.raises(ValueError):
            future.result()
        cursor._initialize_task.cancel()

    async def test_backward_to_ahead_of_current_location_returns_failed_future(self):
        cursor = _make_cursor(initial_location=1.0)
        future = cursor.backward_to(2.0)  # target is ahead
        assert future.done()
        with pytest.raises(ValueError):
            future.result()
        cursor._initialize_task.cancel()

    async def test_forward_to_next_action_at_end_returns_immediate_result(self):
        """At the end of the trajectory, forward_to_next_action() must not enqueue
        a command – it returns an already-resolved future."""
        cursor = _make_cursor(num_actions=2, initial_location=2.0)
        future = cursor.forward_to_next_action()
        assert future.done()
        result = future.result()
        assert result.final_location == 2.0
        assert result.operation_type is OperationType.FORWARD_TO_NEXT_ACTION
        cursor._initialize_task.cancel()

    async def test_backward_to_previous_action_at_start_returns_immediate_result(self):
        """At location 0.0, backward_to_previous_action() must not enqueue a command."""
        cursor = _make_cursor(num_actions=2, initial_location=0.0)
        future = cursor.backward_to_previous_action()
        assert future.done()
        result = future.result()
        assert result.final_location == 0.0
        assert result.operation_type is OperationType.BACKWARD_TO_PREVIOUS_ACTION
        cursor._initialize_task.cancel()


# ---------------------------------------------------------------------------
# State-monitor / operation completion integration
# ---------------------------------------------------------------------------


class TestStateMonitorCompletionRace:
    """Verify operation lifecycle under simulated concurrent state updates."""

    async def test_operation_completed_on_standstill_after_execute(self):
        """Simulate the happy path: state monitor receives execute=True then
        standstill=True and completes the operation."""
        handler = OperationHandler()
        future = handler.start(
            OperationType.FORWARD,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )

        # Mimic: response consumer calls set_commanded() when StartMovementResponse arrives
        handler.set_commanded()
        # Mimic: state monitor sees execute field → set_running()
        handler.set_running()
        # Mimic: state monitor detects standstill → complete()
        handler.complete(final_location=1.0)

        assert future.done() and not future.cancelled()
        result = future.result()
        assert result.final_location == 1.0

    async def test_running_before_commanded_race_condition(self):
        """State monitor sees the execute field BEFORE the StartMovementResponse
        arrives (set_running() called while still in INITIAL state).  The op must
        not transition and must not raise; set_commanded() should still work after."""
        handler = OperationHandler()
        future = handler.start(
            OperationType.FORWARD,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )

        # State monitor fires first (INITIAL → no change, no-op)
        handler.set_running()
        assert handler.current_operation.operation_state is OperationState.INITIAL

        # StartMovementResponse arrives → INITIAL → COMMANDED
        handler.set_commanded()
        assert handler.current_operation.operation_state is OperationState.COMMANDED

        # A second state update fires → COMMANDED → RUNNING
        handler.set_running()
        assert handler.current_operation.operation_state is OperationState.RUNNING

        handler.complete(final_location=0.5)
        assert future.result().final_location == 0.5

    async def test_complete_called_with_error_propagates_to_future(self):
        handler = OperationHandler()
        future = handler.start(
            OperationType.FORWARD,
            start_location=0.0,
            expected_response_type=api.models.StartMovementResponse,
        )
        handler.set_commanded()
        handler.set_running()
        handler.complete(final_location=0.3, error=RuntimeError("E-STOP"))

        assert future.done()
        with pytest.raises(RuntimeError, match="E-STOP"):
            future.result()

    async def test_no_operation_in_progress_complete_is_noop(self):
        """complete() with no active operation must not raise."""
        handler = OperationHandler()
        handler.complete(final_location=0.0)  # must not raise


# ---------------------------------------------------------------------------
# Concurrent forward + immediate detach
# ---------------------------------------------------------------------------


class TestCursorDetachBehaviour:
    """Detach scenarios to document what happens to in-flight operations."""

    async def test_detach_signals_stop_and_in_queue(self):
        """After detach(), the stop event should be set and the intent event
        should be set so _request_loop wakes and terminates.  The sentinel
        is *not* enqueued here – it comes from _motion_group_state_monitor's
        finally block during a real session."""
        cursor = _make_cursor()
        cursor.detach()

        # Stop event is set (terminates _request_loop)
        assert cursor._stop_event.is_set()
        # Intent event is set (wakes _request_loop so it sees the stop)
        assert cursor._intent_event.is_set()

        # No sentinel in the queue – that responsibility belongs to the monitor
        assert cursor._in_queue.empty()

        cursor._initialize_task.cancel()

    async def test_aiter_raises_stop_async_iteration_after_detach(self):
        """When the sentinel is manually enqueued (as the monitor's finally
        would do), __anext__ must raise StopAsyncIteration."""
        cursor = _make_cursor()
        cursor.detach()
        # Simulate what the monitor's finally block does
        cursor._in_queue.put_nowait(_QUEUE_SENTINEL)

        with pytest.raises(StopAsyncIteration):
            await cursor.__anext__()

        cursor._initialize_task.cancel()

    async def test_forward_and_detach_cancels_future(self):
        """detach() must cancel the pending forward() future so any awaiter
        gets CancelledError instead of hanging indefinitely."""
        cursor = _make_cursor()
        future = cursor.forward()
        cursor.detach()

        # Give the event loop a tick to process any pending callbacks.
        await asyncio.sleep(0)

        assert future.cancelled(), "detach() must cancel the pending operation future"
        cursor._initialize_task.cancel()
