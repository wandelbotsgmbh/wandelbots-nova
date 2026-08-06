"""Regression tests for TrajectoryCursor movement-controller parity.

These cover the defects found while establishing that a cursor driven with a
single ``forward()`` is equivalent to the ``move_forward`` movement controller:

- the IO overlay must travel with *every* ``StartMovementRequest`` (the server
  treats each start as an override of the previously attached overlay, so a
  resume that omits it silently clears the remaining outputs);
- a pending intent must not be discarded when the cursor stops, which otherwise
  produced no ``StartMovementRequest`` at all against a short state stream;
- a terminal trajectory state must not complete an operation that the controller
  never acknowledged, which otherwise reported a successful traversal for
  movement that was never commanded.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from typing import AsyncIterator

import pytest

from nova import api
from nova.actions.motions import lin
from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor
from nova.exceptions import ErrorDuringMovement
from nova.types import Pose

pytestmark = pytest.mark.asyncio


def _trajectory(num_actions: int) -> api.models.JointTrajectory:
    n = num_actions + 1
    return api.models.JointTrajectory(
        joint_positions=[api.models.Joints([0.0] * 6)] * n,
        times=[float(i) for i in range(n)],
        locations=[api.models.Location(root=float(i)) for i in range(n)],
    )


def _actions(num_actions: int) -> list:
    return [lin(Pose((i * 100.0, 0, 0, 0, 0, 0))) for i in range(num_actions)]


def _state(
    standstill: bool, execute: api.models.Execute | None = None
) -> api.models.MotionGroupState:
    return api.models.MotionGroupState(
        timestamp=datetime.now(timezone.utc),
        sequence_number=1,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=api.models.Joints(root=[0.0] * 6),
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(limit_reached=[False] * 6),
        standstill=standstill,
        execute=execute,
        description_revision=1,
    )


def _execute(location: float, state=None) -> api.models.Execute:
    return api.models.Execute(
        joint_position=[0.0] * 6,
        details=api.models.TrajectoryDetails(
            trajectory="traj-1",
            location=api.models.Location(root=location),
            state=state or api.models.TrajectoryRunning(time_to_end=0),
        ),
    )


def _set_io(io: str, location: float) -> api.models.SetIO:
    return api.models.SetIO(
        io=api.models.IOValue(api.models.IOBooleanValue(io=io, value=True)),
        location=location,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )


async def _finite_states() -> AsyncIterator[api.models.MotionGroupState]:
    """A short stream that ends — as the movement-controller unit fixtures do."""
    yield _state(False, _execute(0.5))
    yield _state(True, _execute(3.0, api.models.TrajectoryEnded()))
    yield _state(True)


async def _blocking_states() -> AsyncIterator[api.models.MotionGroupState]:
    """A stream that stays open, as a live websocket does."""
    yield _state(False, _execute(0.5))
    await asyncio.Future()


def _states_after(started: asyncio.Event) -> AsyncIterator[api.models.MotionGroupState]:
    """States that only report progress once movement has been commanded.

    An idle state is emitted immediately — a real motion-group stream is always
    live, independent of any trajectory — so the cursor's startup gate is
    satisfied. Trajectory progress and completion only follow the start command,
    which is the ordering a controller actually produces.
    """

    async def gen() -> AsyncIterator[api.models.MotionGroupState]:
        yield _state(True)
        await started.wait()
        yield _state(False, _execute(0.5))
        yield _state(True, _execute(3.0, api.models.TrajectoryEnded()))
        yield _state(True)

    return gen()


async def _responses(*inner) -> AsyncIterator[api.models.ExecuteTrajectoryResponse]:
    for item in inner:
        yield api.models.ExecuteTrajectoryResponse(root=item)
    await asyncio.Future()


def _gated_run() -> tuple[AsyncIterator, AsyncIterator, asyncio.Event]:
    """A (state stream, response stream, start-sent event) triple.

    Models the real ordering: the controller neither acknowledges nor reports any
    trajectory progress until the start command has actually been sent. Without
    this the mocked state stream can run to completion — tearing the cursor down —
    before ``_request_loop`` has had a turn.
    """
    start_sent = asyncio.Event()

    async def responses():
        yield api.models.ExecuteTrajectoryResponse(root=api.models.InitializeMovementResponse())
        await start_sent.wait()
        yield api.models.ExecuteTrajectoryResponse(root=api.models.StartMovementResponse())
        await asyncio.Future()

    return _states_after(start_sent), responses(), start_sent


async def _drive(
    cursor: TrajectoryCursor, responses, timeout: float = 3.0, on_start: asyncio.Event | None = None
) -> list:
    """Run ``cntrl`` to completion, collecting the requests it yields.

    ``on_start`` is set as soon as a ``StartMovementRequest`` leaves the cursor, so
    fixtures can withhold controller feedback until movement was actually commanded.
    """
    requests: list = []

    async def run():
        async for request in cursor.cntrl(responses):
            requests.append(request)
            if on_start is not None and isinstance(request, api.models.StartMovementRequest):
                on_start.set()

    task = asyncio.create_task(run())
    try:
        await asyncio.wait_for(task, timeout=timeout)
    except asyncio.TimeoutError:
        task.cancel()
        raise
    return requests


class TestSetOutputsOverlay:
    """The IO overlay must be attached to every start the cursor emits."""

    async def test_set_outputs_on_initial_start(self):
        outputs = [_set_io("OUT#900", 1.0)]
        states, responses, start_sent = _gated_run()
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=states,
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            set_outputs=outputs,
            detach_on_standstill=True,
            emit_motion_events=False,
        )
        cursor.forward()
        requests = await _drive(cursor, responses, on_start=start_sent)

        starts = [r for r in requests if isinstance(r, api.models.StartMovementRequest)]
        assert len(starts) == 1
        assert starts[0].set_outputs == outputs

    async def test_set_outputs_reattached_on_resume(self):
        """A resume must re-send the overlay; omitting it clears it server-side."""
        outputs = [_set_io("OUT#900", 1.0)]
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_finite_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            set_outputs=outputs,
            emit_motion_events=False,
        )

        def start_command_of(intent):
            command = intent.to_commands()[0]
            assert isinstance(command, api.models.StartMovementRequest)
            return command

        cursor.forward()
        assert start_command_of(cursor._pending_intent).set_outputs == outputs

        # A pause carries no overlay of its own ...
        cursor._operation_handler.set_commanded()
        cursor.pause()
        assert isinstance(cursor._pending_intent.to_commands()[0], api.models.PauseMovementRequest)

        # ... and the resume must carry the full overlay again.
        cursor.forward()
        assert start_command_of(cursor._pending_intent).set_outputs == outputs
        cursor._initialize_task.cancel()

    async def test_backward_start_also_carries_outputs(self):
        outputs = [_set_io("OUT#900", 1.0)]
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_finite_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            initial_location=2.0,
            set_outputs=outputs,
            emit_motion_events=False,
        )
        cursor.backward()
        intent = cursor._pending_intent
        assert intent is not None
        command = intent.to_commands()[0]
        assert isinstance(command, api.models.StartMovementRequest)
        assert command.direction is api.models.Direction.DIRECTION_BACKWARD
        assert command.set_outputs == outputs
        cursor._initialize_task.cancel()

    async def test_constructor_io_defaults_applied_to_start(self):
        start_on_io = api.models.StartOnIO(
            io=api.models.IOBooleanValue(io="IN#1", value=True),
            comparator=api.models.Comparator.COMPARATOR_EQUALS,
            io_origin=api.models.IOOrigin.CONTROLLER,
        )
        pause_on_io = api.models.PauseOnIO(
            io=api.models.IOBooleanValue(io="IN#2", value=True),
            comparator=api.models.Comparator.COMPARATOR_EQUALS,
            io_origin=api.models.IOOrigin.CONTROLLER,
        )
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_finite_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            start_on_io=start_on_io,
            pause_on_io=pause_on_io,
            emit_motion_events=False,
        )
        cursor.forward()
        command = cursor._pending_intent.to_commands()[0]
        assert command.start_on_io == start_on_io
        assert command.pause_on_io == pause_on_io
        cursor._initialize_task.cancel()


class TestPendingIntentSurvivesStop:
    """A queued command must never reach the wire once the cursor is stopping."""

    async def test_queued_intent_is_not_sent_after_detach(self):
        """An explicit abort must not be followed by a start command.

        ``detach()`` cancels the caller's operation, so emitting the queued start
        afterwards would move the robot after the caller was told the movement was
        aborted.
        """
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_blocking_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            emit_motion_events=False,
        )
        future = cursor.forward()
        cursor.detach()

        sent = []
        request_loop = cursor._request_loop()
        try:
            async with asyncio.timeout(1.0):
                async for request in request_loop:
                    sent.append(request)
        except (asyncio.TimeoutError, StopAsyncIteration):
            pass

        assert sent == [], "no command may be sent after the cursor was detached"
        assert future.cancelled()
        cursor._initialize_task.cancel()

    async def test_dropped_intent_fails_rather_than_reporting_success(self):
        """A start that never went out must surface as an error, not a traversal.

        Regression: a short state stream tore the cursor down before the queued
        intent was sent, and the terminal state then resolved the operation as a
        successful move to the end of the trajectory.
        """
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_finite_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            detach_on_standstill=True,
            emit_motion_events=False,
        )
        future = cursor.forward()
        await cursor._motion_group_state_monitor(ready_event=asyncio.Event())

        assert future.done()
        with pytest.raises(ErrorDuringMovement):
            future.result()
        cursor._initialize_task.cancel()


class TestOperationRequiresAcknowledgement:
    """A terminal state must not complete an operation that was never commanded."""

    async def test_uncommanded_operation_is_not_completed_by_terminal_state(self):
        """Regression: forward() resolved successfully having commanded nothing.

        The operation is created but ``cntrl`` never runs, so no request reaches
        the wire. A terminal trajectory state must not be attributed to it.
        """
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_finite_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            emit_motion_events=False,
        )
        future = cursor.forward()
        assert not cursor._operation_handler.is_commanded()

        # Drive the state monitor directly: no command was ever sent.
        await cursor._motion_group_state_monitor(ready_event=asyncio.Event())

        assert future.done()
        with pytest.raises(ErrorDuringMovement):
            future.result()
        cursor._initialize_task.cancel()

    async def test_sent_command_marks_the_operation_commanded(self):
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_blocking_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            emit_motion_events=False,
        )
        cursor.forward()
        assert not cursor._operation_handler.is_commanded()

        # Consume one request from the loop: that is the point of no return.
        request_loop = cursor._request_loop()
        request = await anext(request_loop)

        assert isinstance(request, api.models.StartMovementRequest)
        assert cursor._operation_handler.is_commanded()
        await request_loop.aclose()
        cursor._initialize_task.cancel()

    async def test_sent_pause_marks_the_operation_commanded(self):
        """A pause must be gated the same way, or it can never complete.

        Only ``PauseMovementResponse`` would otherwise mark it commanded, so a
        delayed or lost ack would leave ``pause()`` unresolved.
        """
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_blocking_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            emit_motion_events=False,
        )
        cursor.forward()
        request_loop = cursor._request_loop()
        await anext(request_loop)  # the start

        cursor.pause()
        request = await anext(request_loop)
        assert isinstance(request, api.models.PauseMovementRequest)
        assert cursor._operation_handler.is_commanded()
        await request_loop.aclose()
        cursor._initialize_task.cancel()

    async def test_success_when_start_is_acknowledged(self):
        states, responses, start_sent = _gated_run()
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=states,
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            detach_on_standstill=True,
            emit_motion_events=False,
        )
        future = cursor.forward()
        requests = await _drive(cursor, responses, on_start=start_sent)

        assert any(isinstance(r, api.models.StartMovementRequest) for r in requests)
        result = await future
        assert result.error is None
        assert result.final_location == pytest.approx(3.0)


class TestOptionalConstructorArguments:
    """Parity requirements for one-shot execution."""

    async def test_empty_actions_is_not_a_zero_length_trajectory(self):
        """``actions=[]`` carries no mapping information and must be accepted."""
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_finite_states(),
            joint_trajectory=_trajectory(3),
            actions=[],
            emit_motion_events=False,
        )
        assert cursor.actions is None
        assert cursor.current_action is None
        cursor._initialize_task.cancel()

    async def test_joint_trajectory_is_optional(self):
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_finite_states(),
            actions=_actions(3),
            emit_motion_events=False,
        )
        with pytest.raises(ValueError, match="without a joint_trajectory"):
            _ = cursor.end_location
        cursor._initialize_task.cancel()

    async def test_state_stream_may_be_a_factory(self):
        states, responses, start_sent = _gated_run()
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=lambda: states,
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            detach_on_standstill=True,
            emit_motion_events=False,
        )
        cursor.forward()
        requests = await _drive(cursor, responses, on_start=start_sent)
        assert any(isinstance(r, api.models.StartMovementRequest) for r in requests)


class TestMovementErrorPropagation:
    """Callers expect ErrorDuringMovement, not a bare exception group."""

    async def test_movement_error_surfaces_unwrapped(self):
        cursor = TrajectoryCursor(
            motion_id="traj-1",
            motion_group_state_stream=_blocking_states(),
            joint_trajectory=_trajectory(3),
            actions=_actions(3),
            emit_motion_events=False,
        )
        cursor.forward()
        with pytest.raises(ErrorDuringMovement, match="boom"):
            await _drive(
                cursor,
                _responses(
                    api.models.InitializeMovementResponse(),
                    api.models.StartMovementResponse(),
                    api.models.MovementErrorResponse(message="boom"),
                ),
            )
