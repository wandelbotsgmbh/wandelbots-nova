"""Trajectory cursor for controlling robot movement along a planned trajectory.

This module provides the TrajectoryCursor class and supporting types for interactive
control of robot movement execution. It enables forward/backward movement along a
trajectory, pausing, and stepping through individual actions.

Key concepts:
    - **Location**: A float representing position along the trajectory. Integer values
      correspond to action boundaries (e.g., 0.0 is start of first action, 1.0 is start
      of second action).
    - **Operation**: A single movement command (forward, backward, pause) that can be
      awaited for completion.
    - **Action**: A motion primitive (e.g., ptp, lin) that makes up the trajectory.

Example usage:
    ```python
    cursor = TrajectoryCursor(
        motion_id=motion_id,
        motion_group_state_stream=state_stream,
        joint_trajectory=trajectory,
        actions=actions,
        initial_location=0.0,
    )

    # Move forward to end
    result = await cursor.forward()

    # Step through actions one at a time
    await cursor.forward_to_next_action()
    await cursor.backward_to_previous_action()

    # Pause current movement
    cursor.pause()
    ```
"""

import asyncio
import logging
from dataclasses import dataclass
from datetime import datetime
from enum import Enum, StrEnum, auto
from math import ceil, floor
from typing import AsyncIterator, Optional, Union

import pydantic
from blinker import signal
from icecream import ic

from nova import api
from nova.actions.base import Action
from nova.actions.container import CombinedActions
from nova.exceptions import InitMovementFailed
from nova.types import ExecuteTrajectoryRequestStream, ExecuteTrajectoryResponseStream

logger = logging.getLogger(__name__)

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")

_STREAM_STARTUP_TIMEOUT = 5.0


class OperationType(Enum):
    """Types of movement operations that can be performed on a trajectory.

    Attributes:
        FORWARD: Move forward along the trajectory (towards end).
        BACKWARD: Move backward along the trajectory (towards start).
        FORWARD_TO: Move forward to a specific location.
        BACKWARD_TO: Move backward to a specific location.
        FORWARD_TO_NEXT_ACTION: Move forward to the start of the next action.
        BACKWARD_TO_PREVIOUS_ACTION: Move backward to the start of the previous action.
        PAUSE: Pause the current movement.
    """

    FORWARD = auto()
    BACKWARD = auto()
    FORWARD_TO = auto()
    BACKWARD_TO = auto()
    FORWARD_TO_NEXT_ACTION = auto()
    BACKWARD_TO_PREVIOUS_ACTION = auto()
    PAUSE = auto()


class OperationState(Enum):
    """State machine states for a movement operation.

    The state transitions are:
        INITIAL -> COMMANDED -> RUNNING -> COMPLETED
                            \\-> FAILED
                            \\-> CANCELLED

    Attributes:
        INITIAL: Operation created but not yet sent to the controller.
        COMMANDED: Command sent to controller, awaiting execution start.
        RUNNING: Robot is actively moving (standstill is False).
        COMPLETED: Movement finished successfully (standstill is True).
        FAILED: Movement failed (e.g., E-STOP triggered).
        CANCELLED: Operation was cancelled before completion.
    """

    INITIAL = auto()
    COMMANDED = auto()
    RUNNING = auto()
    COMPLETED = auto()
    FAILED = auto()
    CANCELLED = auto()


@dataclass
class OperationResult:
    """Result returned when a movement operation completes.

    Attributes:
        operation_type: The type of operation that was performed.
        target_location: The intended destination location (if specified).
        start_location: The location where the operation started.
        final_location: The actual location where the robot stopped.
        error: Exception if the operation failed, None otherwise.
    """

    operation_type: OperationType
    target_location: Optional[float] = None
    start_location: Optional[float] = None
    final_location: Optional[float] = None
    error: Optional[Exception] = None


# Type alias for expected response types in _response_consumer
ExpectedResponseType = Union[
    type[api.models.StartMovementResponse], type[api.models.PauseMovementResponse]
]


@dataclass
class Operation:
    """Encapsulates all state for a single movement operation.

    This dataclass tracks the complete lifecycle of a movement operation,
    from creation through completion or cancellation.

    Attributes:
        future: Async future that resolves when the operation completes.
        operation_type: The type of movement being performed.
        operation_state: Current state in the operation lifecycle.
        start_location: Trajectory location when the operation began.
        expected_response_type: The API response type expected for this operation.
        target_location: Target location for targeted movements (forward_to, backward_to).
        interrupt_requested: Flag indicating if cancellation was requested.
    """

    future: asyncio.Future[OperationResult]
    operation_type: OperationType
    operation_state: OperationState
    start_location: float
    expected_response_type: ExpectedResponseType
    target_location: Optional[float] = None
    interrupt_requested: bool = False


class OperationHandler:
    """Manages the lifecycle of movement operations for a TrajectoryCursor.

    This class handles state transitions for operations, ensuring proper sequencing
    and completion of movement commands. Only one operation can be active at a time;
    starting a new operation cancels any pending operation.

    The handler tracks operation state through the lifecycle:
    INITIAL -> COMMANDED -> RUNNING -> COMPLETED/FAILED/CANCELLED
    """

    def __init__(self):
        self._operation: Optional[Operation] = None

    def start(
        self,
        operation_type: OperationType,
        *,
        start_location: float,
        expected_response_type: ExpectedResponseType,
        target_location: Optional[float] = None,
    ) -> asyncio.Future[OperationResult]:
        """Start a new movement operation.

        If an operation is already in progress, it will be cancelled before
        starting the new one.

        Args:
            operation_type: The type of movement to perform.
            start_location: Current position on the trajectory.
            expected_response_type: API response type to expect.
            target_location: Target position for targeted movements.

        Returns:
            A Future that resolves with OperationResult when the operation completes.
        """
        if self._operation and not self._operation.future.done():
            self._operation.future.cancel()

        future: asyncio.Future[OperationResult] = asyncio.Future()
        self._operation = Operation(
            future=future,
            operation_type=operation_type,
            operation_state=OperationState.INITIAL,
            start_location=start_location,
            expected_response_type=expected_response_type,
            target_location=target_location,
            interrupt_requested=False,
        )
        return future

    def set_commanded(self):
        """Transition operation state to COMMANDED.

        Called when the movement command has been sent to the controller
        This is idempotent if already in RUNNING state (due to race conditions
        between state updates and response processing).

        Raises:
            RuntimeError: If transition from current state is invalid.
        """
        assert self._operation is not None
        # we fail if we are already commanded since that would be a logic error
        assert self._operation.operation_state is not OperationState.COMMANDED
        if self._operation.operation_state == OperationState.INITIAL:
            self._operation.operation_state = OperationState.COMMANDED
        elif self._operation.operation_state == OperationState.RUNNING:
            # no-op, already running; this can happen if we process the motion group state with
            # the execute set before the ExecuteTrajectoryResponse
            pass
        else:
            raise RuntimeError(
                f"Cannot set operation to COMMANDED from state {self._operation.operation_state}"
            )

    def set_running(self):
        """Transition operation state to RUNNING.

        Called when the robot begins moving (standstill becomes False).
        This is idempotent and handles race conditions between state updates.

        Raises:
            RuntimeError: If transition from current state is invalid.
        """
        assert self._operation is not None
        if self._operation.operation_state == OperationState.COMMANDED:
            self._operation.operation_state = OperationState.RUNNING
        elif self._operation.operation_state == OperationState.RUNNING:
            # no-op, already running; idempotent
            pass
        elif self._operation.operation_state == OperationState.INITIAL:
            # no-op, this can happen if we process the motion group state with
            # the execute set before the ExecuteTrajectoryResponse which sets COMMANDED
            pass
        else:
            raise RuntimeError(
                f"Cannot set operation to RUNNING from state {self._operation.operation_state}"
            )

    def complete(self, *, final_location: float, error: Optional[Exception] = None) -> None:
        """Complete the current operation and resolve its future.

        Args:
            final_location: The trajectory location where movement stopped.
            error: Optional exception if the operation failed.
        """
        if not self._operation or self._operation.future.done():
            return

        result = OperationResult(
            operation_type=self._operation.operation_type,
            target_location=self._operation.target_location,
            start_location=self._operation.start_location,
            final_location=final_location,
            error=error,
        )
        if error:
            self._operation.future.set_exception(error)
        else:
            ic(result)
            self._operation.future.set_result(result)
        self._reset()

    def in_progress(self) -> bool:
        """Check if an operation is currently active.

        Returns:
            True if an operation exists and its future is not yet resolved.
        """
        return self._operation is not None and not self._operation.future.done()

    @property
    def current_operation(self) -> Optional[Operation]:
        """Get the current operation, if any."""
        return self._operation

    def _reset(self):
        """Clear the current operation."""
        self._operation = None


class MovementOption(StrEnum):
    """Available movement options based on current trajectory position.

    Attributes:
        CAN_MOVE_FORWARD: Robot can move forward (not at end of trajectory).
        CAN_MOVE_BACKWARD: Robot can move backward (not at start of trajectory).
    """

    CAN_MOVE_FORWARD = auto()
    CAN_MOVE_BACKWARD = auto()


# Signals emitted during motion events for external observers
motion_started = signal("motion_started")
motion_stopped = signal("motion_stopped")


class MotionEventType(StrEnum):
    """Types of motion events emitted by the cursor.

    Attributes:
        STARTED: Motion has begun or is continuing.
        STOPPED: Motion has stopped.
    """

    STARTED = auto()
    STOPPED = auto()


class MotionEvent(pydantic.BaseModel):
    """Event data emitted when motion state changes.

    Attributes:
        type: Whether motion started or stopped.
        current_location: Current position on the trajectory.
        current_action: The action at the current location.
        target_location: The intended destination location.
        target_action: The action at the target location.
    """

    type: MotionEventType
    current_location: float
    current_action: Action
    target_location: float
    target_action: Action


ExecuteTrajectoryRequestCommand = Union[
    api.models.InitializeMovementRequest,
    api.models.StartMovementRequest,
    api.models.PauseMovementRequest,
    api.models.PlaybackSpeedRequest,
]


@dataclass(frozen=True)
class _QueueSentinel:
    """Marker type used only as a sentinel for queue termination."""


# The single sentinel value used to signal queue termination
_QUEUE_SENTINEL = _QueueSentinel()


class TrajectoryCursor:
    """Interactive controller for navigating along a planned robot trajectory.

    The TrajectoryCursor provides bidirectional control over trajectory execution,
    allowing forward/backward movement, pausing, and stepping through individual
    actions. It manages the communication with the motion controller via async
    streams and emits events for UI integration.

    The cursor uses a location-based coordinate system where integer values
    represent action boundaries:
        - Location 0.0 = start of first action
        - Location 1.0 = start of second action (end of first)
        - Location N.0 = end of trajectory (for N actions)

    Attributes:
        motion_id: Unique identifier for this motion execution.
        joint_trajectory: The planned joint-space trajectory.
        actions: The sequence of motion actions in the trajectory.

    Example:
        ```python
        async with motion_group.execute_trajectory_async(actions) as cursor:
            # Move forward through the trajectory
            await cursor.forward()

            # Or step through action by action
            while cursor.get_movement_options() & {MovementOption.CAN_MOVE_FORWARD}:
                result = await cursor.forward_to_next_action()
                print(f"Completed action at location {result.final_location}")
        ```
    """

    def __init__(
        self,
        motion_id: str,
        motion_group_state_stream: AsyncIterator[api.models.MotionGroupState],
        joint_trajectory: api.models.JointTrajectory,
        actions: list[Action],
        initial_location: float,
        detach_on_standstill: bool = False,
    ):
        """Initialize a trajectory cursor.

        Args:
            motion_id: Unique identifier for this motion execution.
            motion_group_state_stream: Async stream of motion group state updates.
            joint_trajectory: The planned joint-space trajectory to execute.
            actions: List of motion actions that make up the trajectory.
            initial_location: Starting position on the trajectory (usually 0.0).
            detach_on_standstill: If True, automatically detach when robot stops.
        """
        self.motion_id = motion_id
        self.joint_trajectory = joint_trajectory
        self.actions = CombinedActions(items=actions)  # type: ignore
        self._command_queue: asyncio.Queue[ExecuteTrajectoryRequestCommand | _QueueSentinel] = (
            asyncio.Queue()
        )
        self._in_queue: asyncio.Queue[api.models.MotionGroupState | _QueueSentinel] = (
            asyncio.Queue()
        )
        self._motion_group_state_stream: AsyncIterator[api.models.MotionGroupState] = (
            motion_group_state_stream
        )

        self._current_location = initial_location
        # TODO maybe None instead until we have a target?
        self._target_location = self._current_location
        self._detach_on_standstill = detach_on_standstill

        self._operation_handler = OperationHandler()

        self._initialize_task = asyncio.create_task(self.ainitialize())

    async def ainitialize(self):
        """Async initialization that emits the initial motion event."""
        target_action = self.current_action
        await motion_started.send_async(self, event=self._get_motion_event(target_action))

    @property
    def end_location(self) -> float:
        """The location value at the end of the trajectory."""
        assert getattr(self, "joint_trajectory", None) is not None
        return self.joint_trajectory.locations[-1].root

    @property
    def current_action_start(self) -> float:
        """Location where the current action begins (floor of current location)."""
        return float(floor(self._current_location))

    @property
    def current_action_end(self) -> float:
        """Location where the current action ends (ceil of current location)."""
        return float(ceil(self._current_location))

    @property
    def next_action_start(self) -> float:
        """Location where the next action begins."""
        return self.current_action_end

    @property
    def previous_action_start(self) -> float:
        """Location where the previous action begins."""
        return self.current_action_start - 1.0

    @property
    def current_action_index(self) -> int:
        """Zero-based index of the action at the current location."""
        index = floor(self._current_location)
        if index >= len(self.actions):
            # at the end of the trajectory current action remains the last one
            return len(self.actions) - 1
        return index

    @property
    def current_action(self) -> Action:
        """The action at the current trajectory location."""
        return self.actions[self.current_action_index]

    @property
    def next_action(self) -> Action | None:
        """The action after the current one, or None if at end."""
        next_action_index = self.current_action_index + 1
        if next_action_index >= len(self.actions):
            return None
        return self.actions[next_action_index]

    @property
    def previous_action(self) -> Action | None:
        """The action before the current one, or None if at start."""
        previous_action_index = self.current_action_index - 1
        if previous_action_index < 0:
            return None
        return self.actions[previous_action_index]

    def get_movement_options(self) -> set[MovementOption]:
        """Get the set of currently available movement options.

        Returns:
            Set containing CAN_MOVE_FORWARD if not at end, CAN_MOVE_BACKWARD if not at start.
        """
        options: dict[MovementOption, bool] = {
            MovementOption.CAN_MOVE_FORWARD: self._current_location < self.end_location,
            MovementOption.CAN_MOVE_BACKWARD: self._current_location > 0.0,
        }
        return {option for option, available in options.items() if available}

    def forward(
        self, target_location: float | None = None, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        """Move forward along the trajectory.

        Starts or continues forward movement towards the end of the trajectory,
        or to a specific target location if provided.

        Args:
            target_location: Optional location to stop at. If None, moves to end.
            playback_speed_in_percent: Optional speed override (1-100).

        Returns:
            Future that resolves with OperationResult when movement stops.
        """
        future = self._start_operation(
            OperationType.FORWARD, expected_response_type=api.models.StartMovementResponse
        )

        if target_location is not None:
            self._target_location = target_location

        if playback_speed_in_percent is not None:
            self._command_queue.put_nowait(
                api.models.PlaybackSpeedRequest(playback_speed_in_percent=playback_speed_in_percent)
            )
        self._command_queue.put_nowait(
            api.models.StartMovementRequest(
                direction=api.models.Direction.DIRECTION_FORWARD,
                target_location=(
                    api.models.Location(root=target_location)
                    if target_location is not None
                    else None
                ),
                start_on_io=None,
                pause_on_io=None,
            )
        )

        return future

    def backward(
        self, target_location: float | None = None, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        """Move backward along the trajectory.

        Starts or continues backward movement towards the start of the trajectory,
        or to a specific target location if provided.

        Args:
            target_location: Optional location to stop at. If None, moves to start.
            playback_speed_in_percent: Optional speed override (1-100).

        Returns:
            Future that resolves with OperationResult when movement stops.
        """
        future = self._start_operation(
            OperationType.BACKWARD, expected_response_type=api.models.StartMovementResponse
        )

        if playback_speed_in_percent is not None:
            self._command_queue.put_nowait(
                api.models.PlaybackSpeedRequest(playback_speed_in_percent=playback_speed_in_percent)
            )
        self._command_queue.put_nowait(
            api.models.StartMovementRequest(
                direction=api.models.Direction.DIRECTION_BACKWARD,
                target_location=(
                    api.models.Location(root=target_location)
                    if target_location is not None
                    else None
                ),
                start_on_io=None,
                pause_on_io=None,
            )
        )

        return future

    def forward_to(
        self, location: float, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        """Move forward to a specific location on the trajectory.

        Args:
            location: Target location to move to (must be >= current location).
            playback_speed_in_percent: Optional speed override (1-100).

        Returns:
            Future that resolves with OperationResult when target is reached.
            If location is before current position, returns a failed future.
        """
        if location < self._current_location:
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_exception(
                ValueError("Cannot move forward to a location before the current location")
            )
            return future
        self._target_location = location
        return self.forward(
            target_location=location, playback_speed_in_percent=playback_speed_in_percent
        )

    def backward_to(
        self, location: float, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        """Move backward to a specific location on the trajectory.

        Args:
            location: Target location to move to (must be <= current location).
            playback_speed_in_percent: Optional speed override (1-100).

        Returns:
            Future that resolves with OperationResult when target is reached.
            If location is after current position, returns a failed future.
        """
        if location > self._current_location:
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_exception(
                ValueError("Cannot move backward to a location after the current location")
            )
            return future
        self._target_location = location
        return self.backward(location, playback_speed_in_percent=playback_speed_in_percent)

    def forward_to_next_action(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        """Move forward to the start of the next action.

        Useful for stepping through a trajectory one action at a time.
        If already at an action boundary, moves to the next action.
        If at the end of the trajectory, returns immediately with current location.

        Args:
            playback_speed_in_percent: Optional speed override (1-100).

        Returns:
            Future that resolves with OperationResult when next action start is reached.
        """
        target_location = self.next_action_start
        if self._current_location == target_location:
            target_location += 1.0
        if target_location > len(self.actions):
            # End of trajectory reached - return immediately
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_result(
                OperationResult(
                    final_location=self._current_location,
                    operation_type=OperationType.FORWARD_TO_NEXT_ACTION,
                )
            )
            return future
        return self.forward_to(target_location, playback_speed_in_percent=playback_speed_in_percent)

    def backward_to_previous_action(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        """Move backward to the start of the previous action.

        Useful for stepping backward through a trajectory one action at a time.
        If within an action, moves to the start of that action first.
        If at the start of the trajectory, returns immediately with current location.

        Args:
            playback_speed_in_percent: Optional speed override (1-100).

        Returns:
            Future that resolves with OperationResult when previous action start is reached.
        """
        target_location = (
            self.previous_action_start
            if self._current_location - self.previous_action_start <= 1.0
            else self.current_action_start
        )
        if target_location >= 0:
            return self.backward_to(
                target_location, playback_speed_in_percent=playback_speed_in_percent
            )
        else:
            # At start of trajectory - return immediately
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_result(
                OperationResult(
                    final_location=self._current_location,
                    operation_type=OperationType.BACKWARD_TO_PREVIOUS_ACTION,
                )
            )
            return future

    def pause(self) -> asyncio.Future[OperationResult] | None:
        """Pause the current movement operation.

        Sends a pause command to stop the robot at its current position.
        Has no effect if no operation is currently in progress.

        Returns:
            Future that resolves when the robot has stopped, or None if no operation is active.
        """
        if not self._is_operation_in_progress():
            return None

        future = self._start_operation(
            OperationType.PAUSE, expected_response_type=api.models.PauseMovementResponse
        )
        self._command_queue.put_nowait(api.models.PauseMovementRequest())
        return future

    def detach(self):
        """Detach from the trajectory, stopping control but not necessarily movement.

        This signals the cursor to stop processing commands and state updates.
        Note: This does not guarantee the robot will stop moving immediately.
        """
        self._command_queue.put_nowait(_QUEUE_SENTINEL)
        self._in_queue.put_nowait(_QUEUE_SENTINEL)

    def _start_operation(
        self,
        operation_type: OperationType,
        *,
        expected_response_type: ExpectedResponseType,
        target_location: Optional[float] = None,
    ) -> asyncio.Future[OperationResult]:
        """Start a new operation, returning a Future that will be resolved when the operation completes."""
        return self._operation_handler.start(
            operation_type,
            start_location=self._current_location,
            expected_response_type=expected_response_type,
            target_location=target_location,
        )

    def _complete_operation(self, error: Optional[Exception] = None):
        """Complete the current operation with the given status."""
        self._operation_handler.complete(final_location=self._current_location, error=error)

    def _is_operation_in_progress(self) -> bool:
        return self._operation_handler.in_progress()

    async def cntrl(
        self, response_stream: ExecuteTrajectoryResponseStream
    ) -> ExecuteTrajectoryRequestStream:
        """Main control loop that manages bidirectional communication with the motion controller.

        This async generator handles the protocol for trajectory execution:
        1. Initializes movement with the motion controller
        2. Spawns background tasks for state monitoring and response processing
        3. Yields movement commands from the command queue
        4. Cleans up on completion or error

        Args:
            response_stream: Async iterator of responses from the motion controller.

        Yields:
            Movement request commands to send to the motion controller.

        Raises:
            RuntimeError: If state monitor fails to start within timeout.
        """
        await self._initialize_task

        self._response_stream = response_stream
        async for request in init_movement_gen(
            self.motion_id, response_stream, self._current_location
        ):
            yield request

        motion_group_state_monitor_ready_event = asyncio.Event()
        response_consumer_ready_event = asyncio.Event()
        try:
            async with asyncio.TaskGroup() as tg:
                motion_group_state_monitor_task = tg.create_task(
                    self._motion_group_state_monitor(
                        ready_event=motion_group_state_monitor_ready_event
                    ),
                    name="motion_group_state_monitor",
                )
                response_consumer_task = tg.create_task(
                    self._response_consumer(ready_event=response_consumer_ready_event),
                    name="response_consumer",
                )
                motion_event_updater_task = tg.create_task(
                    self._motion_event_updater(), name="motion_event_updater"
                )
                # The timeout handling here is very defensive programming to avoid silent hangs
                # in case the connection to the API is lost or similar issues occur.
                # It might be overkill but is useful during development and debugging.
                try:
                    await asyncio.wait_for(
                        motion_group_state_monitor_ready_event.wait(),
                        timeout=_STREAM_STARTUP_TIMEOUT,
                    )
                except asyncio.TimeoutError:
                    logger.error(
                        "TrajectoryCursor motion group state monitor failed to start in time"
                    )
                    motion_group_state_monitor_task.cancel()
                    raise RuntimeError("State monitor failed to start in time")

                await response_consumer_ready_event.wait()

                async for request in self._request_loop():
                    yield request

                motion_event_updater_task.cancel()
                response_consumer_task.cancel()
                motion_group_state_monitor_task.cancel()
        except BaseExceptionGroup as eg:
            logger.exception(eg)
            raise
        except asyncio.CancelledError:
            logger.debug("TrajectoryCursor cntrl was cancelled during cleanup of internal tasks")
            raise

        # stopping the external response stream iterator to be sure, but this is a smell
        self._in_queue.put_nowait(_QUEUE_SENTINEL)

    async def _request_loop(self) -> ExecuteTrajectoryRequestStream:
        while True:
            command = await self._command_queue.get()
            if command is _QUEUE_SENTINEL:
                self._command_queue.task_done()
                break
            ic(command)
            assert isinstance(command, ExecuteTrajectoryRequestCommand)
            yield command

            if isinstance(command, api.models.StartMovementRequest):
                match command.direction:
                    case api.models.Direction.DIRECTION_FORWARD:
                        target_action = self.current_action
                        await motion_started.send_async(
                            self, event=self._get_motion_event(target_action)
                        )
                    case api.models.Direction.DIRECTION_BACKWARD:
                        target_action = self.current_action
                        await motion_started.send_async(
                            self, event=self._get_motion_event(target_action)
                        )

            # yield await self._command_queue.get()
            self._command_queue.task_done()

    async def _motion_group_state_monitor(self, ready_event: asyncio.Event):
        """Monitor motion group state and update operation status accordingly.

        Processes state updates from the motion controller to track operation progress,
        detect completion (standstill), and update current location.

        Args:
            ready_event: Event to signal when the monitor is ready to receive states.
        """
        logger.debug("Starting state monitor for trajectory cursor")
        try:
            async for motion_group_state in self._motion_group_state_stream:
                ready_event.set()

                if not self._is_operation_in_progress():
                    # We only care about motion group states if there is an operation in progress
                    # assert that we are the sole reason for movement
                    assert not motion_group_state.execute
                    continue
                else:
                    current_op = self._operation_handler.current_operation
                    assert current_op is not None
                    if not motion_group_state.execute and current_op.operation_state in (
                        OperationState.INITIAL,
                        OperationState.COMMANDED,
                    ):  #
                        continue  # wait for the execution to actually start

                    if not motion_group_state.execute and motion_group_state.standstill:
                        assert current_op.operation_state not in (
                            OperationState.INITIAL,
                            OperationState.COMMANDED,
                        ), (
                            f"Unexpected operation state {current_op.operation_state} when standstill is True"
                        )
                        self._complete_operation()
                        if self._detach_on_standstill:
                            ic()
                            break

                    assert motion_group_state.execute
                    # TODO it is questionable if we want to maintain the semantics of yield motion group states during
                    # execution this is the only reason we do this here
                    self._in_queue.put_nowait(motion_group_state)

                    if motion_group_state.execute and isinstance(
                        motion_group_state.execute.details, api.models.TrajectoryDetails
                    ):
                        self._current_location = motion_group_state.execute.details.location.root
                        match motion_group_state.execute.details.state:
                            case api.models.TrajectoryRunning():
                                self._operation_handler.set_running()  # idempotent
                            case api.models.TrajectoryPausedByUser():
                                self._complete_operation()
                                if self._detach_on_standstill:
                                    break
                            case api.models.TrajectoryEnded():
                                self._complete_operation()
                                if self._detach_on_standstill:
                                    break
                            case _:
                                assert False, (
                                    f"Unexpected or unsupported motion group execute state: {motion_group_state.execute.details.state}"
                                )
        except asyncio.CancelledError:
            logger.debug("TrajectoryCursor motion group state monitor was cancelled")
            raise
        finally:
            # stop the request loop
            self.detach()
            # stop the cursor iterator (TODO is this the right place?)
            self._in_queue.put_nowait(_QUEUE_SENTINEL)

    async def _response_consumer(self, ready_event: asyncio.Event):
        """Process responses from the motion controller and update operation state.

        Handles response messages including movement confirmations, errors, and
        playback speed acknowledgments.

        Args:
            ready_event: Event to signal when the consumer is ready.
        """
        logger.debug("Starting response consumer for trajectory cursor")
        ready_event.set()
        try:
            async for response in self._response_stream:
                logger.debug(f"Received response: {response}")
                assert self._is_operation_in_progress()
                current_op = self._operation_handler.current_operation
                assert current_op is not None

                match response.root:
                    case api.models.PlaybackSpeedResponse():
                        pass  # no-op for now
                    case api.models.MovementErrorResponse():
                        # TODO do we want this to fail the operation? Maybe you could still continue using the cursor?
                        raise Exception(
                            f"Movement error received in trajectory cursor: {response.root.message}"
                        )
                    case api.models.StartMovementResponse() | api.models.PauseMovementResponse():
                        if isinstance(response.root, current_op.expected_response_type):
                            self._operation_handler.set_commanded()
                    case _:
                        raise RuntimeError(
                            f"Unexpected response in trajectory cursor response consumer: {type(response.root)}, "
                            f"expected {current_op.expected_response_type.__name__}"
                        )
        except asyncio.CancelledError:
            logger.debug("TrajectoryCursor response consumer was cancelled")
            raise

    async def _motion_event_updater(self, interval=0.2):
        """Periodically emit motion events during active movement.

        Args:
            interval: Time in seconds between event emissions (default 0.2s).
        """
        while True:
            current_op = self._operation_handler.current_operation
            op_type = current_op.operation_type if current_op else None
            match op_type:
                case OperationType.FORWARD | OperationType.FORWARD_TO:
                    target_action = self.next_action if self.next_action else self.current_action
                    await motion_started.send_async(
                        self, event=self._get_motion_event(target_action)
                    )
                case OperationType.BACKWARD | OperationType.BACKWARD_TO:
                    target_action = (
                        self.previous_action if self.previous_action else self.current_action
                    )
                    await motion_started.send_async(
                        self, event=self._get_motion_event(target_action)
                    )
                case _:
                    pass
            await asyncio.sleep(interval)

    def _get_motion_event(self, target_action: Action) -> MotionEvent:
        """Create a MotionEvent with current cursor state."""
        return MotionEvent(
            type=MotionEventType.STARTED,
            current_location=self._current_location,
            current_action=self.current_action,
            target_location=self._target_location,
            target_action=target_action,
        )

    def __aiter__(self) -> AsyncIterator[api.models.MotionGroupState]:
        """Return self as an async iterator for motion group states."""
        return self

    async def __anext__(self) -> api.models.MotionGroupState:
        """Yield the next motion group state from the internal queue.

        Raises:
            StopAsyncIteration: When the cursor has been detached.
        """
        value = await self._in_queue.get()
        self._in_queue.task_done()
        if isinstance(value, _QueueSentinel):
            raise StopAsyncIteration
        return value


async def init_movement_gen(
    motion_id, response_stream, initial_location
) -> ExecuteTrajectoryRequestStream:
    """Initialize movement on a trajectory with the motion controller.

    This async generator handles the initialization handshake:
    1. Sends an InitializeMovementRequest with the trajectory ID and start location
    2. Waits for and validates the InitializeMovementResponse

    Args:
        motion_id: Unique identifier for the trajectory to execute.
        response_stream: Async iterator of responses from the motion controller.
        initial_location: Starting position on the trajectory.

    Yields:
        The initialization request to send to the motion controller.

    Raises:
        InitMovementFailed: If the motion controller rejects the initialization.
    """
    trajectory_id = api.models.TrajectoryId(id=motion_id)
    init_request = api.models.InitializeMovementRequest(
        trajectory=trajectory_id, initial_location=initial_location
    )
    yield init_request

    execute_trajectory_response = await anext(response_stream)
    initialize_movement_response = execute_trajectory_response.root
    assert isinstance(initialize_movement_response, api.models.InitializeMovementResponse)
    # TODO this should actually check for None but currently the API seems to return an empty string instead
    # create issue with the API to fix this
    if initialize_movement_response.message or initialize_movement_response.add_trajectory_error:
        raise InitMovementFailed(initialize_movement_response)
