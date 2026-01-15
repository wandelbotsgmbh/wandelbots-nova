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
    FORWARD = auto()
    BACKWARD = auto()
    FORWARD_TO = auto()
    BACKWARD_TO = auto()
    FORWARD_TO_NEXT_ACTION = auto()
    BACKWARD_TO_PREVIOUS_ACTION = auto()
    PAUSE = auto()


class OperationState(Enum):
    INITIAL = auto()
    COMMANDED = auto()
    RUNNING = auto()  # standstill switched to True
    COMPLETED = auto()  # standstill switched to False
    FAILED = auto()  # E-STOP? what else?
    CANCELLED = auto()


@dataclass
class OperationResult:
    operation_type: OperationType
    # status: OperationStatus
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
    """Encapsulates all state for a single movement operation."""

    future: asyncio.Future[OperationResult]
    operation_type: OperationType
    operation_state: OperationState
    start_location: float
    expected_response_type: ExpectedResponseType
    target_location: Optional[float] = None
    interrupt_requested: bool = False


class OperationHandler:
    """Encapsulates the bookkeeping for a single movement operation."""

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
        return self._operation is not None and not self._operation.future.done()

    @property
    def current_operation(self) -> Optional[Operation]:
        return self._operation

    def _reset(self):
        self._operation = None


class MovementOption(StrEnum):
    CAN_MOVE_FORWARD = auto()
    CAN_MOVE_BACKWARD = auto()


motion_started = signal("motion_started")
motion_stopped = signal("motion_stopped")


class MotionEventType(StrEnum):
    STARTED = auto()
    STOPPED = auto()


class MotionEvent(pydantic.BaseModel):
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
    """Marker type used only as a sentinel."""


# The single sentinel value
_QUEUE_SENTINEL = _QueueSentinel()


class TrajectoryCursor:
    def __init__(
        self,
        motion_id: str,
        motion_group_state_stream: AsyncIterator[api.models.MotionGroupState],  # TODO ownership?
        joint_trajectory: api.models.JointTrajectory,
        actions: list[Action],
        initial_location: float,
        detach_on_standstill: bool = False,
    ):
        self.motion_id = motion_id
        self.joint_trajectory = joint_trajectory
        self.actions = CombinedActions(items=actions)  # type: ignore
        self._command_queue: asyncio.Queue[ExecuteTrajectoryRequestCommand | _QueueSentinel] = (
            asyncio.Queue()
        )
        self._in_queue: asyncio.Queue[
            api.models.ExecuteTrajectoryResponse | api.models.MotionGroupState | _QueueSentinel
        ] = asyncio.Queue()
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
        target_action = self.current_action
        # TODO maybe we want a dedicated event for initialization?
        await motion_started.send_async(self, event=self._get_motion_event(target_action))

    @property
    def end_location(self) -> float:
        # The assert ensures that we not accidentally use it in __init__ before it is set.
        assert getattr(self, "joint_trajectory", None) is not None
        return self.joint_trajectory.locations[-1].root

    @property
    def current_action_start(self) -> float:
        return float(floor(self._current_location))

    @property
    def current_action_end(self) -> float:
        return float(ceil(self._current_location))

    @property
    def next_action_start(self) -> float:
        return self.current_action_end

    @property
    def previous_action_start(self) -> float:
        return self.current_action_start - 1.0

    @property
    def current_action_index(self) -> int:
        index = floor(self._current_location)
        if index >= len(self.actions):
            # at the end of the trajectory current action remains the last one
            return len(self.actions) - 1
        return index

    @property
    def current_action(self) -> Action:
        return self.actions[self.current_action_index]

    @property
    def next_action(self) -> Action | None:
        next_action_index = self.current_action_index + 1
        if next_action_index >= len(self.actions):
            return None
        return self.actions[next_action_index]

    @property
    def previous_action(self) -> Action | None:
        previous_action_index = self.current_action_index - 1
        if previous_action_index < 0:
            return None
        return self.actions[previous_action_index]

    def get_movement_options(self) -> set[MovementOption]:
        options: dict[MovementOption, bool] = {
            MovementOption.CAN_MOVE_FORWARD: self._current_location < self.end_location,
            MovementOption.CAN_MOVE_BACKWARD: self._current_location > 0.0,
        }
        return {option for option, available in options.items() if available}

    def forward(
        self, target_location: float | None = None, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        # should idempotently move forward
        future = self._start_operation(
            OperationType.FORWARD, expected_response_type=api.models.StartMovementResponse
        )

        # We can only use this with v2
        # TODO what is this for?
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
                # set_ios=self.context.combined_actions.to_set_io(),  # somehow gets called before self.context is set
                start_on_io=None,
                pause_on_io=None,
            )
        )

        return future

    def backward(
        self, target_location: float | None = None, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
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
                # set_ios=self.context.combined_actions.to_set_io(),
                start_on_io=None,
                pause_on_io=None,
            )
        )

        return future

    def forward_to(
        self, location: float, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        # currently we don't complain about invalid locations as long as they are not before the current location
        if location < self._current_location:
            # Create a future that's already resolved with an error
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
        # currently we don't complain about invalid locations as long as they are not after the current location
        if location > self._current_location:
            # Create a future that's already resolved with an error
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
        target_location = self.next_action_start
        if self._current_location == target_location:
            target_location += 1.0
        if target_location > len(self.actions):
            # TODO decide if this should be an error instead
            # End of trajectory reached
            # Create a future that's already resolved with a preset result
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_result(
                OperationResult(
                    final_location=self._current_location,
                    operation_type=OperationType.FORWARD_TO_NEXT_ACTION,
                )
            )
            # future.set_exception(ValueError("No next action found after current location."))
            return future
        return self.forward_to(target_location, playback_speed_in_percent=playback_speed_in_percent)

    def backward_to_previous_action(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
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
            # Create a future that's already resolved with an error
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_result(
                OperationResult(
                    final_location=self._current_location,
                    operation_type=OperationType.BACKWARD_TO_PREVIOUS_ACTION,
                )
            )
            # future.set_exception(ValueError("No previous action found before current location."))
            return future

    def pause(self) -> asyncio.Future[OperationResult] | None:
        # TODO should the RAE not also handle that? It seems it does not. So we do it here.
        if not self._is_operation_in_progress():
            return

        future = self._start_operation(
            OperationType.PAUSE, expected_response_type=api.models.PauseMovementResponse
        )
        self._command_queue.put_nowait(api.models.PauseMovementRequest())
        return future

    def detach(self):
        # TODO this does not stop the movement atm, it just stops controlling movement
        # this is especially open for testing what happens when the movement is backwards
        # and no end is reached
        # TODO only allow finishing if at forward end of trajectory?
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
                # TODO do we need to wait for the response consumer?
                try:
                    await asyncio.wait_for(
                        response_consumer_ready_event.wait(), timeout=_STREAM_STARTUP_TIMEOUT
                    )
                except asyncio.TimeoutError:
                    logger.error("TrajectoryCursor response consumer failed to start in time")
                    response_consumer_task.cancel()
                    raise RuntimeError("Response consumer failed to start in time")

                async for request in self._request_loop():
                    yield request

                response_consumer_task.cancel()
                motion_event_updater_task.cancel()
                motion_group_state_monitor_task.cancel()
        except ExceptionGroup as eg:
            ic(eg)
            logger.error(f"ExceptionGroup in TrajectoryCursor cntrl: {eg}")
            logger.exception(eg)
            raise
        except BaseExceptionGroup as eg:
            ic(eg)
            logger.error(f"BaseExceptionGroup in TrajectoryCursor cntrl: {eg}")
            raise
        except asyncio.CancelledError:
            ic()
            logger.debug("TrajectoryCursor cntrl was cancelled during cleanup of internal tasks")
            raise
        except Exception as e:
            ic(e)
            logger.error(f"Exception in TrajectoryCursor cntrl: {e}")
            raise
        finally:
            ic()

        # stopping the external response stream iterator to be sure, but this is a smell
        self._in_queue.put_nowait(_QUEUE_SENTINEL)

    async def _request_loop(self):
        while True:
            command = await self._command_queue.get()
            if command is _QUEUE_SENTINEL:
                self._command_queue.task_done()
                break
            ic(command)
            yield command
            # TODO maybe set the operation as commanded here?

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
        logger.debug("Starting response consumer for trajectory cursor")
        ready_event.set()
        try:
            async for response in self._response_stream:
                # TODO log response instead of ic?
                ic(response)
                # TODO handle response properly
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
                            ic()
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
        return MotionEvent(
            type=MotionEventType.STARTED,
            current_location=self._current_location,
            current_action=self.current_action,
            target_location=self._target_location,
            target_action=target_action,
        )

    # TODO currently we only enque MotionGroupState, but we might want to enqueue ExecuteTrajectoryResponse as well
    def __aiter__(
        self,
    ) -> AsyncIterator[api.models.ExecuteTrajectoryResponse | api.models.MotionGroupState]:
        return self

    # TODO currently we only enque MotionGroupState, but we might want to enqueue ExecuteTrajectoryResponse as well
    async def __anext__(self) -> api.models.ExecuteTrajectoryResponse | api.models.MotionGroupState:
        value = await self._in_queue.get()
        self._in_queue.task_done()
        match value:
            case _QueueSentinel():
                raise StopAsyncIteration
            case _:
                return value


async def init_movement_gen(
    motion_id, response_stream, initial_location
) -> ExecuteTrajectoryRequestStream:
    # The first request is to initialize the movement
    trajectory_id = api.models.TrajectoryId(id=motion_id)
    init_request = api.models.InitializeMovementRequest(
        trajectory=trajectory_id, initial_location=initial_location
    )
    yield init_request  # type: ignore

    execute_trajectory_response = await anext(response_stream)
    initialize_movement_response = execute_trajectory_response.root
    assert isinstance(initialize_movement_response, api.models.InitializeMovementResponse)
    # TODO this should actually check for None but currently the API seems to return an empty string instead
    # create issue with the API to fix this
    if initialize_movement_response.message or initialize_movement_response.add_trajectory_error:
        raise InitMovementFailed(initialize_movement_response)
