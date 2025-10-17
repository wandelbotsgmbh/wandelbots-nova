import asyncio
from enum import StrEnum, auto
from math import ceil, floor
from typing import AsyncIterator, Optional

import pydantic
import wandelbots_api_client.v2 as wb
from aiohttp_retry import dataclass
from blinker import signal

from nova.actions import MovementControllerContext
from nova.actions.base import Action
from nova.actions.container import CombinedActions
from nova.core import logger
from nova.core.exceptions import ErrorDuringMovement, InitMovementFailed
from nova.types import (
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MotionState,
    MovementControllerFunction,
)
from nova.types.state import motion_group_state_to_motion_state

ExecuteJoggingRequestStream = AsyncIterator[wb.models.ExecuteJoggingRequest]
ExecuteJoggingResponseStream = AsyncIterator[wb.models.ExecuteJoggingResponse]


# TODO: when the message exchange is not working as expected we should gracefully close
def move_forward(context: MovementControllerContext) -> MovementControllerFunction:
    """
    movement_controller is an async function that yields requests to the server.
    If a movement_consumer is provided, we'll asend() each wb.models.MovementMovement to it,
    letting it produce MotionState objects.
    """

    async def movement_controller(
        response_stream: ExecuteTrajectoryResponseStream,
    ) -> ExecuteTrajectoryRequestStream:
        # TODO task error handling

        async def motion_group_state_monitor(
            motion_group_state_stream: AsyncIterator[wb.models.MotionGroupState],
            ready: asyncio.Event,
        ) -> AsyncIterator[MotionState]:
            ready.set()
            async for motion_group_state in motion_group_state_stream:
                if motion_group_state.execute and isinstance(
                    motion_group_state.execute.details.actual_instance.state.actual_instance,
                    wb.models.TrajectoryEnded,
                ):
                    await error_monitor_task_created.wait()
                    error_monitor_task.cancel()
                    break

        async def error_monitor(
            responses: ExecuteTrajectoryResponseStream, to_cancel: list[asyncio.Task]
        ):
            async for execute_trajectory_response in responses:
                instance = execute_trajectory_response.actual_instance
                if isinstance(instance, wb.models.MovementErrorResponse):
                    for task in to_cancel:
                        task.cancel()
                    # TODO how does this propagate?
                    # TODO what happens to the state consumer?
                    raise ErrorDuringMovement(instance.message)

        motion_group_state_stream = context.motion_group_state_stream_gen()
        motion_group_state_monitor_ready = asyncio.Event()
        error_monitor_task_created = asyncio.Event()
        motion_group_state_monitor_task = asyncio.create_task(
            motion_group_state_monitor(motion_group_state_stream, motion_group_state_monitor_ready),
            name="state_consumer",
        )

        await motion_group_state_monitor_ready.wait()
        yield wb.models.InitializeMovementRequest(
            trajectory=wb.models.InitializeMovementRequestTrajectory(
                wb.models.TrajectoryId(id=context.motion_id)
            ),
            initial_location=0,
        )
        execute_trajectory_response = await anext(response_stream)
        initialize_movement_response = execute_trajectory_response.actual_instance
        assert isinstance(initialize_movement_response, wb.models.InitializeMovementResponse)
        # TODO this should actually check for None but currently the API seems to return an empty string instead
        # create issue with the API to fix this
        if (
            initialize_movement_response.message
            or initialize_movement_response.add_trajectory_error
        ):
            raise InitMovementFailed(initialize_movement_response)

        set_io_list = context.combined_actions.to_set_io()
        yield wb.models.StartMovementRequest(
            direction=wb.models.Direction.DIRECTION_FORWARD,
            set_ios=set_io_list,
            start_on_io=None,
            pause_on_io=None,
        )
        execute_trajectory_response = await anext(response_stream)
        start_movement_response = execute_trajectory_response.actual_instance
        assert isinstance(start_movement_response, wb.models.StartMovementResponse)

        error_monitor_task = asyncio.create_task(
            error_monitor(response_stream, [motion_group_state_monitor_task]), name="error_monitor"
        )
        error_monitor_task_created.set()
        try:
            await error_monitor_task
        except asyncio.CancelledError:
            ic()
        await motion_group_state_monitor_task

    return movement_controller


class TrajectoryCursor:
    def __init__(
        self,
        effect_stream: AsyncIterator[wb.models.MotionGroupState],
        joint_trajectory: wb.models.JointTrajectory,
    ):
        self.joint_trajectory = joint_trajectory
        self._command_queue = asyncio.Queue()
        self._response_queue = asyncio.Queue()
        self._effect_stream: AsyncIterator[wb.models.MotionGroupState] = effect_stream
        self._breakpoints = []

    def __call__(self, context: MovementControllerContext):
        self.context = context
        return self.cntrl

    def forward(self):
        self._command_queue.put_nowait(
            wb.models.StartMovementRequest(
                direction=wb.models.Direction.DIRECTION_FORWARD,
                # set_ios=self.context.combined_actions.to_set_io(),  # somehow gets called before self.context is set
                start_on_io=None,
                pause_on_io=None,
            )
        )

    def backward(self):
        self._command_queue.put_nowait(
            wb.models.StartMovementRequest(
                direction=wb.models.Direction.DIRECTION_BACKWARD,
                # set_ios=self.context.combined_actions.to_set_io(),
                start_on_io=None,
                pause_on_io=None,
            )
        )

    def pause(self):
        self._command_queue.put_nowait(wb.models.PauseMovementRequest())

    def pause_at(self, location: float):
        # How to pause at an exact location?
        bisect.insort(self._breakpoints, location)

    async def cntrl(
        self, response_stream: ExecuteTrajectoryResponseStream
    ) -> ExecuteTrajectoryRequestStream:
        self._response_stream = response_stream
        async for request in init_movement_gen(self.context.motion_id, response_stream):
            yield request

        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._effect_consumer(), name="effect_consumer")
            tg.create_task(self._response_consumer(), name="request_consumer")
            tg.create_task(self._combined_response_consumer(), name="combined_response_consumer")

            async for request in self._request_loop():
                yield request

    async def _request_loop(self):
        while True:
            yield await self._command_queue.get()
            self._command_queue.task_done()

    async def _effect_consumer(self):
        async for effect in self._effect_stream:
            # ic(effect)
            self._response_queue.put_nowait(effect)

    async def _response_consumer(self):
        async for response in self._response_stream:
            ic(response)
            self._response_queue.put_nowait(response)

    async def _combined_response_consumer(self):
        last_motion_state = None

        try:
            while True:
                response = await self._response_queue.get()
                # ic(response)
                if isinstance(response, wb.models.MotionGroupState):
                    motion_group_state = response
                    if motion_group_state.execute and isinstance(
                        motion_group_state.execute.details.actual_instance.state.actual_instance,
                        wb.models.TrajectoryEnded,
                    ):
                        # TODO this is not always the intended end of the movement, but how do we know?
                        ic()
                        break

                    motion_state = motion_group_state_to_motion_state(response)
                    if last_motion_state is None:
                        last_motion_state = motion_state
                    self._handle_motion_state(motion_state, last_motion_state)
                    last_motion_state = motion_state
                    self._response_queue.task_done()
                elif isinstance(response, wb.models.ExecuteTrajectoryResponse):
                    if isinstance(response.actual_instance, wb.models.MovementErrorResponse):
                        ic()
                        self._response_queue.task_done()
                        # TODO propagate the error
                        break
                else:
                    ic(f"Unexpected response type: {type(response)}")
                    self._response_queue.task_done()
        except asyncio.CancelledError:
            ic()
            raise
        except Exception as e:
            ic(e)
            raise
        ic()

    def _handle_motion_state(self, curr_motion_state: MotionState, last_motion_state: MotionState):
        if not self._breakpoints:
            return
        curr_location = curr_motion_state.path_parameter
        last_location = last_motion_state.path_parameter
        if curr_location == last_location:
            return
        if curr_location > last_location:
            # moving forwards
            for breakpoint in self._breakpoints:
                if last_location <= breakpoint < curr_location:
                    ic(last_location, breakpoint, curr_location)
                    self.pause()
                    # disable the breakpoint so it doesn't trigger again for the location window
        else:
            # moving backwards
            for breakpoint in self._breakpoints:
                if last_location > breakpoint >= curr_location:
                    ic()
                    self.pause()
                    # disable the breakpoint so it doesn't trigger again for the location window


async def init_movement_gen(motion_id, response_stream) -> ExecuteTrajectoryRequestStream:
    # The first request is to initialize the movement
    yield wb.models.InitializeMovementRequest(
        trajectory=wb.models.InitializeMovementRequestTrajectory(
            wb.models.TrajectoryId(id=motion_id)
        ),
        initial_location=0,
    )  # type: ignore

    # then we get the response
    execute_trajectory_response = await anext(response_stream)
    initialize_movement_response = execute_trajectory_response.actual_instance
    ic(initialize_movement_response)
    assert isinstance(initialize_movement_response, wb.models.InitializeMovementResponse)
    ic()
    # TODO this should actually check for None but currently the API seems to return an empty string instead
    if initialize_movement_response.message or initialize_movement_response.add_trajectory_error:
        raise InitMovementFailed(initialize_movement_response)

    # assert isinstance(
    #     initialize_movement_response.actual_instance, wb.models.InitializeMovementResponse
    # ), "Expected InitializeMovementResponse, got: " + str(
    #     initialize_movement_response.actual_instance
    # )
    # if isinstance(
    #     initialize_movement_response.actual_instance, wb.models.InitializeMovementResponse
    # ):
    #     r1 = initialize_movement_response.actual_instance
    #     if not r1.init_response.succeeded:
    #         raise InitMovementFailed(r1.init_response)


# TODO do we need this?
def trajectory_cursor(context: MovementControllerContext) -> MovementControllerFunction:
    async def movement_controller(
        response_stream: ExecuteTrajectoryResponseStream,
    ) -> ExecuteTrajectoryRequestStream:
        # The second request is to start the movement
        set_io_list = context.combined_actions.to_set_io()
        yield wb.models.StartMovementRequest(  # type: ignore
            set_ios=set_io_list, start_on_io=context.start_on_io, pause_on_io=None
        )

        # then we wait until the movement is finished
        async for execute_trajectory_response in response_stream:
            instance = execute_trajectory_response.actual_instance
            # Stop when standstill indicates motion ended
            if isinstance(instance, wb.models.Standstill):
                if instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                    return

    return TrajectoryCursor(context)


class OperationType(StrEnum):
    FORWARD = auto()
    BACKWARD = auto()
    FORWARD_TO = auto()
    BACKWARD_TO = auto()
    FORWARD_TO_NEXT_ACTION = auto()
    BACKWARD_TO_PREVIOUS_ACTION = auto()
    PAUSE = auto()


@dataclass
class OperationResult:
    operation_type: OperationType
    # status: OperationStatus
    target_location: Optional[float] = None
    start_location: Optional[float] = None
    final_location: Optional[float] = None
    overshoot: Optional[float] = None
    error: Optional[Exception] = None


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


class TrajectoryCursor:
    def __init__(
        self,
        motion_id: str,
        joint_trajectory: wb.models.JointTrajectory,
        actions: list[Action],
        initial_location: float,
        detach_on_standstill: bool = False,
    ):
        self.motion_id = motion_id
        self.joint_trajectory = joint_trajectory
        self.actions = CombinedActions(items=actions)  # type: ignore
        self._command_queue: asyncio.Queue = asyncio.Queue()
        self._COMMAND_QUEUE_SENTINAL = object()
        self._in_queue: asyncio.Queue[wb.models.ExecuteTrajectoryResponse | None] = asyncio.Queue()
        self._current_location = initial_location
        self._overshoot = 0.0
        self._target_location = self.next_action_index + 1.0
        self._detach_on_standstill = detach_on_standstill

        self._current_operation_future: Optional[asyncio.Future[OperationResult]] = None
        self._current_operation_type: Optional[OperationType] = None
        self._current_target_location: Optional[float] = None
        self._current_start_location: Optional[float] = None

        self._initialize_task = asyncio.create_task(self.ainitialize())

    async def ainitialize(self):
        target_action = self.actions[self.next_action_index or 0]
        await motion_started.send_async(self, event=self._get_motion_event(target_action))

    @property
    def end_location(self) -> float:
        # The assert ensures that we not accidentally use it in __init__ before it is set.
        assert getattr(self, "joint_trajectory", None) is not None
        return self.joint_trajectory.locations[-1]

    @property
    def current_action(self) -> Action:
        current_action_index = floor(self._current_location) - 1
        if current_action_index < 0:
            current_action_index = 0
        return self.actions[current_action_index]

    @property
    def next_action_index(self) -> int:
        index = ceil(self._current_location - self._overshoot) - 1
        if index < 0:
            return 0
        if index < len(self.actions):
            return index
        # for now we don't allow forward wrap around
        return len(self.actions) - 1

    @property
    def previous_action_index(self) -> int:
        index = ceil(self._current_location - 1.0 - self._overshoot) - 1
        assert index <= len(self.actions)
        # if index < 0:
        #     # for now we don't allow backward wrap around
        #     return 0
        return index

    def get_movement_options(self) -> set[MovementOption]:
        options = {
            MovementOption.CAN_MOVE_FORWARD: self._current_location < self.end_location,
            MovementOption.CAN_MOVE_BACKWARD: self._current_location > 0.0,
        }
        return {option for option, available in options.items() if available}

    def forward(
        self, location: float | None = None, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        # should idempotently move forward
        future = self._start_operation(OperationType.FORWARD)

        # We can only use this with v2
        # if location is not None:
        #     self._target_location = location

        if playback_speed_in_percent is not None:
            self._command_queue.put_nowait(
                wb.models.PlaybackSpeedRequest(playback_speed_in_percent=playback_speed_in_percent)
            )
        self._command_queue.put_nowait(
            wb.models.StartMovementRequest(
                direction=wb.models.Direction.DIRECTION_FORWARD,
                # set_ios=self.context.combined_actions.to_set_io(),  # somehow gets called before self.context is set
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
        return self.forward(playback_speed_in_percent=playback_speed_in_percent)

    def forward_to_next_action(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        index = self.next_action_index
        if index < len(self.actions):
            return self.forward_to(index + 1.0, playback_speed_in_percent=playback_speed_in_percent)
        else:
            return asyncio.Future()
            # # Create a future that's already resolved with an error
            # future = asyncio.Future()
            # future.set_exception(ValueError("No next action found after current location."))
            # return future

    def backward(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        future = self._start_operation(OperationType.BACKWARD)

        if playback_speed_in_percent is not None:
            self._command_queue.put_nowait(
                wb.models.PlaybackSpeedRequest(playback_speed_in_percent=playback_speed_in_percent)
            )
        self._command_queue.put_nowait(
            wb.models.StartMovementRequest(
                direction=wb.models.Direction.DIRECTION_BACKWARD,
                # set_ios=self.context.combined_actions.to_set_io(),
                start_on_io=None,
                pause_on_io=None,
            )
        )

        return future

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
        return self.backward(playback_speed_in_percent=playback_speed_in_percent)

    def backward_to_previous_action(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        index = self.previous_action_index
        if index + 1.0 >= 0:
            return self.backward_to(
                index + 1.0, playback_speed_in_percent=playback_speed_in_percent
            )
        else:
            # Create a future that's already resolved with an error
            future: asyncio.Future[OperationResult] = asyncio.Future()
            future.set_exception(ValueError("No previous action found before current location."))
            return future

    def _pause(self):
        self._command_queue.put_nowait(wb.models.PauseMovementRequest())

    def pause(self) -> asyncio.Future[OperationResult] | None:
        self._pause()
        if self._current_operation_future is not None:
            return self._current_operation_future
        return None

    def detach(self):
        # TODO this does not stop the movement atm, it just stops controlling movement
        # this is especially open for testing what happens when the movement is backwards
        # and no end is reached
        # TODO only allow finishing if at forward end of trajectory
        self._command_queue.put_nowait(self._COMMAND_QUEUE_SENTINAL)

    def _start_operation(
        self, operation_type: OperationType, target_location: Optional[float] = None
    ) -> asyncio.Future[OperationResult]:
        """Start a new operation, returning a Future that will be resolved when the operation completes."""
        if self._current_operation_future and not self._current_operation_future.done():
            # Cancel the current operation
            self._current_operation_future.cancel()

        self._current_operation_future = asyncio.Future()
        self._current_operation_type = operation_type
        self._current_target_location = target_location
        self._current_start_location = self._current_location
        self._interrupt_requested = False

        return self._current_operation_future

    def _complete_operation(self, error: Optional[Exception] = None):
        """Complete the current operation with the given status."""
        if not self._current_operation_future or self._current_operation_future.done():
            return

        assert self._current_operation_type is not None

        result = OperationResult(
            operation_type=self._current_operation_type,
            # status=status,
            target_location=self._current_target_location,
            start_location=self._current_start_location,
            final_location=self._current_location,
            overshoot=self._overshoot,
            error=error,
        )

        if error:
            self._current_operation_future.set_exception(error)
        else:
            self._current_operation_future.set_result(result)

    async def cntrl(
        self, response_stream: ExecuteTrajectoryResponseStream
    ) -> ExecuteTrajectoryRequestStream:
        await self._initialize_task

        self._response_stream = response_stream
        async for request in init_movement_gen(
            self.motion_id, response_stream, self._current_location
        ):
            yield request

        respons_consumer_ready_event = asyncio.Event()
        self._response_consumer_task: asyncio.Task[None] = asyncio.create_task(
            self._response_consumer(ready_event=respons_consumer_ready_event),
            name="response_consumer",
        )
        motion_event_updater_task = asyncio.create_task(
            self.motion_event_updater(), name="motion_event_updater"
        )
        # we need to wait until the response consumer is ready because it stops the
        # response stream iterator by enquing the sentinel
        # if we cancel immediately due to the first command being a detach the response consumer might get
        # cancelled before it has even started and thus will not react to the cancellation properly
        await respons_consumer_ready_event.wait()
        async for request in self._request_loop():
            yield request
        self._response_consumer_task.cancel()
        try:
            await self._response_consumer_task  # Where to put?
        except asyncio.CancelledError:
            logger.debug("Response consumer task was cancelled during trajectory cursor cleanup")
            pass
        motion_event_updater_task.cancel()
        try:
            await motion_event_updater_task
        except asyncio.CancelledError:
            logger.debug("Motion event updater task was cancelled during trajectory cursor cleanup")
            pass
        # stopping the external response stream iterator to be sure, but this is a smell
        self._in_queue.put_nowait(None)

    async def _request_loop(self):
        while True:
            command = await self._command_queue.get()
            if command is self._COMMAND_QUEUE_SENTINAL:
                self._command_queue.task_done()
                break
            yield command

            if isinstance(command, wb.models.StartMovementRequest):
                match command.direction:
                    case wb.models.Direction.DIRECTION_FORWARD:
                        target_action = self.actions[self.next_action_index]
                        await motion_started.send_async(
                            self, event=self._get_motion_event(target_action)
                        )
                    case wb.models.Direction.DIRECTION_BACKWARD:
                        target_action = self.actions[self.previous_action_index]
                        await motion_started.send_async(
                            self, event=self._get_motion_event(target_action)
                        )

            # yield await self._command_queue.get()
            self._command_queue.task_done()

    async def _response_consumer(self, ready_event: asyncio.Event):
        ready_event.set()
        last_movement = None

        try:
            async for response in self._response_stream:
                self._in_queue.put_nowait(response)
                instance = response.actual_instance
                if isinstance(instance, wb.models.Movement):
                    if last_movement is None:
                        last_movement = instance.movement
                    self._current_location = instance.movement.current_location
                    self._handle_movement(instance.movement, last_movement)
                    last_movement = instance.movement

                elif isinstance(instance, wb.models.Standstill):
                    self._current_location = instance.standstill.location
                    match instance.standstill.reason:
                        case wb.models.StandstillReason.REASON_USER_PAUSED_MOTION:
                            self._overshoot = self._current_location - self._target_location
                            self._complete_operation()
                            if self._detach_on_standstill:
                                break
                        case wb.models.StandstillReason.REASON_MOTION_ENDED:
                            self._overshoot = self._current_location - self._target_location
                            assert self._overshoot == 0.0
                            # self._overshoot = 0.0  # because the RAE takes care of that
                            self._complete_operation()
                            if self._detach_on_standstill:
                                break
                            # self.context.movement_consumer(None)
                            # break
        except asyncio.CancelledError:
            logger.debug("Response consumer was cancelled, cleaning up trajectory cursor")
            raise
        finally:
            # stop the request loop
            self.detach()
            # stop the cursor iterator (TODO is this the right place?)
            self._in_queue.put_nowait(None)  # TODO make sentinel more explicit

    def _handle_movement(
        self, curr_movement: wb.models.MovementMovement, last_movement: wb.models.MovementMovement
    ):
        if not 0.0 <= self._target_location <= self.end_location:
            return
        curr_location = curr_movement.current_location
        last_location = last_movement.current_location
        if curr_location == last_location:
            return
        if curr_location > last_location:
            # moving forwards
            if last_location <= self._target_location < curr_location:
                self._pause()
        else:
            # moving backwards
            if last_location > self._target_location >= curr_location:
                self._pause()

    async def motion_event_updater(self, interval=0.2):
        while True:
            match self._current_operation_type:
                case OperationType.FORWARD | OperationType.FORWARD_TO:
                    target_action = self.actions[self.next_action_index]
                    await motion_started.send_async(
                        self, event=self._get_motion_event(target_action)
                    )
                case OperationType.BACKWARD | OperationType.BACKWARD_TO:
                    target_action = self.actions[self.previous_action_index]
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

    def __aiter__(self) -> AsyncIterator[wb.models.ExecuteTrajectoryResponse]:
        return self

    async def __anext__(self) -> wb.models.ExecuteTrajectoryResponse:
        value = await self._in_queue.get()
        self._in_queue.task_done()
        if value is None:
            raise StopAsyncIteration
        return value


async def init_movement_gen(
    motion_id, response_stream, initial_location
) -> ExecuteTrajectoryRequestStream:
    # The first request is to initialize the movement
    init_request = wb.models.InitializeMovementRequest(
        trajectory=motion_id, initial_location=initial_location
    )
    yield init_request  # type: ignore

    # then we get the response
    initialize_movement_response = await anext(response_stream)
    # TODO handle error response here (MovementError)
    if isinstance(
        initialize_movement_response.actual_instance, wb.models.InitializeMovementResponse
    ):
        r1 = initialize_movement_response.actual_instance
        if not r1.init_response.succeeded:  # TODO we don't come here if there was an error response
            raise InitMovementFailed(r1.init_response)
