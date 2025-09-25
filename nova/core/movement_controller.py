import asyncio
from enum import StrEnum, auto
from functools import singledispatch
from math import ceil, floor
from typing import Any, AsyncIterator, Optional

import pydantic
import wandelbots_api_client as wb
from aiohttp_retry import dataclass
from blinker import signal

from nova.actions import MovementControllerContext
from nova.actions.base import Action
from nova.actions.container import CombinedActions
from nova.core import logger
from nova.core.exceptions import InitMovementFailed
from nova.types import (
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MotionState,
    MovementControllerFunction,
    Pose,
    RobotState,
)


@singledispatch
def movement_to_motion_state(movement: Any) -> MotionState:
    raise NotImplementedError(f"Unsupported movement type: {type(movement)}")


@movement_to_motion_state.register
def _(movement: wb.models.Movement) -> MotionState:
    """Convert a wb.models.Movement to a MotionState."""
    if (
        movement.movement.state is None
        or movement.movement.current_location is None
        or len(movement.movement.state.motion_groups) == 0
    ):
        assert False, "This should not happen"  # depending on NC-1105

    # TODO: in which cases do we have more than one motion group here?
    motion_group = movement.movement.state.motion_groups[0]
    return motion_group_state_to_motion_state(
        motion_group, float(movement.movement.current_location)
    )


@movement_to_motion_state.register
def _(movement: wb.models.StreamMoveResponse) -> MotionState:
    """Convert a wb.models.Movement to a MotionState."""
    if (
        movement.move_response is None
        or movement.state is None
        or movement.move_response.current_location_on_trajectory is None
        or len(movement.state.motion_groups) == 0
    ):
        assert False, "This should not happen"  # depending on NC-1105

    # TODO: in which cases do we have more than one motion group here?
    motion_group = movement.state.motion_groups[0]
    return motion_group_state_to_motion_state(
        motion_group, float(movement.move_response.current_location_on_trajectory)
    )


def motion_group_state_to_motion_state(
    motion_group_state: wb.models.MotionGroupState, path_parameter: float
) -> MotionState:
    tcp_pose = Pose(motion_group_state.tcp_pose)
    joints = (
        tuple(motion_group_state.joint_current.joints) if motion_group_state.joint_current else None
    )
    return MotionState(
        motion_group_id=motion_group_state.motion_group,
        path_parameter=path_parameter,
        state=RobotState(pose=tcp_pose, joints=joints),
    )


def move_forward(context: MovementControllerContext) -> MovementControllerFunction:
    """
    movement_controller is an async function that yields requests to the server.
    If a movement_consumer is provided, we'll asend() each wb.models.MovementMovement to it,
    letting it produce MotionState objects.
    """

    async def movement_controller(
        response_stream: ExecuteTrajectoryResponseStream,
    ) -> ExecuteTrajectoryRequestStream:
        # The first request is to initialize the movement
        yield wb.models.InitializeMovementRequest(trajectory=context.motion_id, initial_location=0)  # type: ignore

        # then we get the response
        initialize_movement_response = await anext(response_stream)
        if isinstance(
            initialize_movement_response.actual_instance, wb.models.InitializeMovementResponse
        ):
            r1 = initialize_movement_response.actual_instance
            if not r1.init_response.succeeded:
                raise InitMovementFailed(r1.init_response)

        # The second request is to start the movement
        set_io_list = context.combined_actions.to_set_io()
        yield wb.models.StartMovementRequest(  # type: ignore
            set_ios=set_io_list, start_on_io=None, pause_on_io=None
        )

        # then we wait until the movement is finished
        async for execute_trajectory_response in response_stream:
            instance = execute_trajectory_response.actual_instance
            # Stop when standstill indicates motion ended
            if isinstance(instance, wb.models.Standstill):
                if instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                    return

    return movement_controller


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
