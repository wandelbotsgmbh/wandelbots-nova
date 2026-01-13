import asyncio
import logging
from datetime import datetime
from enum import Enum, StrEnum, auto
from math import ceil, floor
from typing import AsyncIterator, Optional

import pydantic
from aiohttp_retry import dataclass
from blinker import signal
from icecream import ic

from nova import api
from nova.actions.base import Action
from nova.actions.container import CombinedActions
from nova.exceptions import InitMovementFailed
from nova.types.state import motion_group_state_to_motion_state

logger = logging.getLogger(__name__)

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")

_START_STATE_MONITOR_TIMEOUT = 5.0


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


class OperationHandler:
    """Encapsulates the bookkeeping for a single movement operation."""

    def __init__(self):
        self._future: Optional[asyncio.Future[OperationResult]] = None
        self._operation_type: Optional[OperationType] = None
        self._operation_state: Optional[OperationState] = None
        self._target_location: Optional[float] = None
        self._start_location: Optional[float] = None
        self._interrupt_requested = False

    def start(
        self,
        operation_type: OperationType,
        *,
        start_location: float,
        target_location: Optional[float] = None,
    ) -> asyncio.Future[OperationResult]:
        if self._future and not self._future.done():
            self._future.cancel()

        self._future = asyncio.Future()
        self._operation_type = operation_type
        self._operation_state = (
            OperationState.COMMANDED
        )  # TODO this should be initial and commanded when we have send it to the API
        self._target_location = target_location
        self._start_location = start_location
        self._interrupt_requested = False
        return self._future

    def complete(self, *, final_location: float, error: Optional[Exception] = None) -> None:
        if not self._future or self._future.done():
            return

        assert self._operation_type is not None
        result = OperationResult(
            operation_type=self._operation_type,
            target_location=self._target_location,
            start_location=self._start_location,
            final_location=final_location,
            error=error,
        )
        if error:
            self._future.set_exception(error)
        else:
            ic(result)
            self._future.set_result(result)
        self._reset()

    def in_progress(self) -> bool:
        return self._future is not None and not self._future.done()

    @property
    def current_operation_type(self) -> Optional[OperationType]:
        return self._operation_type

    @property
    def current_operation_state(self) -> Optional[OperationState]:
        return self._operation_state

    @property
    def current_future(self) -> Optional[asyncio.Future[OperationResult]]:
        return self._future

    def _reset(self):
        self._future = None
        self._operation_type = None
        self._target_location = None
        self._start_location = None
        self._interrupt_requested = False


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
        motion_group_state_stream: AsyncIterator[api.models.MotionGroupState],  # TODO ownership?
        joint_trajectory: api.models.JointTrajectory,
        actions: list[Action],
        initial_location: float,
        detach_on_standstill: bool = False,
    ):
        self.motion_id = motion_id
        self.joint_trajectory = joint_trajectory
        self.actions = CombinedActions(items=actions)  # type: ignore
        self._command_queue: asyncio.Queue = asyncio.Queue()
        self._COMMAND_QUEUE_SENTINAL = object()
        self._response_queue: asyncio.Queue[
            api.models.ExecuteTrajectoryResponse | api.models.MotionGroupState
        ] = asyncio.Queue()
        self._in_queue: asyncio.Queue[
            api.models.ExecuteTrajectoryResponse | api.models.MotionGroupState | None
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
        future = self._start_operation(OperationType.FORWARD)

        # We can only use this with v2
        # TODO what is this for?
        if target_location is not None:
            self._target_location = target_location

        if playback_speed_in_percent is not None:
            self._command_queue.put_nowait(
                api.models.PlaybackSpeedRequest(playback_speed_in_percent=playback_speed_in_percent)
            )
        ic(target_location, self._target_location)
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
        future = self._start_operation(OperationType.BACKWARD)

        if playback_speed_in_percent is not None:
            self._command_queue.put_nowait(
                api.models.PlaybackSpeedRequest(playback_speed_in_percent=playback_speed_in_percent)
            )
        ic(target_location, self._target_location)
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
        ic(self._target_location)
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
        ic(self._current_location, target_location)
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
        ic(self._current_location, target_location, self.previous_action_start)
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

        future = self._start_operation(OperationType.PAUSE)
        self._command_queue.put_nowait(api.models.PauseMovementRequest())
        return future

    def detach(self):
        # TODO this does not stop the movement atm, it just stops controlling movement
        # this is especially open for testing what happens when the movement is backwards
        # and no end is reached
        # TODO only allow finishing if at forward end of trajectory?
        self._command_queue.put_nowait(self._COMMAND_QUEUE_SENTINAL)
        self._in_queue.put_nowait(None)  # TODO make sentinel more explicit

    def _start_operation(
        self, operation_type: OperationType, target_location: Optional[float] = None
    ) -> asyncio.Future[OperationResult]:
        """Start a new operation, returning a Future that will be resolved when the operation completes."""
        return self._operation_handler.start(
            operation_type, start_location=self._current_location, target_location=target_location
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

        motion_event_updater_ready_event = asyncio.Event()
        response_consumer_ready_event = asyncio.Event()
        combined_response_consumer_ready_event = asyncio.Event()
        try:
            async with asyncio.TaskGroup() as tg:
                motion_group_state_consumer_task = tg.create_task(
                    self._motion_group_state_consumer(ready_event=motion_event_updater_ready_event),
                    name="motion_group_state_consumer",
                )
                response_consumer_task = tg.create_task(
                    self._response_consumer(ready_event=response_consumer_ready_event),
                    name="response_consumer",
                )
                # combined_response_consumer_task = tg.create_task(
                #     self._combined_response_consumer(
                #         ready_event=combined_response_consumer_ready_event
                #     ),
                #     name="combined_response_consumer",
                # )
                motion_event_updater_task = tg.create_task(
                    self._motion_event_updater(), name="motion_event_updater"
                )
                # we need to wait until the response consumer is ready because it stops the
                # response stream iterator by enquing the sentinel
                # if we cancel immediately due to the first command being a detach the response consumer might get
                # cancelled before it has even started and thus will not react to the cancellation properly
                await motion_event_updater_ready_event.wait()
                await response_consumer_ready_event.wait()
                # await combined_response_consumer_ready_event.wait()
                async for request in self._request_loop():
                    yield request

                response_consumer_task.cancel()
                motion_event_updater_task.cancel()
                motion_group_state_consumer_task.cancel()
                # combined_response_consumer_task.cancel()
        except ExceptionGroup as eg:
            ic(eg)
            logger.error(f"ExceptionGroup in TrajectoryCursor cntrl: {eg}")
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

        # # OLD
        # self._response_consumer_task: asyncio.Task[None] = asyncio.create_task(
        #     self._response_consumer(ready_event=respons_consumer_ready_event),
        #     name="response_consumer",
        # )
        # motion_event_updater_task = asyncio.create_task(
        #     self.motion_event_updater(), name="motion_event_updater"
        # )
        # # we need to wait until the response consumer is ready because it stops the
        # # response stream iterator by enquing the sentinel
        # # if we cancel immediately due to the first command being a detach the response consumer might get
        # # cancelled before it has even started and thus will not react to the cancellation properly
        # await respons_consumer_ready_event.wait()
        # async for request in self._request_loop():
        #     yield request
        # self._response_consumer_task.cancel()
        # try:
        #     await self._response_consumer_task  # Where to put?
        # except asyncio.CancelledError:
        #     logger.debug("Response consumer task was cancelled during trajectory cursor cleanup")
        #     pass
        # motion_event_updater_task.cancel()
        # try:
        #     await motion_event_updater_task
        # except asyncio.CancelledError:
        #     logger.debug("Motion event updater task was cancelled during trajectory cursor cleanup")
        #     pass

        ic()
        # stopping the external response stream iterator to be sure, but this is a smell
        self._in_queue.put_nowait(None)

    async def _request_loop(self):
        while True:
            command = await self._command_queue.get()
            if command is self._COMMAND_QUEUE_SENTINAL:
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

    async def _motion_group_state_consumer(self, ready_event: asyncio.Event):
        async for motion_group_state in self._motion_group_state_stream:
            ready_event.set()

            if not self._is_operation_in_progress():
                # We only care about motion group states if there is an operation in progress
                # assert that we are the sole reason for movement
                assert not motion_group_state.execute
                continue
            else:
                if (
                    not motion_group_state.execute
                    and self._operation_handler.current_operation_state == OperationState.COMMANDED
                ):  #
                    continue  # wait for the execution to actually start

                if not motion_group_state.execute and motion_group_state.standstill:
                    ic()
                    assert self._operation_handler.current_operation_state not in (
                        OperationState.INITIAL,
                        OperationState.COMMANDED,
                    ), (
                        f"Unexpected operation state {self._operation_handler.current_operation_state} when standstill is True"
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
                            if (
                                self._operation_handler.current_operation_state
                                == OperationState.COMMANDED
                            ):
                                # State transition: COMMANDED -> RUNNING
                                self._operation_handler._operation_state = OperationState.RUNNING
                        case api.models.TrajectoryPausedByUser():
                            self._complete_operation()
                            if self._detach_on_standstill:
                                break
                        case api.models.TrajectoryEnded():
                            self._complete_operation()
                            if self._detach_on_standstill:
                                break

        # stop the request loop
        self.detach()
        # stop the cursor iterator (TODO is this the right place?)
        ic()
        self._in_queue.put_nowait(None)  # TODO make sentinel more explicit

    async def _response_consumer(self, ready_event: asyncio.Event):
        ready_event.set()
        async for response in self._response_stream:
            ic(response)
            self._response_queue.put_nowait(response)

    async def _combined_response_consumer(self, ready_event: asyncio.Event):
        ready_event.set()
        last_motion_state = None

        try:
            while True:
                response = await self._response_queue.get()
                if isinstance(response, api.models.MotionGroupState):
                    motion_group_state = response

                    if self._is_operation_in_progress():
                        # ic((motion_group_state.standstill, motion_group_state.execute))
                        pass

                    try:
                        motion_state = motion_group_state_to_motion_state(
                            motion_group_state
                        )  # TODO this should only be called if there is a motion state
                    except ValueError:
                        # TODO somewhat hacky
                        # ic(e)
                        self._response_queue.task_done()
                        continue

                    self._in_queue.put_nowait(response)

                    if last_motion_state is None:
                        last_motion_state = motion_state
                    self._current_location = motion_state.path_parameter
                    last_motion_state = motion_state

                    if (
                        self._is_operation_in_progress()
                        and not motion_group_state.execute
                        and self._operation_handler.current_operation_state
                        == OperationState.COMMANDED
                    ):  #
                        continue  # wait for the execution to actually start

                    assert self._is_operation_in_progress()
                    assert motion_group_state.execute
                    # TODO this is unreachable?
                    if motion_group_state.standstill and not motion_group_state.execute:
                        # ic()
                        assert self._operation_handler.current_operation_state not in (
                            OperationState.INITIAL,
                            OperationState.COMMANDED,
                        ), (
                            f"Unexpected operation state {self._operation_handler.current_operation_state} when standstill is True"
                        )
                        self._complete_operation()
                        if self._detach_on_standstill:
                            ic()
                            break

                    if motion_group_state.execute and isinstance(
                        motion_group_state.execute.details, api.models.TrajectoryDetails
                    ):
                        # ic()
                        match motion_group_state.execute.details.state:
                            case api.models.TrajectoryRunning():
                                # ic()
                                if (
                                    self._operation_handler.current_operation_state
                                    == OperationState.COMMANDED
                                ):
                                    # State transition: COMMANDED -> RUNNING
                                    self._operation_handler._operation_state = (
                                        OperationState.RUNNING
                                    )
                            case api.models.TrajectoryPausedByUser():
                                ic()
                                self._complete_operation()
                                if self._detach_on_standstill:
                                    ic()
                                    break
                            case api.models.TrajectoryEnded():
                                ic()
                                self._complete_operation()
                                if self._detach_on_standstill:
                                    ic()
                                    break

                    self._response_queue.task_done()
        except asyncio.CancelledError:
            logger.debug("Response consumer was cancelled, cleaning up trajectory cursor")
            raise
        finally:
            # stop the request loop
            self.detach()
            # stop the cursor iterator (TODO is this the right place?)
            ic()
            self._in_queue.put_nowait(None)  # TODO make sentinel more explicit

    async def _motion_event_updater(self, interval=0.2):
        while True:
            match self._operation_handler.current_operation_type:
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

    def __aiter__(
        self,
    ) -> AsyncIterator[api.models.ExecuteTrajectoryResponse | api.models.MotionGroupState]:
        return self

    async def __anext__(self) -> api.models.ExecuteTrajectoryResponse | api.models.MotionGroupState:
        value = await self._in_queue.get()
        self._in_queue.task_done()
        if value is None:
            raise StopAsyncIteration
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
