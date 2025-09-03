import asyncio
import bisect
from enum import Enum, auto
from functools import singledispatch
from math import ceil, floor
from typing import Any, AsyncIterator, Callable, Optional

import pydantic
import wandelbots_api_client as wb
from aiohttp_retry import dataclass
from icecream import ic

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


def speed_up(
    context: MovementControllerContext, on_movement: Callable[[MotionState | None], None]
) -> MovementControllerFunction:
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

        counter = 0
        latest_speed = 10
        # then we wait until the movement is finished
        async for execute_trajectory_response in response_stream:
            counter += 1
            instance = execute_trajectory_response.actual_instance
            # Send the current location to the consume
            if isinstance(instance, wb.models.Movement):
                motion_state = movement_to_motion_state(instance)
                if motion_state:
                    on_movement(motion_state)

            # Terminate the generator
            if isinstance(instance, wb.models.Standstill):
                if instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                    on_movement(None)
                    return

            if isinstance(instance, wb.models.PlaybackSpeedResponse):
                playback_speed = instance.playback_speed_response
                logger.info(f"Current playback speed: {playback_speed}")

            if counter % 10 == 0:
                yield wb.models.ExecuteTrajectoryRequest(
                    wb.models.PlaybackSpeedRequest(playback_speed_in_percent=latest_speed)
                )
                counter = 0
                latest_speed += 5
                if latest_speed > 100:
                    latest_speed = 100

    return movement_controller


class OperationType(Enum):
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
        self.actions = CombinedActions(items=actions)
        self._command_queue = asyncio.Queue()
        self._COMMAND_QUEUE_SENTINAL = object()
        self._in_queue: asyncio.Queue[wb.models.ExecuteTrajectoryResponse | None] = asyncio.Queue()
        self._breakpoints = []
        self._current_location = initial_location
        self._overshoot = 0.0
        self._target_location = self.end_location
        self._detach_on_standstill = detach_on_standstill

        self._current_operation_future: Optional[asyncio.Future[OperationResult]] = None
        self._current_operation_type: Optional[OperationType] = None
        self._current_target_location: Optional[float] = None
        self._current_start_location: Optional[float] = None

    def __call__(self, context: MovementControllerContext):
        self.context = context
        self.combined_actions: CombinedActions = context.combined_actions
        self.motion_id = context.motion_id  # TODO does this override here make sense?
        # self.forward_to_next_action()
        return self.cntrl

    @property
    def end_location(self) -> float:
        # The assert ensures that we not accidentally use it in __init__ before it is set.
        assert getattr(self, "joint_trajectory", None) is not None
        return self.joint_trajectory.locations[-1]

    def pause_at(self, location: float, debug_break: bool = True):
        # How to pause at an exact location? (will be implemented by Motion Control)
        bisect.insort(self._breakpoints, location)

    def forward(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        # should idempotently move forward
        future = self._start_operation(OperationType.FORWARD)

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
            future = asyncio.Future()
            future.set_exception(
                ValueError("Cannot move forward to a location before the current location")
            )
            return future
        self._target_location = location
        return self.forward(playback_speed_in_percent=playback_speed_in_percent)

    def forward_to_next_action(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        # TODO this seems to fail if there is only one action
        index = floor(self._current_location - self._overshoot) + 1
        ic(self._current_location, self._overshoot, index, len(self.combined_actions))
        if index < len(self.combined_actions):
            return self.forward_to(index, playback_speed_in_percent=playback_speed_in_percent)
        else:
            # Create a future that's already resolved with an error
            future = asyncio.Future()
            future.set_exception(ValueError("No next action found after current location."))
            return future

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
            future = asyncio.Future()
            future.set_exception(
                ValueError("Cannot move backward to a location after the current location")
            )
            return future
        self._target_location = location
        return self.backward(playback_speed_in_percent=playback_speed_in_percent)

    def backward_to_previous_action(
        self, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[OperationResult]:
        index = ceil(self._current_location - self._overshoot) - 1
        ic(self._current_location, self._overshoot, index, len(self.combined_actions))
        if index >= 0:
            return self.backward_to(index, playback_speed_in_percent=playback_speed_in_percent)
        else:
            # Create a future that's already resolved with an error
            future = asyncio.Future()
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
        self._command_queue.put_nowait(self._COMMAND_QUEUE_SENTINAL)

    async def cntrl(
        self, response_stream: ExecuteTrajectoryResponseStream
    ) -> ExecuteTrajectoryRequestStream:
        self._response_stream = response_stream
        async for request in init_movement_gen(
            self.motion_id, response_stream, self._current_location
        ):
            yield request

        self._response_consumer_task: asyncio.Task[None] = asyncio.create_task(
            self._response_consumer(), name="response_consumer"
        )

        async for request in self._request_loop():
            yield request
        self._response_consumer_task.cancel()
        try:
            await self._response_consumer_task  # Where to put?
        except asyncio.CancelledError:
            pass
        ic()

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

    async def _request_loop(self):
        while True:
            command = await self._command_queue.get()
            if command is self._COMMAND_QUEUE_SENTINAL:
                ic("Detaching movement controller (TrajectoryCursor)")
                self._command_queue.task_done()
                break
            ic(command)
            yield command
            # yield await self._command_queue.get()
            self._command_queue.task_done()
        ic()

    async def _response_consumer(self):
        last_movement = None

        try:
            async for response in self._response_stream:
                self._in_queue.put_nowait(response)
                instance = response.actual_instance
                # ic(instance)
                if isinstance(instance, wb.models.Movement):
                    # ic(instance.movement.state.timestamp)
                    if last_movement is None:
                        last_movement = instance.movement
                    self._current_location = instance.movement.current_location
                    self._handle_movement(instance.movement, last_movement)
                    last_movement = instance.movement
                elif isinstance(instance, wb.models.Standstill):
                    ic(instance.standstill)
                    match instance.standstill.reason:
                        case wb.models.StandstillReason.REASON_USER_PAUSED_MOTION:
                            self._overshoot = self._current_location - self._target_location
                            ic(self._current_location, self._target_location, self._overshoot)
                            self._complete_operation()
                            if self._detach_on_standstill:
                                break
                        case wb.models.StandstillReason.REASON_MOTION_ENDED:
                            self._complete_operation()
                            if self._detach_on_standstill:
                                ic()
                                break
                            # self.context.movement_consumer(None)
                            # break
                else:
                    ic(instance)
                    # Handle other types of instances if needed
        except asyncio.CancelledError:
            ic()
            raise
        except Exception as e:
            ic(e)
            raise
        finally:
            ic()
            # stop the request loop
            self.detach()
            # stop the cursor iterator (TODO is this the right place?)
            self._in_queue.put_nowait(None)  # TODO make sentinel more explicit

    def _handle_movement(
        self, curr_movement: wb.models.MovementMovement, last_movement: wb.models.MovementMovement
    ):
        if not 0.0 < self._target_location < self.end_location:
            return
        curr_location = curr_movement.current_location
        last_location = last_movement.current_location
        if curr_location == last_location:
            return
        if curr_location > last_location:
            # moving forwards
            if last_location <= self._target_location < curr_location:
                ic(last_location, self._target_location, curr_location)
                self._pause()
        else:
            # moving backwards
            if last_location > self._target_location >= curr_location:
                ic(last_location, self._target_location, curr_location)
                self._pause()

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
    yield wb.models.InitializeMovementRequest(
        trajectory=motion_id, initial_location=initial_location
    )  # type: ignore

    # then we get the response
    initialize_movement_response = await anext(response_stream)
    # TODO handle error response here (MovementError)
    if isinstance(
        initialize_movement_response.actual_instance, wb.models.InitializeMovementResponse
    ):
        r1 = initialize_movement_response.actual_instance
        if not r1.init_response.succeeded:  # TODO we don't come here if there was an error response
            raise InitMovementFailed(r1.init_response)


from faststream import FastStream
from faststream.nats import NatsBroker


class TrajectoryTuner:
    def __init__(self, actions, plan_fn, execute_fn):
        self.actions = actions
        self.plan_fn = plan_fn
        self.execute_fn = execute_fn

    async def tune(self):
        finished_tuning = False
        last_operation_result = None  # TODO implement this feature
        joint_trajectory = await self.plan_fn(self.actions)
        current_cursor = None  # TODO maybe we can also initialize this here already

        broker = NatsBroker()

        @broker.subscriber("trajectory-cursor")
        async def controller_handler(command: str, speed: Optional[pydantic.PositiveInt] = None):
            nonlocal last_operation_result, finished_tuning
            ic(command, speed)
            match command:
                case "forward":
                    future = current_cursor.forward(playback_speed_in_percent=speed)
                    # last_operation_result = await future
                case "step-forward":
                    future = current_cursor.forward_to_next_action(playback_speed_in_percent=speed)
                    # await future
                case "backward":
                    future = current_cursor.backward(playback_speed_in_percent=speed)
                    # await future
                case "step-backward":
                    future = current_cursor.backward_to_previous_action(
                        playback_speed_in_percent=speed
                    )
                    # await future
                case "pause":
                    future = current_cursor.pause()
                    # if future is not None:
                    #     await future
                case "finish":
                    current_cursor.detach()
                    finished_tuning = True
                case _:
                    ic("Unknown command")

        async def runtime_monitor(interval=0.5):
            start_time = asyncio.get_event_loop().time()
            while True:
                elapsed = asyncio.get_event_loop().time() - start_time
                logger.warning(f"{elapsed:.2f}s")
                await asyncio.sleep(interval)

        faststream_app = FastStream(broker)
        nats_task = asyncio.create_task(faststream_app.run())
        # runtime_task = asyncio.create_task(runtime_monitor(0.5))  # Output every 0.5 seconds

        try:
            # tuning loop
            current_location = 0.0
            while not finished_tuning:
                # TODO this plans the second time for the same actions when we get here because
                # the initial joint trajectory was already planned before the MotionGroup._execute call
                motion_id, joint_trajectory = await self.plan_fn(self.actions)
                ic(current_location)
                current_cursor = TrajectoryCursor(
                    motion_id,
                    joint_trajectory,
                    self.actions,
                    initial_location=current_location,
                    detach_on_standstill=True,
                )
                execution_task = asyncio.create_task(
                    self.execute_fn(
                        client_request_generator=current_cursor(
                            MovementControllerContext(
                                combined_actions=CombinedActions(items=self.actions),
                                motion_id=motion_id,
                            )
                        )
                    )
                )
                ic()
                async for execute_response in current_cursor:
                    yield execute_response
                ic()
                await execution_task
                current_location = (
                    current_cursor._current_location
                )  # TODO is this the cleanest way to get the current location?

                # somehow obtain the modified actions for the next iteration
                ic()

            # runtime_task.cancel()
            # await runtime_task
        except asyncio.CancelledError:
            ic()
            pass
        faststream_app.exit()
        await nats_task
