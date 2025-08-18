import asyncio
import bisect
from typing import AsyncIterator

import wandelbots_api_client.v2 as wb
from icecream import ic

from nova.actions import MovementControllerContext
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
        yield wb.models.StartMovementRequest(
            set_ios=set_io_list, start_on_io=None, pause_on_io=None
        )  # type: ignore

        # then we wait until the movement is finished
        async for execute_trajectory_response in response_stream:
            instance = execute_trajectory_response.actual_instance
            # Stop when standstill indicates motion ended
            if isinstance(instance, wb.models.Standstill):
                if instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                    return

    return TrajectoryCursor(context)


class Jogger:
    def __init__(self, effect_stream, motion_group_id: str, tcp_name: str | None = None):
        self._motion_group_id = motion_group_id
        self._tcp_name = tcp_name
        # TODO support the other parameters
        self._command_queue = asyncio.Queue()
        self._response_queue = asyncio.Queue()
        self._effect_stream: AsyncIterator[wb.models.MotionGroupState] = effect_stream

    def __call__(self, context: MovementControllerContext):
        self.context = context
        return self.cntrl

    def jog_tcp(
        self,
        translation: list[float] | None = None,
        rotation: list[float] | None = None,
        use_tool_coordinate_system: bool = False,
    ):
        if translation is None:
            translation = [0.0, 0.0, 0.0]
        if rotation is None:
            rotation = [0.0, 0.0, 0.0]
        self._command_queue.put_nowait(
            wb.models.TcpVelocityRequest(
                translation=translation,
                rotation=rotation,
                use_tool_coordinate_system=use_tool_coordinate_system,
            )
        )

    def pause(self):
        self._command_queue.put_nowait(wb.models.PauseJoggingRequest())

    async def cntrl(
        self, response_stream: AsyncIterator[wb.models.ExecuteJoggingResponse]
    ) -> AsyncIterator[wb.models.ExecuteJoggingRequest]:
        self._response_stream = response_stream
        async for request in init_jogging_gen(
            self._motion_group_id, self._tcp_name, response_stream
        ):
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
        try:
            while True:
                response = await self._response_queue.get()
                # TODO singledispatch?
                if isinstance(response, wb.models.MotionGroupState):
                    self._handle_motion_state(response)
                elif isinstance(response, wb.models.ExecuteJoggingResponse):
                    if isinstance(response.actual_instance, wb.models.MovementErrorResponse):
                        ic()
                        self._response_queue.task_done()
                        break
                self._response_queue.task_done()
        except asyncio.CancelledError:
            ic()
            raise
        except Exception as e:
            ic(e)
            raise
        ic()

    def _handle_motion_state(self, motion_group_state: wb.models.MotionGroupState):
        pass
        # ic(motion_group_state)


async def init_jogging_gen(
    motion_group_id, tcp_name, response_stream
) -> ExecuteJoggingRequestStream:
    # The first request is to initialize the movement
    yield wb.models.InitializeJoggingRequest(motion_group=motion_group_id, tcp=tcp_name)  # type: ignore

    # then we get the response
    initialize_jogging_response = await anext(response_stream)
    assert isinstance(
        initialize_jogging_response.actual_instance, wb.models.InitializeJoggingResponse
    ), "Expected InitializeJoggingResponse, got: " + str(
        initialize_jogging_response.actual_instance
    )
    if isinstance(initialize_jogging_response.actual_instance, wb.models.InitializeJoggingResponse):
        assert initialize_jogging_response.actual_instance.kind == "INITIALIZE_RECEIVED"
