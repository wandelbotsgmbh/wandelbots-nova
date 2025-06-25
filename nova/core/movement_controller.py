import asyncio
import bisect
from datetime import datetime
from typing import AsyncIterator, Callable

import wandelbots_api_client.v2 as wb
from icecream import ic

from nova.actions import MovementControllerContext
from nova.core import logger
from nova.core.exceptions import InitMovementFailed
from nova.types import (
    ExecuteJoggingRequestStream,
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MotionState,
    MovementControllerFunction,
    Pose,
    RobotState,
)

ic.configureOutput(
    includeContext=True,
    prefix=lambda ic: f"{datetime.now()} - {ic.frame.filename}:{ic.frame.lineno} - ",
)


def movement_to_motion_state(movement: wb.models.Movement) -> MotionState:
    """Convert a wb.models.Movement to a MotionState."""
    if (
        movement.movement.state is None
        or movement.movement.current_location is None
        or len(movement.movement.state.active_motion_groups) == 0
    ):
        assert False, "This should not happen"  # depending on NC-1105

    # TODO: in which cases do we have more than one motion group here?
    motion_group = movement.movement.state.active_motion_groups[0]
    return motion_group_state_to_motion_state(
        motion_group, float(movement.movement.current_location)
    )


def motion_group_state_to_motion_state(
    motion_group_state: wb.models.MotionGroupState, path_parameter: float
) -> MotionState:
    tcp_pose = Pose(
        tuple(motion_group_state.tcp_pose.position + motion_group_state.tcp_pose.orientation)
    )
    joints = (
        tuple(motion_group_state.joint_current.joints) if motion_group_state.joint_current else None
    )
    return MotionState(
        motion_group_id=motion_group_state.motion_group,
        path_parameter=path_parameter,
        state=RobotState(pose=tcp_pose, joints=joints),
    )


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
        # The first request is to initialize the movement
        yield wb.models.InitializeMovementRequest(
            message_type="InitializeMovementRequest",
            trajectory=wb.models.InitializeMovementRequestTrajectory(
                wb.models.TrajectoryId(message_type="TrajectoryId", id=context.motion_id)
            ),
            initial_location=0,
        )  # type: ignore

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
        yield wb.models.StartMovementRequest(
            message_type="StartMovementRequest",
            direction=wb.models.Direction.DIRECTION_FORWARD,
            set_ios=set_io_list,
            start_on_io=None,
            pause_on_io=None,
        )  # type: ignore

        # then we wait until the movement is finished
        async for execute_trajectory_response in response_stream:
            instance = execute_trajectory_response.actual_instance
            # Stop when standstill indicates motion ended
            if isinstance(instance, wb.models.Standstill):
                if instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                    return

    return movement_controller


class Jogger:
    def __init__(self, effect_stream, motion_group_id: str, tcp_name: str | None = None):
        self._motion_group_id = motion_group_id
        self._tcp_name = tcp_name
        # TODO support the other parameters
        self._command_queue = asyncio.Queue()
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
        self, response_stream: ExecuteTrajectoryResponseStream
    ) -> ExecuteTrajectoryRequestStream:
        self._response_stream = response_stream
        async for request in init_movement_gen(self._motion_group_id, response_stream):
            yield request

        with asyncio.TaskGroup() as tg:
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
            self._response_queue.put_nowait(effect)

    async def _response_consumer(self):
        async for response in self._response_stream:
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
        ic(motion_group_state)


class TrajectoryCursor:
    def __init__(self, joint_trajectory: wb.models.JointTrajectory):
        self.joint_trajectory = joint_trajectory
        self._command_queue = asyncio.Queue()
        self._breakpoints = []

    def __call__(self, context: MovementControllerContext):
        self.context = context
        return self.cntrl

    def pause_at(self, location: float):
        # How to pause at an exact location?
        bisect.insort(self._breakpoints, location)

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

    async def cntrl(
        self, response_stream: ExecuteTrajectoryResponseStream
    ) -> ExecuteTrajectoryRequestStream:
        self._response_stream = response_stream
        async for request in init_movement_gen(self.context.motion_id, response_stream):
            yield request

        self._response_consumer_task: asyncio.Task[None] = asyncio.create_task(
            self._response_consumer(), name="response_consumer"
        )

        async for request in self._request_loop():
            yield request
        await self._response_consumer_task  # Where to put?

    async def _request_loop(self):
        while True:
            yield await self._command_queue.get()
            self._command_queue.task_done()

    async def _response_consumer(self):
        last_movement = None

        try:
            async for response in self._response_stream:
                instance = response.actual_instance
                if isinstance(instance, wb.models.Movement):
                    if last_movement is None:
                        last_movement = instance.movement
                    self._handle_movement(instance.movement, last_movement)
                    last_movement = instance.movement

                if isinstance(instance, wb.models.Standstill):
                    if instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                        # self.context.movement_consumer(None)
                        ic()
                        break
        except asyncio.CancelledError:
            ic()
            raise
        except Exception as e:
            ic(e)
            raise
        ic()

    def _handle_movement(
        self, curr_movement: wb.models.MovementMovement, last_movement: wb.models.MovementMovement
    ):
        if not self._breakpoints:
            return
        curr_location = curr_movement.current_location
        last_location = last_movement.current_location
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


async def init_jogging_gen(
    motion_group_id, tcp_name, response_stream
) -> ExecuteJoggingRequestStream:
    # The first request is to initialize the movement
    yield wb.models.InitializeJoggingRequest(motion_group=motion_group_id, tcp=tcp_name)  # type: ignore

    # then we get the response
    initialize_jogging_response = await anext(response_stream)
    assert isinstance(initialize_jogging_response, wb.models.ExecuteTrajectoryResponse), (
        "Expected ExecuteTrajectoryResponse, got: " + str(initialize_jogging_response)
    )
    assert isinstance(
        initialize_jogging_response.actual_instance, wb.models.InitializeJoggingResponse
    ), "Expected InitializeJoggingResponse, got: " + str(
        initialize_jogging_response.actual_instance
    )
    if isinstance(initialize_jogging_response.actual_instance, wb.models.InitializeJoggingResponse):
        assert initialize_jogging_response.actual_instance.kind == "INITIALIZE_RECEIVED"


async def init_movement_gen(motion_id, response_stream) -> ExecuteTrajectoryRequestStream:
    # The first request is to initialize the movement
    yield wb.models.InitializeMovementRequest(trajectory=motion_id, initial_location=0)  # type: ignore

    # then we get the response
    initialize_movement_response = await anext(response_stream)
    if isinstance(
        initialize_movement_response.actual_instance, wb.models.InitializeMovementResponse
    ):
        r1 = initialize_movement_response.actual_instance
        if not r1.init_response.succeeded:
            raise InitMovementFailed(r1.init_response)


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
        yield wb.models.StartMovementRequest(
            set_ios=set_io_list, start_on_io=None, pause_on_io=None
        )  # type: ignore

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
