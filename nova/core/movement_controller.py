import asyncio
import bisect
import pdb
from functools import singledispatch
from typing import Any, Callable
import sys

import wandelbots_api_client as wb
from icecream import ic

from nova.actions import MovementControllerContext
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
from .debug_cursor import CursorPdb


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


class TrajectoryCursor:
    def __init__(self, joint_trajectory: wb.models.JointTrajectory):
        self.joint_trajectory = joint_trajectory
        self._command_queue = asyncio.Queue()
        self._breakpoints = []
        self._COMMAND_QUEUE_SENTINAL = object()
        self._current_location = 0.0

    def __call__(self, context: MovementControllerContext):
        self.context = context
        return self.cntrl

    def pause_at(self, location: float, debug_break: bool = True):
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

    def forward_to(self, location: float):
        # TODO gererated unchecked code
        if not self._breakpoints:
            raise ValueError("No breakpoints set. Use pause_at() to set a breakpoint.")
        if location not in self._breakpoints:
            raise ValueError(f"Location {location} is not a valid breakpoint.")
        index = bisect.bisect_left(self._breakpoints, location)
        if index < len(self._breakpoints) and self._breakpoints[index] == location:
            self.pause_at(location)
        else:
            raise ValueError(f"Location {location} is not a valid breakpoint.")

    def forward_to_next(self):
        # use self._current_location to find the next breakpoint
        index = bisect.bisect_right(self._breakpoints, self._current_location)
        if index < len(self._breakpoints):
            next_breakpoint = self._breakpoints[index]
            self.forward_to(next_breakpoint)
        else:
            raise ValueError("No next breakpoint found after current location.")

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

    def detach(self):
        # TODO this does not stop the movement atm, it just stops controlling movement
        # this is especially open for testing what happens when the movement is backwards
        # and no end is reached
        self._command_queue.put_nowait(self._COMMAND_QUEUE_SENTINAL)

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
            ic()
            yield request
        ic()
        await self._response_consumer_task  # Where to put?

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

    async def _response_consumer(self):
        last_movement = None

        try:
            async for response in self._response_stream:
                instance = response.actual_instance
                if isinstance(instance, wb.models.Movement):
                    if last_movement is None:
                        last_movement = instance.movement
                    self._current_location = instance.movement.current_location
                    self._handle_movement(instance.movement, last_movement)
                    last_movement = instance.movement
                elif isinstance(instance, wb.models.Standstill):
                    ic()
                    if instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                        # self.context.movement_consumer(None)
                        ic()
                        break
                    elif instance.standstill.reason == wb.models.StandstillReason.REASON_USER_PAUSED_MOTION:
                        ic()
                        CursorPdb(self).set_trace(sys._getframe())
                        break
                else:
                    ic(instance)
                    # Handle other types of instances if needed
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
