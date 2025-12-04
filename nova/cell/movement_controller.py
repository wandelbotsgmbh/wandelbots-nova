import asyncio
import logging
from typing import AsyncIterator

from nova import api
from nova.actions import MovementControllerContext
from nova.exceptions import ErrorDuringMovement, InitMovementFailed
from nova.types import (
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MovementControllerFunction,
)

logger = logging.getLogger(__name__)

ExecuteJoggingRequestStream = AsyncIterator[api.models.ExecuteJoggingRequest]
ExecuteJoggingResponseStream = AsyncIterator[api.models.ExecuteJoggingResponse]


# TODO: when the message exchange is not working as expected we should gracefully close
# TODO: add the set_io functionality Thorsten Blatter did for the force torque sensor at Schaeffler
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
            motion_group_state_stream: AsyncIterator[api.models.MotionGroupState],
            ready: asyncio.Event,
        ):
            ready.set()
            await error_monitor_task_created.wait()
            try:
                async for motion_group_state in motion_group_state_stream:
                    if motion_group_state.execute and isinstance(
                        motion_group_state.execute.details, api.models.TrajectoryDetails
                    ):
                        logger.info(
                            f"BugSearch: MotionGroupState: {motion_group_state.execute.details}"
                        )
                        if isinstance(
                            motion_group_state.execute.details.state, api.models.TrajectoryEnded
                        ):
                            logger.info("BugSearch: Trajectory ended, cancelling error monitor")
                            error_monitor_task.cancel()
                            break
            finally:
                error_monitor_task.cancel()

        async def error_monitor(
            responses: ExecuteTrajectoryResponseStream, to_cancel: list[asyncio.Task]
        ):
            try:
                async for execute_trajectory_response in responses:
                    if isinstance(execute_trajectory_response, api.models.MovementErrorResponse):
                        # TODO how does this propagate?
                        # TODO what happens to the state consumer?
                        logger.info("BugSearch: MovementErrorResponse received, raising exception")
                        raise ErrorDuringMovement(execute_trajectory_response.message)
            finally:
                for task in to_cancel:
                    task.cancel()

        logger.info("BugSearch: Starting movement controller")
        motion_group_state_stream = context.motion_group_state_stream_gen()
        motion_group_state_monitor_ready = asyncio.Event()
        error_monitor_task_created = asyncio.Event()
        motion_group_state_monitor_task = asyncio.create_task(
            motion_group_state_monitor(motion_group_state_stream, motion_group_state_monitor_ready),
            name="state_consumer",
        )

        await motion_group_state_monitor_ready.wait()
        trajectory_id = api.models.TrajectoryId(id=context.motion_id)
        logger.info("BugSearch: Before initialize movement")
        yield api.models.InitializeMovementRequest(
            trajectory=trajectory_id, initial_location=api.models.Location(0)
        )
        logger.info("BugSearch: After initialize movement")
        execute_trajectory_response = await anext(response_stream)
        initialize_movement_response = execute_trajectory_response.root
        assert isinstance(initialize_movement_response, api.models.InitializeMovementResponse)
        # TODO this should actually check for None but currently the API seems to return an empty string instead
        # create issue with the API to fix this
        if (
            initialize_movement_response.message
            or initialize_movement_response.add_trajectory_error
        ):
            raise InitMovementFailed(initialize_movement_response)

        logger.info("BugSearch: 4")
        set_io_list = context.combined_actions.to_set_io()
        yield api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            set_outputs=set_io_list,
            start_on_io=context.start_on_io,
            pause_on_io=None,
        )
        logger.info("BugSearch: 5")
        execute_trajectory_response = await anext(response_stream)
        start_movement_response = execute_trajectory_response
        assert isinstance(start_movement_response.root, api.models.StartMovementResponse)

        logger.info("BugSearch: 6")
        error_monitor_task = asyncio.create_task(
            error_monitor(response_stream, [motion_group_state_monitor_task]), name="error_monitor"
        )

        error_monitor_task_created.set()
        await motion_group_state_monitor_task
        await error_monitor_task
        logger.info("BugSearch: 8")

    return movement_controller
