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

_START_STATE_MONITOR_TIMEOUT = 5.0


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
        async def motion_group_state_monitor(stream_started_event: asyncio.Event):
            try:
                logger.info("Starting state monitor for trajectory")
                trajectory_ended = False
                async for motion_group_state in context.motion_group_state_stream_gen():
                    if not stream_started_event.is_set():
                        stream_started_event.set()

                    logger.debug(
                        f"Trajectory: {context.motion_id} state monitor received state: {motion_group_state}"
                    )

                    if trajectory_ended and motion_group_state.standstill:
                        logger.info(
                            f"Trajectory: {context.motion_id} state monitor detected standstill."
                        )
                        return

                    if not motion_group_state.execute or not isinstance(
                        motion_group_state.execute.details, api.models.TrajectoryDetails
                    ):
                        continue

                    if isinstance(
                        motion_group_state.execute.details.state, api.models.TrajectoryEnded
                    ):
                        logger.info(
                            f"Trajectory: {context.motion_id} state monitor ended with TrajectoryEnded"
                        )
                        trajectory_ended = True
                        continue

                logger.info(
                    f"Trajectory: {context.motion_id} state monitor ended without TrajectoryEnded"
                )
            except BaseException as e:
                logger.error(
                    f"Trajectory: {context.motion_id} state monitor ended with exception: {type(e).__name__}: {e}"
                )
                raise

        trajectory_id = api.models.TrajectoryId(id=context.motion_id)
        initialize_movement_request = api.models.InitializeMovementRequest(
            trajectory=trajectory_id, initial_location=api.models.Location(0)
        )

        logger.info(f"Trajectory: {context.motion_id} Initializing movement with request")
        yield initialize_movement_request
        initialize_movement_response = await anext(response_stream)

        logger.info(f"Trajectory: {context.motion_id} received initialize movement response.")
        if not isinstance(initialize_movement_response.root, api.models.InitializeMovementResponse):
            raise Exception(
                f"Expected InitializeMovementResponse but got: {initialize_movement_response.root}"
            )

        if (
            initialize_movement_response.root.message
            or initialize_movement_response.root.add_trajectory_error
        ):
            logger.error(
                f"Trajectory: {context.motion_id} initialization failed: {initialize_movement_response.root}"
            )
            raise InitMovementFailed(initialize_movement_response.root)

        # before sending start movement start state monitoring to not loose any data
        # at this point we have exclusive right to do movement on the robot, any movement should be
        # what is coming from our trajectory
        try:
            stream_started_event = asyncio.Event()
            state_monitor = asyncio.create_task(
                motion_group_state_monitor(stream_started_event),
                name=f"motion-group-state-monitor-{context.motion_id}",
            )
            logger.info(f"Trajectory: {context.motion_id} waiting for state monitor to start")
            await asyncio.wait_for(
                stream_started_event.wait(), timeout=_START_STATE_MONITOR_TIMEOUT
            )
        except asyncio.TimeoutError:
            logger.error(f"Trajectory: {context.motion_id} state monitor failed to start in time")
            state_monitor.cancel()
            raise Exception("State monitor failed to start in time")

        set_io_list = context.combined_actions.to_set_io()
        start_movement_request = api.models.StartMovementRequest(
            direction=api.models.Direction.DIRECTION_FORWARD,
            set_outputs=set_io_list,
            start_on_io=context.start_on_io,
            pause_on_io=None,
        )
        logger.info(f"Trajectory: {context.motion_id} sending StartMovementRequest")
        yield start_movement_request

        start_movement_response = await anext(response_stream)
        logger.info(f"Trajectory: {context.motion_id} received start movement response.")
        if not isinstance(start_movement_response.root, api.models.StartMovementResponse):
            raise Exception(
                f"Expected StartMovementResponse but got: {start_movement_response.root}"
            )

        # the only possible response we can get from web socket at this point is a movement failure
        error_message = ""

        async def error_response_consumer(response_stream: ExecuteTrajectoryResponseStream):
            response = await anext(response_stream)
            if not isinstance(response.root, api.models.MovementErrorResponse):
                logger.error(
                    f"Trajectory: {context.motion_id} received unexpected response: {response}"
                )
                return

            nonlocal error_message
            error_message = response.root.message

        error_consumer = asyncio.create_task(
            error_response_consumer(response_stream),
            name=f"execute-trajectory-error-consumer-{context.motion_id}",
        )
        tasks = {error_consumer, state_monitor}

        try:
            logger.info(f"Trajectory: {context.motion_id} waiting for completion or error")
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            if state_monitor in done:
                logger.info(f"Trajectory: {context.motion_id} completed via state monitor")
                return

            if error_consumer in done:
                logger.info(f"Trajectory: {context.motion_id} error consumer completed")
                raise ErrorDuringMovement(
                    f"Error occurred during trajectory execution: {error_message}"
                )
        except BaseException as e:
            logger.error(
                f"Trajectory: {context.motion_id} encountered exception: {type(e).__name__}: {e}"
            )
            raise
        finally:
            for task in tasks:
                if not task.done():
                    logger.info(
                        f"Trajectory: {context.motion_id} cancelling task: {task.get_name()}"
                    )
                    task.cancel()

            logger.info(f"Trajectory: {context.motion_id} waiting for tasks to finish")
            await asyncio.gather(*tasks, return_exceptions=True)
            logger.info(f"Trajectory: {context.motion_id} all tasks finished")

    return movement_controller
