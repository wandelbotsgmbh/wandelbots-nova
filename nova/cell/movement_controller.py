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
        async def motion_group_state_monitor():
            async for motion_group_state in context.motion_group_state_stream_gen():
                if not motion_group_state.execute or not isinstance(
                    motion_group_state.execute.details, api.models.TrajectoryDetails
                ):
                    continue

                if isinstance(motion_group_state.execute.details.state, api.models.TrajectoryEnded):
                    return

        # create a generator from websocket
        # pass this generator to movement controller and get another generator back
        # drive this generator and send each data to the websocket
        

        # as long as there is something to return the flow will continue
        # first we send initialize movement
        # than start monitoring
        # if monitoring finishes return

        # if 
        # than start movement


        # the generator we return should first initialize the movement
        # and consume the response to make sure we started
        # and than start the movement
        # and than consume  

        # as long as we return a response, flow will continue
        # if we send a data, there should be matching resposne from server

        # first request


        # if we don't return something or finish the function, flow is blocked
        # if we return something but don't get an answer flow is blocked

        # handle initialize movement
        trajectory_id = api.models.TrajectoryId(id=context.motion_id)
        initialize_movement_request = api.models.InitializeMovementRequest(
            trajectory=trajectory_id, initial_location=api.models.Location(0)
        )

        yield initialize_movement_request
        initialize_movement_response = await anext(response_stream)
        assert isinstance(initialize_movement_response.root, api.models.InitializeMovementResponse)

        if (
            initialize_movement_response.root.message
            or initialize_movement_response.root.add_trajectory_error
        ):
            raise InitMovementFailed(initialize_movement_response.root)



        # before sending start movement start state monitoring to not loose any data
        # at this point we have exclusive right to do movement on the robot, any movement should be what is coming from our trajectory
        state_monitor = asyncio.create_task(motion_group_state_monitor())
        await asyncio.sleep(0)

        set_io_list = context.combined_actions.to_set_io()
        start_movement_request = api.models.StartMovementRequest(
                direction=api.models.Direction.DIRECTION_FORWARD,
                set_outputs=set_io_list,
                start_on_io=context.start_on_io,
                pause_on_io=None,
        )
        yield start_movement_request


        start_movement_response = await anext(response_stream)
        assert isinstance(start_movement_response.root, api.models.StartMovementResponse)

        # the only possible response we can get from web socket at this point is a movement failure
        error_consumer = asyncio.create_task(anext(response_stream))
        done, pending = await asyncio.wait(fs=[error_consumer, state_monitor], return_when=asyncio.FIRST_COMPLETED)
        if done == state_monitor:
            return
        
        if done == error_consumer:
            raise ErrorDuringMovement("Error occurred during trajectory execution")


    return movement_controller
