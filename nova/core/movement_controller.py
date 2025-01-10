import wandelbots_api_client as wb
from loguru import logger

from nova.actions import (
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MovementControllerContext,
    MovementControllerFunction,
)
from nova.core.exceptions import InitMovementFailed


def move_forward(context: MovementControllerContext) -> MovementControllerFunction:
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

        # then we wait until the movement is finished
        async for execute_trajectory_response in response_stream:
            r2 = execute_trajectory_response.actual_instance
            # Terminate the generator
            if isinstance(r2, wb.models.Standstill):
                if r2.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                    return

    return movement_controller


def speed_up(context: MovementControllerContext) -> MovementControllerFunction:
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
            response = execute_trajectory_response.actual_instance
            # Terminate the generator
            if isinstance(response, wb.models.Standstill):
                if response.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                    return

            if isinstance(response, wb.models.PlaybackSpeedResponse):
                playback_speed = response.playback_speed_response
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
