from typing import Callable

import wandelbots_api_client.v2 as wb

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


def motion_group_state_to_motion_state(
    motion_group_state: wb.models.MotionGroupState,
) -> MotionState:
    tcp_pose = Pose(
        tuple(motion_group_state.tcp_pose.position + motion_group_state.tcp_pose.orientation)
    )
    joints = (
        tuple(motion_group_state.joint_current.joints) if motion_group_state.joint_current else None
    )
    # TODO not very clean
    path_parameter = (
        motion_group_state.execute.details.actual_instance.location
        if motion_group_state.execute
        and motion_group_state.execute.details
        and isinstance(
            motion_group_state.execute.details.actual_instance, wb.models.TrajectoryDetails
        )
        else None
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


def speed_up(
    context: MovementControllerContext, on_movement: Callable[[MotionState | None], None]
) -> MovementControllerFunction:
    async def movement_controller(
        response_stream: ExecuteTrajectoryResponseStream,
    ) -> ExecuteTrajectoryRequestStream:
        # The first request is to initialize the movement
        yield wb.models.InitializeMovementRequest(trajectory=context.motion_id, initial_location=0)

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
