import wandelbots_api_client.v2 as wb

from nova.actions import MovementControllerContext
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
    joints = tuple(motion_group_state.joint_position) if motion_group_state.joint_position else None
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
        state=RobotState(pose=tcp_pose, tcp=motion_group_state.tcp, joints=joints),
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
                wb.models.TrajectoryId(id=context.motion_id)
            ),
            initial_location=0,
        )

        # then we get the response
        initialize_movement_response = await anext(response_stream)
        assert isinstance(initialize_movement_response, wb.models.InitializeMovementResponse)
        if initialize_movement_response.message is not None:
            raise InitMovementFailed(initialize_movement_response.message)

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
