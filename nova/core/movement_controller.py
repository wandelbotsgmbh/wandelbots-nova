from functools import singledispatch
from typing import Any

import wandelbots_api_client as wb

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
        yield wb.models.InitializeMovementRequest(trajectory=context.motion_id, initial_location=0)

        # then we get the response
        initialize_movement_response = await anext(response_stream)
        if isinstance(
            initialize_movement_response.actual_instance, wb.models.InitializeMovementResponse
        ):
            r1 = initialize_movement_response.actual_instance
            if not r1.init_response.succeeded:
                raise InitMovementFailed(r1.init_response)

        # Send playback speed request AFTER initialization but BEFORE starting movement
        # This ensures the speed is set before the movement begins
        yield wb.models.ExecuteTrajectoryRequest(
            wb.models.PlaybackSpeedRequest(playback_speed_in_percent=context.effective_speed)
        )

        # Wait for playback speed response
        playback_speed_response = await anext(response_stream)
        if isinstance(playback_speed_response.actual_instance, wb.models.PlaybackSpeedResponse):
            logger.info(
                f"Playback speed set to: {playback_speed_response.actual_instance.playback_speed_response}%"
            )

        # The second request is to start the movement
        set_io_list = context.combined_actions.to_set_io()
        yield wb.models.StartMovementRequest(
            set_ios=set_io_list, start_on_io=None, pause_on_io=None
        )

        # --- Responsive runtime playback speed update logic ---
        from nova.core.playback_control import PlaybackSpeedPercent, RobotId, get_playback_manager

        robot_id = RobotId(
            context.motion_id.split(":")[0] if ":" in context.motion_id else context.motion_id
        )
        manager = get_playback_manager()
        last_sent_speed = context.effective_speed
        last_check_time = 0.0
        check_interval = 0.2  # Check every 200ms

        # Run an initial check before starting execution
        method_speed = (
            PlaybackSpeedPercent(context.method_speed) if context.method_speed is not None else None
        )
        effective_speed = manager.get_effective_speed(robot_id, method_speed=method_speed)
        if effective_speed != last_sent_speed:
            logger.info(f"Initial playback speed update: {last_sent_speed}% -> {effective_speed}%")
            last_sent_speed = effective_speed
            yield wb.models.ExecuteTrajectoryRequest(
                wb.models.PlaybackSpeedRequest(playback_speed_in_percent=effective_speed)
            )

        # then we wait until the movement is finished
        import time

        last_check_time = time.time()
        check_interval = 0.2  # Check every 200ms

        async for execute_trajectory_response in response_stream:
            # Check for speed changes periodically
            current_time = time.time()
            if current_time - last_check_time > check_interval:
                last_check_time = current_time

                method_speed = (
                    PlaybackSpeedPercent(context.method_speed)
                    if context.method_speed is not None
                    else None
                )
                effective_speed = manager.get_effective_speed(robot_id, method_speed=method_speed)

                if effective_speed != last_sent_speed:
                    logger.info(
                        f"Runtime playback speed change detected: {last_sent_speed}% -> {effective_speed}%"
                    )
                    last_sent_speed = effective_speed
                    yield wb.models.ExecuteTrajectoryRequest(
                        wb.models.PlaybackSpeedRequest(playback_speed_in_percent=effective_speed)
                    )

            instance = execute_trajectory_response.actual_instance
            # Stop when standstill indicates motion ended
            if isinstance(instance, wb.models.Standstill):
                if instance.standstill.reason == wb.models.StandstillReason.REASON_MOTION_ENDED:
                    break

    return movement_controller
