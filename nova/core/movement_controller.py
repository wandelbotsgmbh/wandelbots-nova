from functools import singledispatch
from typing import Any, Union

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
    Movement controller that manages robot trajectory execution, runtime speed changes, pause/resume, and direction control.

    This controller handles bidirectional websocket communication with the robot execution system,
    processing responses while simultaneously monitoring for external speed, state, and direction changes from tools
    like VS Code extensions or runtime control interfaces.

    The controller uses a dual approach for change detection:
    1. Response-based: Checks for speed/state/direction changes on every websocket response
    2. Event-driven: Continuous monitoring with minimal latency for responsive UI controls

    For VS Code sliders, pause/resume buttons, direction controls, and other interactive controls, the event-driven
    approach provides near-instantaneous response times (1ms yields vs 50ms polling).

    Supported runtime controls:
    - Speed changes: Updates playback speed dynamically during execution
    - Pause/Resume: Pauses robot execution and resumes from the same position
    - Direction changes: Sets forward/backward direction applied on next resume

    Direction Control:
    - Direction changes take effect immediately when paused
    - When resumed after a direction change, the robot will execute the trajectory in the new direction
    - Forward direction: Normal trajectory execution from current position to end
    - Backward direction: Reverse trajectory execution from current position toward start

    Args:
        context: Movement controller context containing trajectory, robot ID, and initial speed

    Returns:
        Async generator function that yields websocket requests for robot execution
    """

    async def movement_controller(
        response_stream: ExecuteTrajectoryResponseStream,
    ) -> ExecuteTrajectoryRequestStream:
        import asyncio

        # Set up runtime speed monitoring for external speed changes
        from nova.core.playback_control import (
            MotionGroupId,
            PlaybackDirection,
            PlaybackSpeedPercent,
            PlaybackState,
            get_playback_manager,
        )

        robot_id = MotionGroupId(context.robot_id)
        manager = get_playback_manager()

        # Set execution state to indicate movement is starting
        manager.set_execution_state(robot_id, PlaybackState.EXECUTING)

        # Use a queue to coordinate between request generation and response processing
        request_queue: asyncio.Queue[
            Union[wb.models.ExecuteTrajectoryRequest, wb.models.StartMovementRequest]
        ] = asyncio.Queue()
        motion_completed = asyncio.Event()
        last_sent_speed = context.effective_speed
        last_sent_state = PlaybackState.PLAYING  # Track current playback state
        last_sent_direction = PlaybackDirection.FORWARD  # Track current playback direction
        pending_direction_logged = False  # Track if we've already logged a pending direction change

        async def response_processor():
            """Process websocket responses and check for external speed changes on each response."""
            nonlocal last_sent_speed, last_sent_state, last_sent_direction, pending_direction_logged

            try:
                async for response in response_stream:
                    instance = response.actual_instance

                    # Handle initialization response
                    if isinstance(instance, wb.models.InitializeMovementResponse):
                        if not instance.init_response.succeeded:
                            raise InitMovementFailed(instance.init_response)

                    # Handle playback speed responses
                    elif isinstance(instance, wb.models.PlaybackSpeedResponse):
                        logger.info(
                            f"Playback speed confirmed: requested_value={instance.playback_speed_response.requested_value}%"
                        )

                    # Handle pause movement responses
                    elif isinstance(instance, wb.models.PauseMovementResponse):
                        logger.info("Movement pause acknowledged by robot")

                    # Check for external speed, state, and direction changes on every websocket response for responsive detection
                    method_speed = (
                        PlaybackSpeedPercent(context.method_speed)
                        if context.method_speed is not None
                        else None
                    )
                    effective_speed = manager.get_effective_speed(
                        robot_id, method_speed=method_speed
                    )
                    effective_state = manager.get_effective_state(robot_id)
                    effective_direction = manager.get_effective_direction(robot_id)

                    # Handle speed changes
                    if effective_speed != last_sent_speed:
                        logger.info(
                            f"Runtime playback speed change: {last_sent_speed}% -> {effective_speed}%"
                        )
                        last_sent_speed = effective_speed
                        # Queue the speed change request for transmission
                        speed_request = wb.models.ExecuteTrajectoryRequest(
                            wb.models.PlaybackSpeedRequest(
                                playback_speed_in_percent=effective_speed
                            )
                        )
                        logger.debug(f"Queuing speed change request: {effective_speed}%")
                        await request_queue.put(speed_request)

                    # Handle state changes (pause/resume)
                    if effective_state != last_sent_state:
                        logger.info(
                            f"Runtime playback state change: {last_sent_state.value} -> {effective_state.value}"
                        )
                        last_sent_state = effective_state

                        if effective_state == PlaybackState.PAUSED:
                            # Queue pause request (wrapped in ExecuteTrajectoryRequest)
                            pause_request = wb.models.ExecuteTrajectoryRequest(
                                wb.models.PauseMovementRequest()
                            )
                            logger.debug("Queuing pause movement request")
                            await request_queue.put(pause_request)
                        else:  # PlaybackState.PLAYING
                            # Queue resume request (using StartMovementRequest)
                            # Use the current effective direction for resume
                            set_io_list = context.combined_actions.to_set_io()
                            wb_direction = (
                                wb.models.Direction.DIRECTION_FORWARD
                                if effective_direction == PlaybackDirection.FORWARD
                                else wb.models.Direction.DIRECTION_BACKWARD
                            )
                            resume_request = wb.models.StartMovementRequest(
                                set_ios=set_io_list,
                                start_on_io=None,
                                pause_on_io=None,
                                direction=wb_direction,
                            )
                            logger.debug(
                                f"Queuing resume movement request with direction: {effective_direction.value}"
                            )
                            await request_queue.put(resume_request)
                            # Update last sent direction since we're resuming with new direction
                            last_sent_direction = effective_direction
                            # Reset the pending direction logged flag since we've applied the change
                            pending_direction_logged = False

                    # Handle direction changes (only effective when paused, applied on next resume)
                    elif (
                        effective_direction != last_sent_direction and not pending_direction_logged
                    ):
                        logger.info(
                            f"Runtime playback direction change: {last_sent_direction.value} -> {effective_direction.value} (will apply on next resume)"
                        )
                        # Don't update last_sent_direction here - it will be updated when we actually resume
                        # This just logs the direction change for user awareness
                        pending_direction_logged = True  # Mark that we've logged this change

                    # Stop when standstill indicates motion ended
                    if isinstance(instance, wb.models.Standstill):
                        if (
                            instance.standstill.reason
                            == wb.models.StandstillReason.REASON_MOTION_ENDED
                        ):
                            logger.info("Motion completed")
                            # Set execution state to indicate movement has ended
                            manager.set_execution_state(robot_id, PlaybackState.IDLE)
                            motion_completed.set()
                            break

            except Exception as e:
                logger.error(f"Error in response processor: {e}")
                motion_completed.set()

        async def speed_monitor():
            """
            Event-driven speed, state, and direction monitor for immediate response to external changes.

            This provides near-instantaneous speed change, pause/resume, and direction detection for responsive
            UI controls like VS Code sliders, pause/resume buttons, and direction controls, avoiding polling delays that
            could make the UI feel sluggish.
            """
            nonlocal last_sent_speed, last_sent_state, last_sent_direction, pending_direction_logged

            try:
                # Log initialization info for debugging
                logger.debug(f"Speed monitor using manager: {id(manager)} (type: {type(manager)})")
                logger.debug(f"Speed monitor using robot_id: {robot_id!r} (type: {type(robot_id)})")

                check_count = 0
                while not motion_completed.is_set():
                    # Check immediately, then wait briefly to allow other coroutines to run
                    method_speed = (
                        PlaybackSpeedPercent(context.method_speed)
                        if context.method_speed is not None
                        else None
                    )
                    effective_speed = manager.get_effective_speed(
                        robot_id, method_speed=method_speed
                    )
                    effective_state = manager.get_effective_state(robot_id)
                    effective_direction = manager.get_effective_direction(robot_id)

                    # Detect speed changes immediately
                    if effective_speed != last_sent_speed:
                        logger.info(
                            f"Speed monitor detected runtime playback speed change: {last_sent_speed}% -> {effective_speed}%"
                        )
                        last_sent_speed = effective_speed
                        # Queue the speed change request for immediate transmission
                        speed_request = wb.models.ExecuteTrajectoryRequest(
                            wb.models.PlaybackSpeedRequest(
                                playback_speed_in_percent=effective_speed
                            )
                        )
                        logger.debug(
                            f"Speed monitor queuing speed change request: {effective_speed}%"
                        )
                        await request_queue.put(speed_request)

                    # Detect state changes immediately
                    if effective_state != last_sent_state:
                        logger.info(
                            f"Speed monitor detected runtime playback state change: {last_sent_state.value} -> {effective_state.value}"
                        )
                        last_sent_state = effective_state

                        if effective_state == PlaybackState.PAUSED:
                            # Queue pause request (wrapped in ExecuteTrajectoryRequest)
                            pause_request = wb.models.ExecuteTrajectoryRequest(
                                wb.models.PauseMovementRequest()
                            )
                            logger.debug("Speed monitor queuing pause movement request")
                            await request_queue.put(pause_request)
                        else:  # PlaybackState.PLAYING
                            # Queue resume request (using StartMovementRequest)
                            # Use the current effective direction for resume
                            set_io_list = context.combined_actions.to_set_io()
                            wb_direction = (
                                wb.models.Direction.DIRECTION_FORWARD
                                if effective_direction == PlaybackDirection.FORWARD
                                else wb.models.Direction.DIRECTION_BACKWARD
                            )
                            resume_request = wb.models.StartMovementRequest(
                                set_ios=set_io_list,
                                start_on_io=None,
                                pause_on_io=None,
                                direction=wb_direction,
                            )
                            logger.debug(
                                f"Speed monitor queuing resume movement request with direction: {effective_direction.value}"
                            )
                            await request_queue.put(resume_request)
                            # Update last sent direction since we're resuming with new direction
                            last_sent_direction = effective_direction
                            # Reset the pending direction logged flag since we've applied the change
                            pending_direction_logged = False

                    # Detect direction changes immediately (only effective when paused, applied on next resume)
                    elif (
                        effective_direction != last_sent_direction and not pending_direction_logged
                    ):
                        logger.info(
                            f"Speed monitor detected runtime playback direction change: {last_sent_direction.value} -> {effective_direction.value} (will apply on next resume)"
                        )
                        # Don't update last_sent_direction here - it will be updated when we actually resume
                        # This just logs the direction change for user awareness
                        pending_direction_logged = True  # Mark that we've logged this change

                    # Brief yield to allow other coroutines to run, much faster than 50ms polling
                    await asyncio.sleep(0.001)  # 1ms yield instead of 50ms polling

                    check_count += 1
                    # Periodic debug logging to monitor speed state (less frequent now)
                    if check_count % 1000 == 0:  # Every 1000 checks (~1 second) instead of every 20
                        with manager._lock:
                            external_override = manager._external_overrides.get(robot_id)
                            decorator_default = manager._decorator_defaults.get(robot_id)
                        logger.debug(
                            f"Speed monitor check #{check_count}: effective_speed={effective_speed}%, effective_state={effective_state.value}, effective_direction={effective_direction.value}, last_sent_speed={last_sent_speed}%, last_sent_state={last_sent_state.value}, last_sent_direction={last_sent_direction.value}, external_override={external_override}, decorator_default={decorator_default}, method_speed={method_speed}"
                        )

            except Exception as e:
                logger.error(f"Error in speed monitor: {e}")
                motion_completed.set()

        async def request_generator():
            """Generate websocket requests: initialization, initial speed, start, and queued runtime changes."""
            try:
                # 1. Initialize movement
                yield wb.models.InitializeMovementRequest(
                    trajectory=context.motion_id, initial_location=0
                )

                # 2. Set initial playback speed
                yield wb.models.ExecuteTrajectoryRequest(
                    wb.models.PlaybackSpeedRequest(
                        playback_speed_in_percent=context.effective_speed
                    )
                )
                logger.info(f"Initial playback speed set to: {context.effective_speed}%")

                # 3. Start the movement
                set_io_list = context.combined_actions.to_set_io()
                yield wb.models.StartMovementRequest(
                    set_ios=set_io_list, start_on_io=None, pause_on_io=None
                )

                # 4. Continuously yield any queued requests (like runtime speed changes)
                while not motion_completed.is_set():
                    try:
                        # Wait for queued requests with a timeout to allow checking motion_completed
                        request = await asyncio.wait_for(request_queue.get(), timeout=0.1)
                        logger.debug(f"Yielding queued request: {type(request)}")
                        yield request
                    except asyncio.TimeoutError:
                        # No queued requests, continue monitoring
                        continue

            except asyncio.CancelledError:
                logger.info("Request generator cancelled")
            except Exception as e:
                logger.error(f"Error in request generator: {e}")

        # Start concurrent tasks for response processing and speed monitoring
        response_task = asyncio.create_task(response_processor())
        speed_monitor_task = asyncio.create_task(speed_monitor())

        try:
            # Yield requests from the generator
            async for request in request_generator():
                yield request
        finally:
            # Ensure proper cleanup of all tasks and execution state
            manager.set_execution_state(robot_id, PlaybackState.IDLE)
            motion_completed.set()
            response_task.cancel()
            speed_monitor_task.cancel()
            try:
                await response_task
            except asyncio.CancelledError:
                pass
            try:
                await speed_monitor_task
            except asyncio.CancelledError:
                pass

    return movement_controller
