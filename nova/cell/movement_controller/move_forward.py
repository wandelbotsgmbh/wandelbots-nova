import asyncio
import logging

from nova import api
from nova.actions import MovementControllerContext
from nova.actions.async_action import AsyncAction, ErrorHandlingMode
from nova.cell.movement_controller.async_action_executor import AsyncActionExecutor
from nova.exceptions import ErrorDuringMovement, InitMovementFailed
from nova.types import (
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MovementControllerFunction,
    Pose,
)
from nova.types.state import RobotState

logger = logging.getLogger(__name__)

_START_STATE_MONITOR_TIMEOUT = 5.0


# TODO: when the message exchange is not working as expected we should gracefully close
# TODO: add the set_io functionality Thorsten Blatter did for the force torque sensor at Schaeffler
def move_forward(context: MovementControllerContext) -> MovementControllerFunction:
    """
    movement_controller is an async function that yields requests to the server.
    If a movement_consumer is provided, we'll asend() each wb.models.MovementMovement to it,
    letting it produce MotionState objects.
    """
    # Initialize async action executor if there are async actions
    async_actions = context.combined_actions.get_async_actions()
    executor: AsyncActionExecutor | None = None

    if async_actions:
        # Check for blocking actions and warn (move_forward doesn't truly pause motion)
        blocking_actions = [
            al for al in async_actions if isinstance(al.action, AsyncAction) and al.action.blocking
        ]
        if blocking_actions:
            logger.warning(
                f"Trajectory {context.motion_id} has {len(blocking_actions)} blocking async action(s). "
                "move_forward controller executes blocking actions but cannot pause robot motion. "
                "For true motion pause during blocking actions, use TrajectoryCursor instead."
            )

        executor = context.async_action_executor or AsyncActionExecutor(
            motion_group_id=context.motion_id,
            async_actions=async_actions,
            error_mode=ErrorHandlingMode.COLLECT,  # Don't raise during motion
        )
        logger.info(
            f"Trajectory {context.motion_id}: initialized executor with "
            f"{len(async_actions)} async action(s)"
        )

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

                    # Execute async actions based on location
                    if (
                        executor
                        and motion_group_state.execute
                        and isinstance(
                            motion_group_state.execute.details, api.models.TrajectoryDetails
                        )
                    ):
                        current_location = motion_group_state.execute.details.location.root

                        # Build RobotState from motion_group_state
                        tcp_pose = motion_group_state.tcp_pose
                        pose = Pose(tcp_pose) if tcp_pose else Pose((0, 0, 0, 0, 0, 0))
                        robot_state = RobotState(
                            pose=pose,
                            tcp=motion_group_state.tcp,
                            joints=tuple(motion_group_state.joint_position.joints),
                        )

                        try:
                            await executor.check_and_trigger(
                                current_location=current_location, current_state=robot_state
                            )
                        except Exception as e:
                            logger.error(f"Trajectory {context.motion_id}: async action error: {e}")
                            # Continue execution - errors are collected in executor

                    if trajectory_ended and motion_group_state.standstill:
                        logger.info(
                            f"Trajectory: {context.motion_id} state monitor detected standstill."
                        )
                        # Wait for remaining async actions before returning
                        if executor:
                            await executor.wait_for_all_actions()
                            summary = executor.get_summary()
                            logger.info(
                                f"Trajectory {context.motion_id}: async actions completed - "
                                f"{summary['succeeded']} succeeded, {summary['failed']} failed"
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
                # Ensure any remaining actions complete
                if executor:
                    await executor.wait_for_all_actions()
            except asyncio.CancelledError:
                # Cancel pending async actions on cancellation
                if executor:
                    await executor.cancel_all_actions()
                raise
            except BaseException as e:
                logger.error(
                    f"Trajectory: {context.motion_id} state monitor ended with exception: {type(e).__name__}: {e}"
                )
                if executor:
                    await executor.cancel_all_actions()
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
