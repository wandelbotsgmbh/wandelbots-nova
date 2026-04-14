import asyncio
import logging
from dataclasses import dataclass
from typing import Union

from nova import api
from nova.actions import MovementControllerContext
from nova.cell.movement_controller.trajectory_state_machine import TrajectoryExecutionMachine
from nova.exceptions import ErrorDuringMovement, InitMovementFailed
from nova.types import (
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MovementControllerFunction,
)
from nova.types.state import robot_state_from_motion_group_state

logger = logging.getLogger(__name__)

_START_STATE_MONITOR_TIMEOUT = 5.0


@dataclass(frozen=True)
class _QueueSentinel:
    """Sentinel value to signal the end of the command queue."""


_QUEUE_SENTINEL = _QueueSentinel()

_CommandQueueItem = Union[
    api.models.StartMovementRequest, api.models.PauseMovementRequest, _QueueSentinel
]


def move_forward(context: MovementControllerContext) -> MovementControllerFunction:
    """Movement controller that executes a trajectory forward to completion.

    Sends requests to the motion controller via an async generator. Supports async actions
    through the executor from context. For blocking async actions, sends PauseMovementRequest
    to the robot controller, executes the action, then resumes with StartMovementRequest.
    """
    executor = context.async_action_executor

    async def movement_controller(
        response_stream: ExecuteTrajectoryResponseStream,
    ) -> ExecuteTrajectoryRequestStream:
        command_queue: asyncio.Queue[_CommandQueueItem] = asyncio.Queue()
        movement_error: Exception | None = None

        # Wire executor pause/resume callbacks to the command queue
        if executor:

            async def pause_motion():
                command_queue.put_nowait(api.models.PauseMovementRequest())

            async def resume_motion():
                command_queue.put_nowait(
                    api.models.StartMovementRequest(
                        direction=api.models.Direction.DIRECTION_FORWARD
                    )
                )

            executor._pause_callback = pause_motion
            executor._resume_callback = resume_motion

        async def state_monitor(stream_started_event: asyncio.Event):
            try:
                logger.info("Starting state monitor for trajectory")
                machine = TrajectoryExecutionMachine()
                machine.send("start")

                async for motion_group_state in context.motion_group_state_stream_gen():
                    if not stream_started_event.is_set():
                        stream_started_event.set()

                    logger.debug(
                        f"Trajectory: {context.motion_id} state monitor received state: "
                        f"{motion_group_state}"
                    )

                    result = machine.process_motion_state(motion_group_state)

                    if executor and result.location is not None:
                        robot_state = robot_state_from_motion_group_state(motion_group_state)
                        try:
                            await executor.check_and_trigger(
                                current_location=result.location, current_state=robot_state
                            )
                        except Exception as e:
                            logger.error(f"Trajectory {context.motion_id}: async action error: {e}")

                    if result.skip:
                        continue

                    if machine.is_ended or machine.is_paused:
                        terminal_state = "ended" if machine.is_ended else "paused"
                        logger.info(
                            f"Trajectory: {context.motion_id} state monitor reached "
                            f"{terminal_state} at standstill."
                        )
                        if executor:
                            await executor.wait_for_all_actions()
                            summary = executor.get_summary()
                            logger.info(
                                f"Trajectory {context.motion_id}: async actions completed - "
                                f"{summary['succeeded']} succeeded, {summary['failed']} failed"
                            )
                        return

                    if machine.is_waiting_for_standstill:
                        logger.debug(f"Trajectory: {context.motion_id} waiting for standstill")

                logger.info(
                    f"Trajectory: {context.motion_id} state monitor ended "
                    "without terminal trajectory state"
                )
                if executor:
                    await executor.wait_for_all_actions()
            except asyncio.CancelledError:
                if executor:
                    await executor.cancel_all_actions()
                raise
            except BaseException as e:
                logger.error(
                    f"Trajectory: {context.motion_id} state monitor ended with "
                    f"exception: {type(e).__name__}: {e}"
                )
                if executor:
                    await executor.cancel_all_actions()
                raise

        async def response_consumer():
            """Consume responses from the motion controller websocket.

            Handles StartMovementResponse and PauseMovementResponse (from pause/resume cycles)
            and detects MovementErrorResponse.
            """
            nonlocal movement_error
            async for response in response_stream:
                match response.root:
                    case api.models.StartMovementResponse() | api.models.PauseMovementResponse():
                        logger.debug(
                            f"Trajectory: {context.motion_id} received "
                            f"{type(response.root).__name__}"
                        )
                    case api.models.PlaybackSpeedResponse():
                        pass
                    case api.models.MovementErrorResponse():
                        movement_error = ErrorDuringMovement(
                            f"Error occurred during trajectory execution: {response.root.message}"
                        )
                        return
                    case _:
                        logger.error(
                            f"Trajectory: {context.motion_id} received unexpected "
                            f"response: {response}"
                        )

        # === Initialize movement ===
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
                f"Trajectory: {context.motion_id} initialization failed: "
                f"{initialize_movement_response.root}"
            )
            raise InitMovementFailed(initialize_movement_response.root)

        # === Start state monitor before sending start to not lose any data ===
        try:
            stream_started_event = asyncio.Event()
            state_monitor_task = asyncio.create_task(
                state_monitor(stream_started_event),
                name=f"motion-group-state-monitor-{context.motion_id}",
            )
            logger.info(f"Trajectory: {context.motion_id} waiting for state monitor to start")
            await asyncio.wait_for(
                stream_started_event.wait(), timeout=_START_STATE_MONITOR_TIMEOUT
            )
        except asyncio.TimeoutError:
            logger.error(f"Trajectory: {context.motion_id} state monitor failed to start in time")
            state_monitor_task.cancel()
            raise Exception("State monitor failed to start in time")

        # === Start response consumer ===
        response_consumer_task = asyncio.create_task(
            response_consumer(), name=f"response-consumer-{context.motion_id}"
        )

        # === Enqueue initial start command ===
        set_io_list = context.combined_actions.to_set_io()
        command_queue.put_nowait(
            api.models.StartMovementRequest(
                direction=api.models.Direction.DIRECTION_FORWARD,
                set_outputs=set_io_list,
                start_on_io=context.start_on_io,
                pause_on_io=context.pause_on_io,
            )
        )

        # === Watch for completion and signal the command queue ===
        async def completion_watcher():
            await asyncio.wait(
                {state_monitor_task, response_consumer_task}, return_when=asyncio.FIRST_COMPLETED
            )
            command_queue.put_nowait(_QUEUE_SENTINEL)

        watcher_task = asyncio.create_task(
            completion_watcher(), name=f"completion-watcher-{context.motion_id}"
        )

        tasks = {state_monitor_task, response_consumer_task, watcher_task}

        try:
            # === Yield commands until a task completes ===
            logger.info(f"Trajectory: {context.motion_id} entering command loop")
            while True:
                command = await command_queue.get()
                if isinstance(command, _QueueSentinel):
                    break
                logger.info(f"Trajectory: {context.motion_id} sending {type(command).__name__}")
                yield command

            # === Check outcome ===
            if movement_error:
                raise movement_error

            if state_monitor_task.done():
                logger.info(f"Trajectory: {context.motion_id} completed via state monitor")
                state_monitor_task.result()  # re-raise any exception from state monitor
                return

            if response_consumer_task.done():
                logger.info(f"Trajectory: {context.motion_id} response consumer completed")
                return
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
