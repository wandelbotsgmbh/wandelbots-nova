import asyncio
import json
import logging
from typing import Optional

import pydantic

from .movement_controller.trajectory_cursor import MotionEvent, TrajectoryCursor, motion_started

logger = logging.getLogger(__name__)


class TrajectoryTuner:
    """Coordinate interactive trajectory tuning via NATS commands.

    The ``TrajectoryTuner`` orchestrates planning and execution of motion
    trajectories while allowing an external controller (e.g. UI or tool)
    to step through, play back, and adjust the motion via NATS messages.

    At a high level, the tuner:

    * Uses ``plan_fn`` to generate a trajectory from a set of input actions.
    * Uses ``execute_fn`` to execute the planned motion on the robot.
    * Creates and manages a :class:`TrajectoryCursor` instance which controls
      playback (forward, backward, step-wise navigation, pause, detach, etc.).
    * Subscribes to the ``"trajectory-cursor"`` NATS subject to receive
      external commands: ``"forward"``, ``"backward"``, ``"step-forward"``,
      ``"step-backward"``, ``"pause"``, and ``"finish"``.
    * Publishes motion events to ``"editor.motion-event"`` and available
      movement options to ``"editor.movement.options"``.

    The tuner runs in a loop, re-planning and re-executing the trajectory
    each time the cursor is paused or reaches a boundary, until a ``"finish"``
    command is received.

    Parameters
    ----------
    plan_fn :
        Async callable that plans a trajectory from actions. Must return
        a tuple of ``(motion_id, joint_trajectory)``.
    execute_fn :
        Async callable that executes motion segments. Called with a
        ``client_request_generator`` from the :class:`TrajectoryCursor`.

    Notes
    -----
    Requires a valid program context with a connected NATS client. Use within
    a ``@nova.program`` decorated function.

    Example
    -------
    .. code-block:: python

        tuner = TrajectoryTuner(plan_fn=plan_trajectory, execute_fn=execute_motion)
        async for response in tuner.tune(actions, motion_group_state_stream_fn):
            # handle execution responses
            pass
    """

    def __init__(self, plan_fn, execute_fn):
        self.plan_fn = plan_fn
        self.execute_fn = execute_fn

    async def tune(self, actions, motion_group_state_stream_fn):
        finished_tuning = False
        continue_tuning_event = asyncio.Event()
        last_operation_result = None  # TODO implement this feature
        subscriptions = []

        # Import here to avoid circular import
        from nova import get_current_program_context

        ctx = get_current_program_context()
        if ctx is None:
            raise RuntimeError("TrajectoryTuner requires a valid program context")
        nats = ctx.nova.nats
        if nats is None or not nats.is_connected:
            raise RuntimeError("TrajectoryTuner requires a connected NATS client")

        current_cursor: Optional[TrajectoryCursor] = None

        async def controller_handler(msg):
            nonlocal last_operation_result, finished_tuning, current_cursor
            try:
                data = json.loads(msg.data.decode())
                command = data.get("command")
                speed = data.get("speed")
                if speed is not None:
                    speed = pydantic.PositiveInt(speed)
            except (json.JSONDecodeError, ValueError) as e:
                logger.warning(f"Invalid message format in trajectory-cursor: {e}")
                return

            if current_cursor is None:
                logger.warning("Received command but cursor is not initialized")
                return

            match command:
                case "forward":
                    continue_tuning_event.set()
                    current_cursor.forward(playback_speed_in_percent=speed)
                    # last_operation_result = await future
                case "step-forward":
                    continue_tuning_event.set()
                    current_cursor.forward_to_next_action(playback_speed_in_percent=speed)
                    # await future
                case "backward":
                    continue_tuning_event.set()
                    current_cursor.backward(playback_speed_in_percent=speed)
                    # await future
                case "step-backward":
                    continue_tuning_event.set()
                    current_cursor.backward_to_previous_action(playback_speed_in_percent=speed)
                    # await future
                case "pause":
                    current_cursor.pause()
                    # if future is not None:
                    #     await future
                case "finish":
                    # TODO only allow finishing if at forward end of trajectory
                    continue_tuning_event.set()
                    current_cursor.detach()
                    finished_tuning = True
                case _:
                    logger.warning(f"Unknown command received in trajectory-cursor: {command}")

        async def movement_options_handler(msg):
            try:
                data = json.loads(msg.data.decode())
                logger.info(f"Received movement options: {data}")
            except json.JSONDecodeError as e:
                logger.warning(f"Invalid JSON in movement options: {e}")

        # Subscribe to NATS subjects
        cursor_sub = await nats.subscribe("trajectory-cursor", cb=controller_handler)
        subscriptions.append(cursor_sub)
        options_sub = await nats.subscribe("editor.movement.options", cb=movement_options_handler)
        subscriptions.append(options_sub)

        @motion_started.connect
        async def on_motion_started(sender, event: MotionEvent):
            await nats.publish(
                subject="editor.motion-event", payload=event.model_dump_json().encode()
            )

        try:
            # tuning loop
            current_location = 0.0
            while not finished_tuning:
                # TODO this plans the second time for the same actions when we get here because
                # the initial joint trajectory was already planned before the MotionGroup._execute call
                motion_id, joint_trajectory = await self.plan_fn(actions)
                current_cursor = TrajectoryCursor(
                    motion_id,
                    # motion_group_state_stream_fn(response_rate_msecs=200),
                    motion_group_state_stream_fn(),
                    joint_trajectory,
                    actions,
                    initial_location=current_location,
                    detach_on_standstill=True,
                )
                # wait for user to send next command
                logger.info("Cursor initialized. Waiting for user command...")
                # publish movement options
                await nats.publish(
                    subject="editor.movement.options",
                    payload=json.dumps(
                        {"options": list(current_cursor.get_movement_options())}
                    ).encode(),
                )

                await continue_tuning_event.wait()
                execution_task = asyncio.create_task(
                    self.execute_fn(client_request_generator=current_cursor.cntrl)
                )
                async for execute_response in current_cursor:
                    yield execute_response
                current_cursor.detach()
                await execution_task
                continue_tuning_event.clear()
                current_location = (
                    current_cursor._current_location
                )  # TODO is this the cleanest way to get the current location?

                # somehow obtain the modified actions for the next iteration

        except asyncio.CancelledError:
            logger.debug(
                f"TrajectoryTuner main loop was cancelled during cleanup. "
                f"finished_tuning={finished_tuning}, current_location={current_location}, last_operation_result={last_operation_result}"
            )
        finally:
            # Clean up subscriptions
            for sub in subscriptions:
                await sub.unsubscribe()
