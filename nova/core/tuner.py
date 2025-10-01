import asyncio
from typing import Optional

import pydantic
from faststream import FastStream
from faststream.nats import NatsBroker

from nova.core import logger

from .movement_controller import MotionEvent, TrajectoryCursor, motion_started


class TrajectoryTuner:
    def __init__(self, actions, plan_fn, execute_fn):
        self.actions = actions
        self.plan_fn = plan_fn
        self.execute_fn = execute_fn

    async def tune(self):
        finished_tuning = False
        continue_tuning_event = asyncio.Event()
        faststream_app_ready_event = asyncio.Event()
        last_operation_result = None  # TODO implement this feature

        broker = NatsBroker()

        @broker.subscriber("trajectory-cursor")
        async def controller_handler(command: str, speed: Optional[pydantic.PositiveInt] = None):
            nonlocal last_operation_result, finished_tuning
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

        async def runtime_monitor(interval=0.5):
            start_time = asyncio.get_event_loop().time()
            while True:
                elapsed = asyncio.get_event_loop().time() - start_time
                logger.warning(f"{elapsed:.2f}s")
                await asyncio.sleep(interval)

        faststream_app = FastStream(broker)

        @faststream_app.after_startup
        async def after_startup():
            faststream_app_ready_event.set()

        faststream_app_task = asyncio.create_task(faststream_app.run())
        # runtime_task = asyncio.create_task(runtime_monitor(0.5))  # Output every 0.5 seconds

        @motion_started.connect
        async def on_motion_started(sender, event: MotionEvent):
            await broker.publish(
                # {"line": event.target_action.metas["line_number"]}, "editor.line.select"
                event,
                "editor.motion-event",
            )

        @broker.subscriber("editor.movement.options")
        async def movement_options_handler(msg):
            logger.info(f"Received movement options: {msg}")

        try:
            # tuning loop
            await faststream_app_ready_event.wait()
            current_location = 0.0
            while not finished_tuning:
                # TODO this plans the second time for the same actions when we get here because
                # the initial joint trajectory was already planned before the MotionGroup._execute call
                motion_id, joint_trajectory = await self.plan_fn(self.actions)
                current_cursor = TrajectoryCursor(
                    motion_id,
                    joint_trajectory,
                    self.actions,
                    initial_location=current_location,
                    detach_on_standstill=True,
                )
                # wait for user to send next command
                logger.info("Cursor initialized. Waiting for user command...")
                # publish movement options
                await broker.publish(
                    {"options": list(current_cursor.get_movement_options())},
                    "editor.movement.options",
                )

                await continue_tuning_event.wait()
                execution_task = asyncio.create_task(
                    self.execute_fn(client_request_generator=current_cursor.cntrl)
                )
                async for execute_response in current_cursor:
                    yield execute_response
                await execution_task
                continue_tuning_event.clear()
                current_location = (
                    current_cursor._current_location
                )  # TODO is this the cleanest way to get the current location?

                # somehow obtain the modified actions for the next iteration

            # runtime_task.cancel()
            # await runtime_task
        except asyncio.CancelledError:
            logger.debug(
                f"TrajectoryTuner main loop was cancelled during cleanup. "
                f"finished_tuning={finished_tuning}, current_location={current_location}, last_operation_result={last_operation_result}"
            )
            pass
        faststream_app.exit()
        await faststream_app_task
