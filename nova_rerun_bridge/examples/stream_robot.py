"""
This example demonstrates how to stream the robot state into the timeline "time".
This can be used to record the actual motions of the robot.
"""

import asyncio
import signal
from contextlib import asynccontextmanager

import nova
from nova import Nova, api
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova_rerun_bridge import NovaRerunBridge


@asynccontextmanager
async def handle_shutdown():
    stop = asyncio.Future()

    def signal_handler():
        if not stop.done():
            stop.set_result(None)

    loop = asyncio.get_event_loop()
    loop.add_signal_handler(signal.SIGINT, signal_handler)
    try:
        yield stop
    finally:
        loop.remove_signal_handler(signal.SIGINT)


@nova.program(
    name="08_stream_robot",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        cell = nova.cell()
        controller = await cell.controller("ur10")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            # this streams the current robot state into the timeline "time"
            # use this to record actual motions of the robot
            await bridge.start_streaming(motion_group)

            # In addition to log the robot state you can still log other data
            # like trajectories into the "time_interval_0.016" timeline
            await bridge.log_saftey_zones(motion_group)

            # Keep streaming until Ctrl+C
            async with handle_shutdown() as stop:
                try:
                    await stop
                except asyncio.CancelledError:
                    pass
                finally:
                    await bridge.stop_streaming()


if __name__ == "__main__":
    asyncio.run(test())
