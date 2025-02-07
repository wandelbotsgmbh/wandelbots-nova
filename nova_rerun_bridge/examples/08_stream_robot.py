import asyncio
import signal
from contextlib import asynccontextmanager

from nova.api import models
from nova.core.nova import Nova

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


async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.start_streaming(motion_group)

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
