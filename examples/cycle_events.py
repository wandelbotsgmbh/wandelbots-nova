import asyncio

import nova
from nova import Nova
from nova.events import Cycle


@nova.program()
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        # Track a process cycle in the cell.
        # This will generate events cycle_start on entering and cycle_finish on exiting
        # the context manager.
        async with Cycle(cell.cell_id):
            # Run some process
            await asyncio.sleep(1)

        # If the context manager is exited with an exception, it will generate a cycle_failed event.
        async with Cycle(cell.cell_id):
            await asyncio.sleep(0.5)
            raise Exception("Something went wrong")

        # Control the cycle manually
        cycle = Cycle(cell.cell_id)
        # start() returns the start time as a datetime
        start_time = await cycle.start()
        print(f"Cycle started at {start_time}")
        await asyncio.sleep(1)
        # finish() returns the cycle time as a timedelta
        cycle_time = await cycle.finish()
        print(f"Cycle finished in {cycle_time}")

        # The cycle can also be failed manually
        await cycle.start()
        try:
            await asyncio.sleep(0.5)
            raise Exception("Something went wrong")
        except Exception as e:
            await cycle.fail(e)


if __name__ == "__main__":
    asyncio.run(main())
