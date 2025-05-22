import asyncio
from icecream import ic

from nova import Nova
from nova.api import models
from nova.events import Cycle


async def main():
    async with Nova() as nova:
        ch = Cycle(nova.cell().cell_id)
        ic()
        await ch.start()
        await asyncio.sleep(1)
        await ch.finish()
        
        await ch.start()
        await asyncio.sleep(.5)
        await ch.fail("Test failure")

        async with Cycle(nova.cell().cell_id):
            await asyncio.sleep(1)
        
        async with ch:
            await asyncio.sleep(1)
            raise RuntimeError("Test failure")
        
        ic()



if __name__ == "__main__":
    asyncio.run(main())