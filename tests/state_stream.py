import asyncio

from decouple import config
from icecream import ic

from nova import Nova
from nova.core.robot_cell import RobotCell

NOVA_API = config("NOVA_API")


async def main():
    async with Nova(host=NOVA_API) as nova:
        cell = nova.cell()
        controller = await cell.controller("ur")
        ic(controller)

        rc = RobotCell(ur=controller)
        async for rcs in rc.stream_state(500):
            ic(rcs)


if __name__ == "__main__":
    asyncio.run(main())
