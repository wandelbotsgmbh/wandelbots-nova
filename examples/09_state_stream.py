"""
Example: Obtain and print a state stream info from a robot cell.
"""

import asyncio
from argparse import ArgumentParser
from contextlib import suppress

from nova import Nova
from nova.cell.robot_cell import RobotCell


async def main(controller_name: str = "controller") -> None:
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller(controller_name)

        rc = RobotCell(**{controller_name: controller})
        async for controller_state in rc.stream_state(rate_msecs=500):
            print(controller_state)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument(
        "--controller", type=str, default="controller", help="Name of the controller"
    )
    args = parser.parse_args()

    with suppress(KeyboardInterrupt):
        asyncio.run(main(controller_name=args.controller))
