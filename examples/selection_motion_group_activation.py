"""
Example: Move multiple robots to perform coordinated movements.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio
from math import pi

import nova
from nova import MotionGroup, Nova
from nova.actions import cartesian_ptp
from nova.api import models
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose


async def move_robot(motion_group: MotionGroup, tcp: str):
    home_pose = Pose((200, 200, 600, 0, pi, 0))
    target_pose = home_pose @ (100, 0, 0, 0, 0, 0)
    actions = [
        cartesian_ptp(home_pose),
        cartesian_ptp(target_pose),
        cartesian_ptp(target_pose @ (0, 0, 100, 0, 0, 0)),
        cartesian_ptp(target_pose @ (0, 100, 0, 0, 0, 0)),
        cartesian_ptp(home_pose),
    ]

    await motion_group.plan_and_execute(actions, tcp=tcp)  # type: ignore


@nova.program(
    name="Selection Motion Group Activation",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type=models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            ),
            virtual_controller(
                name="ur5",
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type=models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR5E,
            ),
        ],
        cleanup_controllers=True,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        ur10 = await cell.controller("ur10")
        ur5 = await cell.controller("ur5")
        tcp = "Flange"

        flange_state = await ur10[0].get_state(tcp)
        print(flange_state)

        # activate all motion groups
        async with ur10:
            await move_robot(ur10.motion_group("0@ur10"), tcp)

        # activate motion group 0
        async with ur10.motion_group("0@ur10") as mg_0:
            await move_robot(mg_0, tcp)

        # activate motion group 0
        async with ur10[0] as mg_0:
            await move_robot(mg_0, tcp)

        # activate motion group 0 from two different controllers
        async with ur10[0] as ur_0_mg, ur5[0] as kuka_0_mg:
            await asyncio.gather(move_robot(ur_0_mg, tcp), move_robot(kuka_0_mg, tcp))

        # activate motion group 0 from two different controllers
        mg_0 = ur10.motion_group("0@ur10")
        mg_1 = ur5.motion_group("0@ur5")
        async with mg_0, mg_1:
            await asyncio.gather(move_robot(mg_0, tcp), move_robot(mg_1, tcp))


if __name__ == "__main__":
    asyncio.run(main())
