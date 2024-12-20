"""
This example demonstrates how to activate specific motion groups for two robot controllers
and execute simultaneous movements for both robots.

The robots used in this example are:
- A Universal Robots (UR) controller
- A KUKA controller

Each robot moves between a predefined home pose and a target pose sequentially.
"""

from math import pi
from nova import Nova, MotionGroup
from nova.types import Pose
from nova.actions import ptp
import asyncio


async def move_robot(motion_group: MotionGroup, tcp: str):
    home_pose = Pose((200, 200, 600, 0, pi, 0))
    target_pose = home_pose @ (100, 0, 0, 0, 0, 0)
    actions = [
        ptp(home_pose),
        ptp(target_pose),
        ptp(target_pose @ (0, 0, 100, 0, 0, 0)),
        ptp(target_pose @ (0, 100, 0, 0, 0, 0)),
        ptp(home_pose),
    ]

    await motion_group.run(actions, tcp=tcp)


async def main():
    nova = Nova()
    cell = nova.cell()
    ur = await cell.controller("ur")
    kuka = await cell.controller("kuka")
    tcp = "Flange"

    flange_state = await ur[0].get_state(tcp=tcp)
    print(flange_state)

    # activate all motion groups
    async with ur:
        await move_robot(ur.motion_group(0))

    # activate motion group 0
    async with ur.motion_group(0) as mg_0:
        await move_robot(mg_0)

    # activate motion group 0
    async with ur[0] as mg_0:
        await move_robot(mg_0)

    # activate motion group 0 from two different controllers
    async with ur[0] as ur_0_mg, kuka[0] as kuka_0_mg:
        await asyncio.gather(move_robot(ur_0_mg, tcp), move_robot(kuka_0_mg, tcp))

    # activate motion group 0 from two different controllers
    mg_0 = ur.motion_group(0)
    mg_1 = kuka.motion_group(0)
    async with mg_0, mg_1:
        await asyncio.gather(move_robot(mg_0, tcp), move_robot(mg_1, tcp))


if __name__ == "__main__":
    asyncio.run(main())
