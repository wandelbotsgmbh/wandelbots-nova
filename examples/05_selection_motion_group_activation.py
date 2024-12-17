"""
This example demonstrates how to activate specific motion groups for two robot controllers
and execute simultaneous movements for both robots.

The robots used in this example are:
- A Universal Robots (UR) controller
- A KUKA controller

Each robot moves between a predefined home pose and a target pose sequentially.
"""

from contextlib import AsyncExitStack
from math import pi
from nova import Nova, ptp, MotionGroup, Pose
import asyncio


async def move_robot(motion_group: MotionGroup):
    home_pose = Pose((200, 200, 600, 0, pi, 0))
    target_pose = home_pose @ (100, 0, 0, 0, 0, 0)
    actions = [ptp(home_pose), ptp(target_pose), ptp(home_pose)]

    await motion_group.run(actions, tcp="Flange")


async def main():
    nova = Nova()
    cell = nova.cell()
    ur = await cell.controller("ur")
    kuka = await cell.controller("kuka")

    ur_0_mg = ur.get_motion_group()
    kuka_0_mg = kuka.get_motion_group()

    async with AsyncExitStack() as stack:
        await stack.enter_async_context(ur_0_mg)
        await stack.enter_async_context(kuka_0_mg)

        await asyncio.gather(move_robot(ur_0_mg), move_robot(kuka_0_mg))


if __name__ == "__main__":
    asyncio.run(main())
