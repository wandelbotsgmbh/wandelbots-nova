import asyncio
from math import pi

from nova import MotionGroup, Nova
from nova.actions import ptp
from nova.types import Pose

"""
Example: Move multiple robots to perform coordinated movements.

Prerequisites:
- A cell with two robots named "ur" and "kuka".
"""


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

    await motion_group.plan_and_execute(actions, tcp=tcp)


async def main():
    nova = Nova()
    cell = nova.cell()
    ur = await cell.controller("ur")
    kuka = await cell.controller("kuka")
    tcp = "Flange"

    flange_state = await ur[0].get_state(tcp)
    print(flange_state)

    # activate all motion groups
    async with ur:
        await move_robot(ur.motion_group(0), tcp)

    # activate motion group 0
    async with ur.motion_group(0) as mg_0:
        await move_robot(mg_0, tcp)

    # activate motion group 0
    async with ur[0] as mg_0:
        await move_robot(mg_0, tcp)

    # activate motion group 0 from two different controllers
    async with ur[0] as ur_0_mg, kuka[0] as kuka_0_mg:
        await asyncio.gather(move_robot(ur_0_mg, tcp), move_robot(kuka_0_mg, tcp))

    # activate motion group 0 from two different controllers
    mg_0 = ur.motion_group(0)
    mg_1 = kuka.motion_group(0)
    async with mg_0, mg_1:
        await asyncio.gather(move_robot(mg_0, tcp), move_robot(mg_1, tcp))

    await nova.close()


if __name__ == "__main__":
    asyncio.run(main())
