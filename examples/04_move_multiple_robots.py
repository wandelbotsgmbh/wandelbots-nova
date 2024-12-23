from nova import Nova, Controller, speed_up_movement_controller
from nova.actions import ptp, jnt
from math import pi
import asyncio


async def move_robot(controller: Controller):
    home_joints = (0, -pi / 4, -pi / 4, -pi / 4, pi / 4, 0)

    async with controller[0] as motion_group:
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        current_pose = await motion_group.tcp_pose(tcp)
        target_pose = current_pose @ (100, 0, 0, 0, 0, 0)
        actions = [jnt(home_joints), ptp(target_pose), jnt(home_joints)]

        await motion_group.run(actions, tcp=tcp, movement_controller=speed_up_movement_controller)


async def main():
    nova = Nova()
    cell = nova.cell()
    ur = await cell.controller("ur")
    kuka = await cell.controller("kuka")

    await asyncio.gather(move_robot(ur), move_robot(kuka))


if __name__ == "__main__":
    asyncio.run(main())
