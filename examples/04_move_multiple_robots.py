import asyncio

from nova import Controller, Nova
from nova.api import models
from nova.actions import jnt, ptp

"""
Example: Move multiple robots simultaneously.

Prerequisites:
- A cell with two robots: one named "ur" and another named "kuka".
"""


async def move_robot(controller: Controller):
    async with controller[0] as motion_group:
        home_joints = await motion_group.joints()
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        current_pose = await motion_group.tcp_pose(tcp)
        target_pose = current_pose @ (100, 0, 0, 0, 0, 0)
        actions = [jnt(home_joints), ptp(target_pose), jnt(home_joints)]

        await motion_group.plan_and_execute(actions, tcp)


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        ur = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        kuka = await cell.ensure_virtual_robot_controller(
            "kuka", models.VirtualControllerTypes.KUKA_MINUS_KR6_R700_2, models.Manufacturer.KUKA
        )
        await asyncio.gather(move_robot(ur), move_robot(kuka))
        await cell.delete_robot_controller(ur.name)
        await cell.delete_robot_controller(kuka.name)


if __name__ == "__main__":
    asyncio.run(main())
