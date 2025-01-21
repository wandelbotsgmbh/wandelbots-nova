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
        ur10 = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        ur5 = await cell.ensure_virtual_robot_controller(
            "ur5",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR5E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        await asyncio.gather(move_robot(ur5), move_robot(ur10))
        await cell.delete_robot_controller(ur5.name)
        await cell.delete_robot_controller(ur10.name)


if __name__ == "__main__":
    asyncio.run(main())
