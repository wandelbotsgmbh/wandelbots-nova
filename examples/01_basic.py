import asyncio

from nova import Nova
from nova.api import models

"""
Example: Getting the current state of a robot.

Prerequisites:
- At least one robot added to the cell.
"""


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        async with controller[0] as motion_group:
            tcp_names = await motion_group.tcp_names()
            print(tcp_names)
    
            tcp = tcp_names[0]

            # Current motion group state
            state = await motion_group.get_state(tcp)
            print(state)

            # Current joints positions
            joints = await motion_group.joints()
            print(joints)

            # Current TCP pose
            tcp_pose = await motion_group.tcp_pose(tcp)
            print(tcp_pose)

        await cell.delete_robot_controller(controller.name)


if __name__ == "__main__":
    asyncio.run(main())
