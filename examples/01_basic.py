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
        # TODO: add a controller
        await cell.add_virtual_controller(
            "ur", models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E
        )
        await asyncio.sleep(30)

        controllers = await cell.controllers()
        print(controllers)
        controller = controllers[0]

        async with controller:
            activated_motion_group_ids = await controller.activated_motion_group_ids()
            print(activated_motion_group_ids)

        motion_group = controller[0]

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


if __name__ == "__main__":
    asyncio.run(main())
