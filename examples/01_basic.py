import asyncio

from nova import Nova

"""
Example: Getting the current state of a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controllers = await cell.controllers()
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
