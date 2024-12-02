import asyncio

from wandelbots.core.nova import Nova


async def main():
    nova = Nova()
    cell = nova.cell()
    async with cell.controller("ur10e") as controller:
        motion_group = controller.get_motion_group()

        # Current motion group state
        state = await motion_group.get_state("Flange")
        print(state)

        # Current joints positions
        joints = await motion_group.joints("Flange")
        print(joints)

        # Current TCP pose
        tcp_pose = await motion_group.tcp_pose("Flange")
        print(tcp_pose)



if __name__ == "__main__":
    asyncio.run(main())
