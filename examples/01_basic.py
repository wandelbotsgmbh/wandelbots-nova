import asyncio

from nova import Nova


async def main():
    nova = Nova(host="172.30.0.135")
    cell = nova.cell()
    controllers = await cell.controllers()
    controller = controllers[0]
    motion_group = controller[0]

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
