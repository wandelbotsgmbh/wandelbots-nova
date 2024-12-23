import asyncio
from nova import Nova


async def main():
    nova = Nova()
    cell = nova.cell()
    controllers = await cell.controllers()
    controller = controllers[0]
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
