from wandelbots import use_nova_access_token, Controller
from decouple import config
import asyncio


async def main():
    nova = use_nova_access_token(config("NOVA_HOST"), access_token=config("NOVA_ACCESS_TOKEN"))
    controller = Controller(nova, cell=config("CELL_ID"), controller_host=config("CONTROLLER_HOST"))

    # Connect to the controller and activate motion groups
    async with controller:
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
