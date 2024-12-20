from nova import Nova
from nova.types import Pose
from math import pi

# TODO: update I/O interaction interface
from nova.actions import WriteAction, ptp, jnt
import asyncio


async def main():
    nova = Nova()
    cell = nova.cell()
    controllers = await cell.controllers()

    # Define a home position
    home_joints = (0, -pi / 4, -pi / 4, -pi / 4, pi / 4, 0)

    # Connect to the controller and activate motion groups
    async with controllers[0] as motion_group:
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose(tcp)
        target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))
        actions = [
            jnt(home_joints),
            # TODO: controller.write_on_path("digital_out[0]", value=False),
            WriteAction(device_id="ur", key="digital_out[0]", value=False),
            ptp(target_pose),
            jnt(home_joints),
        ]

        # io_value = await controller.read_io("digital_out[0]")

        # plan_response = await motion_group.plan(trajectory, tcp="Flange")
        # print(plan_response)

        def print_motion(motion):
            print(motion)

        await motion_group.run(actions, tcp=tcp, initial_movement_consumer=print_motion)
        await motion_group.run(actions, tcp=tcp)
        await motion_group.run(ptp(target_pose), tcp=tcp)


if __name__ == "__main__":
    asyncio.run(main())
