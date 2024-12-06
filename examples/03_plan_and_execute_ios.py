from nova import Nova, Pose, ptp, jnt

# TODO: public interface
from nova.types.action import WriteAction
import asyncio
import numpy as np


async def main():
    nova = Nova()
    cell = nova.cell()
    controller = await cell.controller("ur")

    # Define a home position
    home_joints = (0, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0)

    # Connect to the controller and activate motion groups
    async with controller:
        motion_group = controller.get_motion_group()

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose("Flange")
        target_pose = current_pose @ Pose.from_tuple((100, 0, 0, 0, 0, 0))
        actions = [
            jnt(home_joints),
            # controller.write_on_path("digital_out[0]", value=False),
            WriteAction(device_id="ur", key="digital_out[0]", value=False),
            ptp(target_pose),
            jnt(home_joints),
        ]

        # io_value = await controller.read_io("digital_out[0]")

        # plan_response = await motion_group.plan(trajectory, tcp="Flange")
        # print(plan_response)

        motion_iter = motion_group.stream_run(actions, tcp="Flange")
        async for motion_state in motion_iter:
            print(motion_state)

        await motion_group.run(actions, tcp="Flange")
        await motion_group.run(ptp(target_pose), tcp="Flange")


if __name__ == "__main__":
    asyncio.run(main())
