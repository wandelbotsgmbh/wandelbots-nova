from wandelbots import Nova, ptp, jnt, Pose
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
        target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))

        actions = [jnt(home_joints), ptp(target_pose), jnt(home_joints)]

        # plan_response = await motion_group.plan(trajectory, tcp="Flange")
        # print(plan_response)

        await motion_group.run(actions, tcp="Flange")


if __name__ == "__main__":
    asyncio.run(main())
