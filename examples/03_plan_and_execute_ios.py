from wandelbots import use_nova_access_token, Controller
from wandelbots.core.nova import Nova
from wandelbots.types.trajectory import MotionTrajectory, WriteAction
from wandelbots.types.pose import Pose
from wandelbots.types.motion import ptp, jnt
from decouple import config
import asyncio
import numpy as np


async def main():
    nova = Nova()
    cell = nova.cell()

    # Define a home position
    home_joints = (0, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0)

    # Connect to the controller and activate motion groups
    async with await cell.controller("ur") as controller:
        motion_group = controller.get_motion_group()

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose("Flange")
        target_pose = current_pose @ Pose.from_tuple((100, 0, 0, 0, 0, 0))
        trajectory = MotionTrajectory(
            items=[
                jnt(home_joints),
                WriteAction(device_id="ur", key="digital_out[0]", value=False),
                ptp(target_pose),
                jnt(home_joints),
            ]
        )

        # plan_response = await motion_group.plan(trajectory, tcp="Flange")
        # print(plan_response)

        motion_iter = motion_group.stream_move(trajectory, tcp="Flange")
        async for motion_state in motion_iter:
            print(motion_state)


if __name__ == "__main__":
    asyncio.run(main())
