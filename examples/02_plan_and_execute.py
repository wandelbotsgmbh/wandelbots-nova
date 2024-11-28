from wandelbots import use_nova_access_token, Controller
from wandelbots.types.trajectory import MotionTrajectory
from wandelbots.types.pose import Pose
from wandelbots.types.motion import ptp, jnt
from decouple import config
import asyncio
import numpy as np


async def run_wandelscript(file: str, variables: dict[str, str | int | float]) -> str:
    pass


async def main():
    nova = use_nova_access_token()
    controller = Controller(nova, cell=config("CELL_ID"), controller_host=config("CONTROLLER_HOST"))

    # Define a home position
    home_joints = (0, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0)

    # Connect to the controller and activate motion groups
    async with controller:
        motion_group = controller[0]

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose("Flange")
        # TODO: fix pose concatenation: target_pose = current_pose @ Vector3d(x=100, y=0, z=0)
        target_pose = Pose(**current_pose.model_dump()).to_tuple()

        trajectory = MotionTrajectory(items=[jnt(home_joints), ptp(target_pose), jnt(home_joints)])

        # plan_response = await motion_group.plan(trajectory, tcp="Flange")
        # print(plan_response)

        motion_iter = motion_group.stream_move(trajectory, tcp="Flange")
        async for motion_state in motion_iter:
            print(motion_state)

        result = await run_wandelscript("ws/move.ws", {"box_size": 20})
        print(result)


"""
move via p2p() to home
...
python_function()
...
"""

if __name__ == "__main__":
    asyncio.run(main())
