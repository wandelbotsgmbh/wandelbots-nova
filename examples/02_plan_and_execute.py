from nova import Nova
from nova.actions import ptp, jnt
from nova.types import Pose
import asyncio

"""
Example: Perform relative movements with a robot.

Prerequisites:
- At least one robot added to the cell.
"""

async def main():
    nova = Nova()
    cell = nova.cell()
    controllers = await cell.controllers()
    controller = controllers[0]

    # Define a home position

    # Connect to the controller and activate motion groups
    async with controller[0] as motion_group:
        home_joints = await motion_group.joints()
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose(tcp)
        target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

        actions = [
            jnt(home_joints),
            ptp(target_pose),
            jnt(home_joints),
            ptp(target_pose @ [100, 0, 0, 0, 0, 0]),
            jnt(home_joints),
            ptp(target_pose @ (100, 100, 0, 0, 0, 0)),
            jnt(home_joints),
            ptp(target_pose @ Pose((0, 100, 0, 0, 0, 0))),
            jnt(home_joints),
        ]

        joint_trajectory = await motion_group.plan(actions, tcp)
        await motion_group.execute(joint_trajectory, tcp, actions=actions)


if __name__ == "__main__":
    asyncio.run(main())
