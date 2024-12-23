from nova import Nova
from nova.actions import ptp, jnt
from nova.types import Pose
from math import pi
import asyncio

from nova.core.movement_controller import move_forward


async def main():
    nova = Nova()
    cell = nova.cell()
    controllers = await cell.controllers()
    controller = controllers[0]

    # Define a home position
    home_joints = (0, -pi / 4, -pi / 4, -pi / 4, pi / 4, 0)

    # Connect to the controller and activate motion groups
    async with controller[0] as motion_group:
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose(tcp)
        target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

        actions = [
            jnt(home_joints),
            ptp(target_pose),
            jnt(home_joints),
            ptp(target_pose @ [200, 0, 0, 0, 0, 0]),
            jnt(home_joints),
            ptp(target_pose @ (300, 0, 0, 0, 0, 0)),
            jnt(home_joints),
            ptp(target_pose @ Pose((300, 0, 0, 0, 0, 0))),
            jnt(home_joints),
            ptp(target_pose @ Pose((400, 0, 0, 0, 0, 0))),
            jnt(home_joints),
        ]

        await motion_group.run(actions, tcp=tcp, movement_controller=move_forward)


if __name__ == "__main__":
    asyncio.run(main())
