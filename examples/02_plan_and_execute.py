from nova import Nova, ptp, jnt, Pose, pi
import asyncio
import numpy as np

from nova.core.movement_controller import move_forward


async def main():
    nova = Nova()
    cell = nova.cell()
    controller = await cell.controller("ur")

    # Define a home position
    home_joints = (0, -pi / 4, -pi / 4, -pi / 4, pi / 4, 0)

    # Connect to the controller and activate motion groups
    async with controller:
        motion_group = controller.get_motion_group()

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose("Flange")
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
            ptp(target_pose),
            jnt(home_joints),
        ]

        # plan_response = await motion_group.plan(trajectory, tcp="Flange")
        # print(plan_response)

        await motion_group.run(actions, tcp="Flange", movement_controller=move_forward)


if __name__ == "__main__":
    asyncio.run(main())
