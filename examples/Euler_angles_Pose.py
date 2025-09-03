"""
Example: Perform relative movements with a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

import numpy as np

import nova
from nova import Nova, api, viewers
from nova.actions import jnt, ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose


@nova.program(
    name="Plan and Execute",
    viewer=viewers.Rerun(),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur10")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # move the robot to a home pose:
            home_pose = [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
            await motion_group.plan_and_execute(jnt(home_pose), tcp=tcp)
            current_pose = await motion_group.tcp_pose(tcp)

            # move the tcp to rotation (90, 0, 90) "xyz"

            # rotate with rotation vector:
            target_pose = Pose(
                current_pose[0], current_pose[1], current_pose[2], 1.2091996, -1.2091996, 1.2091996
            )
            await motion_group.plan_and_execute(ptp(target_pose), tcp=tcp)

            # move back to home_pose
            await motion_group.plan_and_execute(jnt(home_pose), tcp=tcp)
            current_pose = await motion_group.tcp_pose(tcp)

            # rotate with euler angles:
            position = (current_pose[0], current_pose[1], current_pose[2])
            euler_angles_rad = (np.pi / 2, 0, np.pi / 2)  # (90, 0, 90) in radians
            target_pose = Pose.from_euler(
                position, euler_angles_rad, convention="xyz", degrees=False
            )
            await motion_group.plan_and_execute(
                ptp(target_pose), settings=MotionSettings(tcp_velocity_limit=250), tcp=tcp
            )


if __name__ == "__main__":
    asyncio.run(main())
