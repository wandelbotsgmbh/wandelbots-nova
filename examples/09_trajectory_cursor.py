import asyncio
from math import pi

from nova import MotionSettings, Nova
from nova.actions import jnt, lin
from nova.api import models
from nova.core.movement_controller import TrajectoryCursor
from nova.types import Pose

"""
Example: Perform relative movements with a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

from datetime import datetime

from icecream import ic

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            # home_joints = await motion_group.joints()
            home_joints = [-pi, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0]
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]
            motion_iter = await motion_group.plan_and_execute([jnt(home_joints)], tcp)

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            dist = 300
            target_pose = current_pose @ Pose((dist, 0, 0, 0, 0, 0))
            ic(current_pose, target_pose)

            actions = [
                lin(current_pose @ Pose((0, dist, 0, 0, 0, 0))),
                lin(current_pose @ Pose((0, dist, dist, 0, 0, 0))),
                lin(current_pose @ Pose((0, 0, dist, 0, 0, 0))),
                lin(current_pose @ Pose((0, 0, 0, 0, 0, 0))),
                jnt(home_joints),
            ]

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=100)

        joint_trajectory = await motion_group.plan(actions, tcp)
        trajectory_cursor = TrajectoryCursor(joint_trajectory)
        motion_iter = motion_group.stream_execute(
            joint_trajectory, tcp, actions=actions, movement_controller=trajectory_cursor
        )

        async def driver():
            trajectory_cursor.pause_at(1.5)
            ic()
            trajectory_cursor.forward()
            await asyncio.sleep(5.5)
            ic()
            trajectory_cursor.forward()
            await asyncio.sleep(2)
            ic()
            trajectory_cursor.pause()
            ic()
            trajectory_cursor.backward()
            await asyncio.sleep(4)
            ic()
            trajectory_cursor.pause()
            trajectory_cursor.forward()
            await asyncio.sleep(4)

        driver_task = asyncio.create_task(driver())
        async for motion_state in motion_iter:
            ic(motion_state.path_parameter)
        await driver_task
        # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
