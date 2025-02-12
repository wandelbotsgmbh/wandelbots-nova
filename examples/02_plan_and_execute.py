import asyncio

from nova import MotionSettings, Nova
from nova.actions import jnt, ptp
from nova.api import models
from nova.types import Pose

"""
Example: Perform relative movements with a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

from icecream import ic

async def main():
    ic("bar")
    async with Nova() as nova:
        ic("baz")
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

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
                ptp(target_pose @ [50, 0, 0, 0, 0, 0]),
                jnt(home_joints),
                ptp(target_pose @ (50, 100, 0, 0, 0, 0)),
                jnt(home_joints),
                ptp(target_pose @ Pose((0, 50, 0, 0, 0, 0))),
                jnt(home_joints),
            ]

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=300)

        joint_trajectory = await motion_group.plan(actions, tcp)
        async for motion_state in motion_group.stream_execute(joint_trajectory, tcp, actions=actions):
            ic(motion_state)

        #await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    ic("foo")
    asyncio.run(main())
