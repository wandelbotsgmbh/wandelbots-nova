import asyncio

from nova import MotionSettings, Nova
from nova.actions import collision_free, io_write, jnt, ptp
from nova.api import models
from nova.types import Pose

"""
Example: Perform a multi-step trajectory with collision avoidance.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""


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
            home_joints = await motion_group.joints()
            home_pose = await motion_group.tcp_pose()

            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))

            actions = [
                ptp(target_pose),
                collision_free(home_joints),
                ptp(target_pose @ [50, 0, 0, 0, 0, 0]),
                io_write(key="digital_out[0]", value=True),
                jnt(home_joints),
                ptp(target_pose @ (50, 100, 0, 0, 0, 0)),
                collision_free(home_pose),
                ptp(target_pose @ Pose((0, 50, 0, 0, 0, 0))),
                jnt(home_joints),
            ]

        # you can update the settings of the action
        for action in actions:
            if action.is_motion():
                action.settings = MotionSettings(tcp_velocity_limit=200)

        joint_trajectory = await motion_group.plan(
            actions, tcp, start_joint_position=(-0.0429, -1.8781, 1.8464, -2.1366, -1.4861, 1.0996)
        )
        await motion_group.execute(joint_trajectory, tcp, actions=actions)

        value = await controller.read("digital_out[0]")
        print(f"digital out: {value}")

        await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
