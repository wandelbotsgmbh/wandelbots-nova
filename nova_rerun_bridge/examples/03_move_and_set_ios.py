import asyncio

from nova import Nova
from nova.actions import cartesian_ptp, io_write, joint_ptp
from nova.api import models
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge

"""
Example: Move the robot and set I/Os on the path.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""


async def main():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))
            actions = [
                joint_ptp(home_joints),
                io_write(key="digital_out[0]", value=False),
                cartesian_ptp(target_pose),
                joint_ptp(home_joints),
            ]

            # io_value = await controller.read_io("digital_out[0]")
            joint_trajectory = await motion_group.plan(actions, tcp)

            await bridge.log_actions(actions)
            await bridge.log_trajectory(joint_trajectory, tcp, motion_group)


if __name__ == "__main__":
    asyncio.run(main())
