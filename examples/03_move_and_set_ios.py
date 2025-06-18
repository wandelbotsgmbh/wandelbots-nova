"""
Example: Move the robot and set I/Os on the path.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

from nova import Nova
from nova.actions import cartesian_ptp, joint_ptp
from nova.actions.io import io_write
from nova.api import models
from nova.types import Pose


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
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))
            actions = [
                joint_ptp(home_joints),
                io_write(key="tool_out[0]", value=False),
                cartesian_ptp(target_pose),
                joint_ptp(home_joints),
            ]

            async for motion_state in motion_group.stream_plan_and_execute(actions, tcp):
                print(motion_state)

            io_value = await controller.read("tool_out[0]")
            print(io_value)
            await controller.write("tool_out[0]", True)
            written_io_value = await controller.read("tool_out[0]")
            print(written_io_value)

        await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
