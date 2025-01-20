import asyncio

from nova import Nova
from nova.api import models

# TODO: update I/O interaction interface
from nova.actions import WriteAction, jnt, ptp
from nova.types import Pose

"""
Example: Move the robot and set I/Os on the path.

Prerequisites:
- At least one robot added to the cell.
- The robot must have a digital output named "digital_out[0]".
"""


async def main():
    with Nova() as nova:
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
            jnt(home_joints),
            WriteAction(key="digital_out[0]", value=False),
            ptp(target_pose),
            jnt(home_joints),
        ]

            # io_value = await controller.read_io("digital_out[0]")
            await motion_group.plan_and_execute(actions, tcp)


if __name__ == "__main__":
    asyncio.run(main())
