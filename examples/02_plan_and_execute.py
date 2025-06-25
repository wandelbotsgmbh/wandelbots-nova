"""
Example: Perform relative movements with a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

import nova
from nova import Nova, api
from nova.actions import cartesian_ptp, io_write, joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose


@nova.program(
    name="02 Plan and Execute",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=True,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            actions = [
                joint_ptp(home_joints),
                cartesian_ptp(target_pose),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ Pose((50, 0, 0, 0, 0, 0))),
                joint_ptp(home_joints),
                io_write(key="tool_out[0]", value=False),
                cartesian_ptp(target_pose @ Pose((50, 100, 0, 0, 0, 0))),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ Pose((0, 50, 0, 0, 0, 0))),
                joint_ptp(home_joints),
            ]

        joint_trajectory = await motion_group.plan(actions, tcp)
        motion_iter = motion_group.stream_execute(joint_trajectory, tcp, actions=actions)
        async for motion_state in motion_iter:
            print(motion_state)

        # Read and write IO values
        io_value = await controller.read("tool_out[0]")
        print(io_value)
        await controller.write("tool_out[0]", True)
        written_io_value = await controller.read("tool_out[0]")
        print(written_io_value)


if __name__ == "__main__":
    asyncio.run(main())
