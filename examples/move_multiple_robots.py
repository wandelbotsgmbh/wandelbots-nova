"""
Example: Move multiple robots simultaneously.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

import nova
from nova import Controller, api, run_program
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.types import Pose


async def move_robot(controller: Controller):
    motion_group = controller[0]
    home_joints = await motion_group.joints()
    tcp_names = await motion_group.tcp_names()
    tcp = tcp_names[0]

    current_pose = await motion_group.tcp_pose(tcp)
    target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))
    actions = [joint_ptp(home_joints), cartesian_ptp(target_pose), joint_ptp(home_joints)]

    await motion_group.plan_and_execute(actions, tcp)


@nova.program(
    id="move_multiple_robots",
    name="Move Multiple Robots",
    viewer=nova.viewers.Rerun(),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
            ),
            virtual_controller(
                name="ur5",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR5E,
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def move_multiple_robots(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    ur10 = await cell.controller("ur10")
    ur5 = await cell.controller("ur5")
    await asyncio.gather(move_robot(ur5), move_robot(ur10))


if __name__ == "__main__":
    run_program(move_multiple_robots)
