# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "wandelbots-nova",
#     "pydantic",
#     "httpx",
# ]
# ///

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
from nova import MotionSettings, Nova
from nova.actions import cartesian_ptp, joint_ptp, linear
from nova.api import models
from nova.types import Pose
import pydantic


class ProgramParameter(nova.ProgramParameter):
    # box size greater than 3 and less than 6 and required
    box_size: int = pydantic.Field(gt=3, lt=6, description="Size of the box")
    # box length greater than 0 and not required
    box_length: int = pydantic.Field(default=10, gt=0, description="Length of the box")


@nova.program(parameter=ProgramParameter, name="example_program")
async def program(nova_context: Nova, arguments: ProgramParameter):
    cell = nova_context.cell()
    controller = await cell.controller("controller")

    # Connect to the controller and activate motion groups
    async with controller[0] as motion_group:
        home_joints = await motion_group.joints()
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        pick_pose = Pose((100, 0, 0, 0, 0, 0))
        drop_pose = Pose((0, 100, 0, 0, 0, 0))

        actions = [
            joint_ptp(home_joints),
            # go to pick pose
            cartesian_ptp(pick_pose @ (0, 0, -100, 0, 0, 0)),
            linear(pick_pose),
            linear(pick_pose @ (0, 0, -100, 0, 0, 0)),
            # go to drop pose
            cartesian_ptp(drop_pose @ (0, 0, -100, 0, 0, 0)),
            linear(drop_pose),
            linear(drop_pose @ (0, 0, -100, 0, 0, 0)),
            # go to home pose
            joint_ptp(home_joints),
        ]

    # you can update the settings of the action
    for action in actions:
        action.settings = MotionSettings(tcp_velocity_limit=200)

    joint_trajectory = await motion_group.plan(actions, tcp)
    motion_iter = motion_group.stream_execute(joint_trajectory, tcp, actions=actions)
    async for motion_state in motion_iter:
        print(motion_state)


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        await program(nova_context=nova.context(), arguments=ProgramParameter(box_size=4, box_length=5))
        await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
