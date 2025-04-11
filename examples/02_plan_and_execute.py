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

import pydantic

from nova import MotionSettings, Nova, program
from nova.actions import cartesian_ptp, joint_ptp, linear
from nova.api import models
from nova.types import Pose


class ProgramParameter(program.ProgramParameter):
    # Required integer with validation
    number_of_picks: int = pydantic.Field(gt=0, description="Number of picks to perform")


@program.define_program(parameter=ProgramParameter, name="example_program")
async def pick_and_place_program(nova_context: Nova, number_of_picks: int):
    print(number_of_picks)
    cell = nova_context.cell()
    controller = await cell.controller("controller")

    # Connect to the controller and activate motion groups
    async with controller[0] as motion_group:
        home_joints = await motion_group.joints()
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]
        current_pose = await motion_group.tcp_pose(tcp)

        pick_pose = Pose((100, 0, 0, 0, 0, 0))
        drop_pose = Pose((0, 100, 0, 0, 0, 0))

        actions = [
            joint_ptp(home_joints),
            # go to pick pose
            cartesian_ptp(current_pose @ pick_pose @ (0, 0, -100, 0, 0, 0)),
            linear(current_pose @ pick_pose),
            linear(current_pose @ pick_pose @ (0, 0, -100, 0, 0, 0)),
            # go to drop pose
            cartesian_ptp(current_pose @ drop_pose @ (0, 0, -100, 0, 0, 0)),
            linear(current_pose @ drop_pose),
            linear(current_pose @ drop_pose @ (0, 0, -100, 0, 0, 0)),
            linear(current_pose @ drop_pose @ (0, 0, -100, 0, 0, 0)),
            # go to home pose
            joint_ptp(home_joints),
        ]

    # you can update the settings of the action
    for action in actions:
        action.settings = MotionSettings(tcp_velocity_limit=200)

    joint_trajectory = await motion_group.plan(actions, tcp)
    for i in range(number_of_picks):
        print(f"Executing pick {i+1} of {number_of_picks}")
        await motion_group.execute(joint_trajectory, tcp, actions=actions)


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "controller",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        await program.run(pick_and_place_program, ProgramParameter(number_of_picks=2))
        # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    #schema = ProgramParameter.model_json_schema()
    #with open("schema.json", "w") as f:
    #    json.dump(schema, f, indent=2)
    asyncio.run(main())
