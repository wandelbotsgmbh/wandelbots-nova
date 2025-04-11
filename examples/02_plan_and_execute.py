#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "wandelbots-nova",
#     "pydantic==2.11.3",
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
import os

import pydantic

from nova import MotionSettings, Nova
from nova.actions import cartesian_ptp, joint_ptp, linear
from nova.api import models
from nova.types import Pose


class ProgramParameter(pydantic.BaseModel):
    # Required integer with validation
    number_of_picks: int = pydantic.Field(gt=0, description="Number of picks to perform")


async def main(args: ProgramParameter):
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "controller",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
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
        for i in range(args.number_of_picks):
            print(f"Executing pick {i + 1} of {args.number_of_picks}")
            await motion_group.execute(joint_trajectory, tcp, actions=actions)


if __name__ == "__main__":
    # TODO: add nova util to create a parser based on the ProgramParameter model
    # ./examples/02_plan_and_execute.py --args={"number_of_picks": 3}
    args = ProgramParameter.model_validate_json(os.environ.get("NOVA_PROGRAM_ARGS", "{}"))
    asyncio.run(main(args))
