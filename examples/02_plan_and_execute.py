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
import json
from pathlib import Path

import pydantic
import yaml

import nova
from nova import MotionSettings, Nova
from nova.actions import cartesian_ptp, joint_ptp, linear
from nova.api import models
from nova.types import Pose


class ProgramParameter(nova.ProgramParameter):
    # Required integer with validation
    number_of_picks: int = pydantic.Field(gt=0, description="Number of picks to perform")

    # Optional float with range validation
    speed_factor: float = pydantic.Field(default=1.0, ge=0.1, le=2.0, description="Speed multiplier for movements")

    # Required string with regex pattern
    robot_name: str = pydantic.Field(..., pattern="^[A-Za-z0-9_-]+$", description="Name of the robot to control")

    # Optional boolean with default
    enable_logging: bool = pydantic.Field(default=False, description="Enable detailed motion logging")

    # List of integers with length validation
    waypoint_indices: list[int] = pydantic.Field(
        default_factory=list,
        min_items=1,
        max_items=10,
        description="Indices of waypoints to visit"
    )

    # Optional string with allowed values
    operation_mode: str = pydantic.Field(
        default="standard",
        pattern="^(standard|advanced|debug)$",
        description="Operation mode for the program"
    )

    # Required tuple of floats for coordinates
    home_pose: Pose = pydantic.Field(
        ...,
        description="Home position coordinates (x, y, z)"
    )


@nova.program(parameter=ProgramParameter, name="example_program")
async def program(nova_context: Nova, number_of_picks: int):
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
    for i in range(number_of_picks):
        await motion_group.execute(joint_trajectory, tcp, actions=actions)


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        await program(nova, **ProgramParameter(number_of_picks=2).model_dump())
        await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    schema = ProgramParameter.model_json_schema()
    with open("schema.json", "w") as f:
        json.dump(schema, f, indent=2)
    # asyncio.run(main())
