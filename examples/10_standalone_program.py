#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "wandelbots-nova",
#     "pydantic==2.11.3",
#     "docstring_parser",
#     "httpx",
# ]
# ///

import asyncio

from pydantic import Field

import nova
from nova import Nova, api
from nova.actions import cartesian_ptp, joint_ptp, linear
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose


@nova.program(
    controllers=[
        virtual_controller(
            name="controller",
            manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
            type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
        )
    ],
    cleanup_controllers=True,
)
async def main(
    number_of_picks: int = Field(gt=0, description="Number of picks to perform"),
):
    """
    Pick and place program for a UR10e robot.
    """
    async with Nova() as nova:
        cell = nova.cell()
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
            print(f"Executing pick {i + 1} of {number_of_picks}")
            await motion_group.execute(joint_trajectory, tcp, actions=actions)


if __name__ == "__main__":
    # TODO: also handle args from env
    # Create parser from the function's input model
    args = main.create_parser().parse_args()

    # Convert args to dict and run the function
    asyncio.run(main(**vars(args)))
