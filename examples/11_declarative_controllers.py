#!/usr/bin/env -S uv run --script

"""
Example: Declarative controller creation with multiple robots.

This example demonstrates the new declarative controller pattern where controllers
are defined in the @nova.program decorator and automatically created/cleaned up.

Prerequisites:
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

from pydantic import Field

import nova
from nova import Nova, api
from nova.actions import cartesian_ptp, joint_ptp
from nova.actions.motions import Motion
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose


@nova.program(
    controllers=[
        virtual_controller(
            name="ur-robot",
            manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
            type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
        ),
        virtual_controller(
            name="abb-robot", 
            manufacturer=api.models.Manufacturer.ABB,
            type=api.models.VirtualControllerTypes.ABB_MINUS_IRB1200_7,
        ),
    ],
    cleanup_controllers=True,
)
async def main(
    number_of_moves: int = Field(
        default=3, gt=0, description="Number of moves to perform per robot"
    ),
):
    """
    Example showing declarative controller creation with multiple robots.

    This program demonstrates:
    - Declarative controller setup using @nova.program decorator
    - Multiple controllers (UR and ABB robots) created automatically
    - Controllers are available without manual creation
    - Automatic cleanup when program completes
    """
    async with Nova() as nova:
        cell = nova.cell()

        ur_controller = await cell.controller("ur-robot")
        abb_controller = await cell.controller("abb-robot")

        print(f"Successfully accessed UR robot controller: {ur_controller.id}")
        print(f"Successfully accessed ABB robot controller: {abb_controller.id}")

        print(f"\n--- UR Robot Movement ({number_of_moves} moves) ---")
        async with ur_controller[0] as ur_motion_group:
            ur_home_joints = await ur_motion_group.joints()
            ur_tcp_names = await ur_motion_group.tcp_names()
            ur_tcp = ur_tcp_names[0]
            ur_current_pose = await ur_motion_group.tcp_pose(ur_tcp)

            ur_actions: list[Motion] = []
            ur_actions.append(joint_ptp(ur_home_joints))

            for i in range(number_of_moves):
                offset_pose = Pose((50 * (i + 1), 0, 0, 0, 0, 0))
                ur_actions.append(cartesian_ptp(ur_current_pose @ offset_pose))
                ur_actions.append(joint_ptp(ur_home_joints))

            for action in ur_actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            ur_trajectory = await ur_motion_group.plan(ur_actions, ur_tcp)
            print("Executing UR robot movements...")
            await ur_motion_group.execute(ur_trajectory, ur_tcp, actions=ur_actions)

        print(f"\n--- ABB Robot Movement ({number_of_moves} moves) ---")
        async with abb_controller[0] as abb_motion_group:
            abb_home_joints = await abb_motion_group.joints()
            abb_tcp_names = await abb_motion_group.tcp_names()
            abb_tcp = abb_tcp_names[0]
            abb_current_pose = await abb_motion_group.tcp_pose(abb_tcp)

            abb_actions: list[Motion] = []
            abb_actions.append(joint_ptp(abb_home_joints))

            for i in range(number_of_moves):
                offset_pose = Pose((0, 50 * (i + 1), 0, 0, 0, 0))
                abb_actions.append(cartesian_ptp(abb_current_pose @ offset_pose))
                abb_actions.append(joint_ptp(abb_home_joints))

            for action in abb_actions:
                action.settings = MotionSettings(tcp_velocity_limit=150)

            abb_trajectory = await abb_motion_group.plan(abb_actions, abb_tcp)
            print("Executing ABB robot movements...")
            await abb_motion_group.execute(abb_trajectory, abb_tcp, actions=abb_actions)

        print("\n--- Program Complete ---")
        print("Controllers will be automatically cleaned up due to cleanup_controllers=True")


if __name__ == "__main__":
    args = main.create_parser().parse_args()

    asyncio.run(main(**vars(args)))
