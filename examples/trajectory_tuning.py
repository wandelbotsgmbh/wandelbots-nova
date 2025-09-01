import asyncio
from math import pi

import nova
from nova import Nova, api
from nova.actions import jnt, lin
from nova.cell.controllers import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose

"""
Example: Perform relative movements with a robot.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

from datetime import datetime

from icecream import ic

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")


@nova.program(
    name="Basic Program",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur10e")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            # home_joints = await motion_group.joints()
            home_joints = [0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2]
            tcp_names = await motion_group.tcp_names()
            ic(tcp_names)
            tcp = tcp_names[0]
            motion_iter = await motion_group.plan_and_execute([jnt(home_joints)], tcp)
            ic()

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            dist = 300
            target_pose = current_pose @ Pose((dist, 0, 0, 0, 0, 0))
            ic(current_pose, target_pose)

            actions = [
                lin(current_pose @ Pose((0, dist, 0, 0, 0, 0))),
                lin(current_pose @ Pose((0, dist, dist, 0, 0, 0))),
                lin(current_pose @ Pose((0, 0, dist, 0, 0, 0))),
                lin(current_pose @ Pose((0, 0, 0, 0, 0, 0))),
                # jnt(home_joints),
            ]
            ic(actions)

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=100)

        joint_trajectory = await motion_group.plan(actions, tcp)
        # TODO this probaly not consumes the state stream immediately and thus might cause issues
        # only start consuming the state stream when the trajectory is actually executed
        await motion_group.execute(joint_trajectory, tcp, actions=actions)
        # try it twice to show that it works with multiple trajectories
        await motion_group.execute(joint_trajectory, tcp, actions=actions)

        ic("Program finished")
        # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
