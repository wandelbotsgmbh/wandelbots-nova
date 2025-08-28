import asyncio
from datetime import datetime
from math import pi

from icecream import ic

import nova
from nova import Nova, api
from nova.actions import jnt, lin
from nova.cell.controllers import virtual_controller
from nova.core.movement_controller import TrajectoryCursor
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose

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
            tcp = tcp_names[0]
            motion_iter = await motion_group.plan_and_execute([jnt(home_joints)], tcp)

            start_pose = await motion_group.tcp_pose(tcp)
            ic(start_pose)
            dist = 300

            actions = [
                lin(start_pose @ Pose((0, dist, 0, 0, 0, 0))),
                lin(start_pose @ Pose((0, dist, dist, 0, 0, 0))),
                lin(start_pose @ Pose((0, 0, dist, 0, 0, 0))),
                lin(start_pose @ Pose((0, 0, 0, 0, 0, 0))),
                # jnt(home_joints),
            ]
            ic(actions)

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=100)

        joint_trajectory = await motion_group.plan(actions, tcp)

        async def driver():
            await trajectory_cursor.forward_to_next_action()
            ic()
            trajectory_cursor.detach()

        trajectory_cursor = TrajectoryCursor(joint_trajectory, 0)
        async with asyncio.TaskGroup() as tg:
            tg.create_task(
                motion_group.execute(
                    joint_trajectory, tcp, actions=actions, movement_controller=trajectory_cursor
                )
            )
            await asyncio.sleep(1)
            tg.create_task(driver())
        ic()

        ic()
        trajectory_cursor = TrajectoryCursor(joint_trajectory, trajectory_cursor._current_location)
        async with asyncio.TaskGroup() as tg:
            tg.create_task(
                motion_group.execute(
                    joint_trajectory, tcp, actions=actions, movement_controller=trajectory_cursor
                )
            )
            ic()
            await asyncio.sleep(1)
            tg.create_task(driver())
        ic()

        ic("Program finished")
        # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
