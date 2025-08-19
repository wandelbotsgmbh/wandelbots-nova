import asyncio
from math import pi

from loguru import logger

import nova
from nova import Nova, api
from nova.actions import jnt, lin
from nova.cell.controllers import virtual_controller
from nova.core.movement_controller import TrajectoryCursor
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
                jnt(home_joints),
            ]

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=100)

        joint_trajectory = await motion_group.plan(actions, tcp)
        # TODO this probaly not consumes the state stream immediately and thus might cause issues
        # only start consuming the state stream when the trajectory is actually executed
        trajectory_cursor = TrajectoryCursor(joint_trajectory)
        motion_iter = motion_group.stream_execute(
            joint_trajectory, tcp, actions=actions, movement_controller=trajectory_cursor
        )

        async def driver():
            trajectory_cursor.pause_at(0.7)
            ic("FORWARD")
            trajectory_cursor.forward()
            await asyncio.sleep(5.5)
            ic("FORWARD")
            trajectory_cursor.forward()
            await asyncio.sleep(2)
            ic("PAUSE")
            trajectory_cursor.pause()
            # await asyncio.sleep(1)
            # ic("BACKWARD")
            # trajectory_cursor.backward()
            # await asyncio.sleep(4)
            # ic("PAUSE")
            # trajectory_cursor.pause()
            ic("FORWARD")
            trajectory_cursor.forward()
            await asyncio.sleep(4)
            trajectory_cursor.detach()
            ic("DRIVER DONE")

        async def runtime_monitor(interval=0.5):
            start_time = asyncio.get_event_loop().time()
            while True:
                elapsed = asyncio.get_event_loop().time() - start_time
                logger.warning(f"{elapsed:.2f}s")
                await asyncio.sleep(interval)

        driver_task = asyncio.create_task(driver())
        runtime_task = asyncio.create_task(runtime_monitor(0.5))  # Output every 0.5 seconds

        try:
            async for motion_state in motion_iter:
                pass
                # ic(motion_state.path_parameter)
            await driver_task
        finally:
            runtime_task.cancel()
            try:
                await runtime_task
            except asyncio.CancelledError:
                pass

        ic("Program finished")
        # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
