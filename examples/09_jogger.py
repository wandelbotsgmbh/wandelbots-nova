import asyncio
from datetime import datetime
from math import pi

from icecream import ic

from nova import Nova
from nova.actions import jnt
from nova.api import models
from nova.core.movement_controller import Jogger

"""
Example: Perform jogging movements with a robot using the Jogger movement controller.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            # Move to home position first
            home_joints = [-pi, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0]
            # tcp_names = await motion_group.tcp_names()
            # tcp = tcp_names[0]
            tcp = "Flange"
            motion_iter = await motion_group.plan_and_execute([jnt(home_joints)], tcp)

            # Wait for home movement to complete
            async for motion_state in motion_iter:
                ic(f"Homing: {motion_state.path_parameter}")

            # Get current TCP pose for reference
            current_pose = await motion_group.tcp_pose(tcp)
            ic(f"Starting pose: {current_pose}")

            # Create effect stream for the jogger
            effect_stream = motion_group.stream_state()

            # Create jogger instance
            jogger = Jogger(effect_stream, motion_group.motion_group_id, tcp)

            # Start jogging session
            motion_iter = motion_group.stream_execute_jogging(movement_controller=jogger)

            async def driver():
                # Wait a moment before starting jogging commands
                await asyncio.sleep(1)

                # Jog in positive X direction
                ic("Jogging in +X direction")
                jogger.jog_tcp(translation=[50, 0, 0])
                await asyncio.sleep(2)

                # Pause jogging
                ic("Pausing jog")
                jogger.pause()
                await asyncio.sleep(1)

                # Jog in positive Y direction
                ic("Jogging in +Y direction")
                jogger.jog_tcp(translation=[0, 50, 0])
                await asyncio.sleep(2)

                # Jog in negative Z direction
                ic("Jogging in -Z direction")
                jogger.jog_tcp(translation=[0, 0, -30])
                await asyncio.sleep(2)

                # Rotate around Z axis
                ic("Rotating around Z axis")
                jogger.jog_tcp(rotation=[0, 0, 0.2])
                await asyncio.sleep(1.5)

                # Stop all movement
                ic("Stopping jog")
                jogger.pause()
                await asyncio.sleep(1)

                # Jog back to approximate starting position
                ic("Jogging back to start")
                jogger.jog_tcp(translation=[-50, -50, 30])
                await asyncio.sleep(3)

                # Final pause
                jogger.pause()

            driver_task = asyncio.create_task(driver())

            # Monitor the jogging motion states
            motion_count = 0
            async for motion_state in motion_iter:
                motion_count += 1
                if motion_count % 10 == 0:  # Print every 10th state to reduce output
                    ic(f"Motion state: {motion_state}")

                # Break after driver completes and some additional states
                if driver_task.done() and motion_count > 50:
                    break

            await driver_task
            # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
