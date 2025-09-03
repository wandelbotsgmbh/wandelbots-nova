"""
Example: Trajectory with repeated executions using TrajectoryCursor.

This example demonstrates:
- Creating a trajectory with two actions (start and target) connected by linear motion
- Using TrajectoryCursor to control trajectory execution with different initial locations
- Executing the same trajectory multiple times with alternating directions:
  * Even executions (0, 2, 4...): start from beginning (forward: start → target)
  * Odd executions (1, 3, 5...): start from end (backward: target → start)

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio
from math import pi

from icecream import ic

import nova
from nova import Nova, api
from nova.actions import jnt, lin
from nova.cell import virtual_controller
from nova.core.movement_controller import TrajectoryCursor
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose


@nova.program(
    name="Trajectory Repeat Execution",
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
            # Move to home position first
            home_joints = [0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2]
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Move to home position
            print("Moving to home position...")
            await motion_group.plan_and_execute([jnt(home_joints)], tcp)

            # Get the starting pose for our trajectory
            start_pose = await motion_group.tcp_pose(tcp)
            print(f"Starting pose: {start_pose}")

            # Define the trajectory movement (100mm along X and Y axes)
            movement_distance = 300  # mm
            target_offset = Pose((movement_distance, movement_distance, 0, 0, 0, 0))

            # Configure motion settings
            motion_settings = MotionSettings(tcp_velocity_limit=500)

            # Number of trajectory executions
            num_executions = 60

            # Plan the trajectory once - we'll reuse it with different initial locations
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ target_offset

            print(f"Initial pose: {current_pose}")
            print(f"Target pose: {target_pose}")

            # Create trajectory with two actions connected by linear motion
            actions = [
                lin(current_pose, settings=motion_settings),  # Start action
                lin(target_pose, settings=motion_settings),  # Target action
            ]

            # Plan the trajectory once
            print("Planning trajectory...")
            joint_trajectory = await motion_group.plan(actions, tcp)
            ic(joint_trajectory.locations[0], joint_trajectory.joint_positions[0])
            ic(joint_trajectory.locations[-1], joint_trajectory.joint_positions[-1])
            initial_location = joint_trajectory.locations[0]

            for execution in range(num_executions):
                print(f"\n--- Execution {execution + 1}/{num_executions} ---")

                # Create TrajectoryCursor with the specified initial location
                trajectory_cursor = TrajectoryCursor(
                    motion_id="",  # Will be set during execution
                    joint_trajectory=joint_trajectory,
                    actions=actions,
                    initial_location=initial_location,
                    detach_on_standstill=True,
                )

                # Execute the trajectory using the motion group's execute method with TrajectoryCursor
                print("Executing trajectory...")

                async def execute_trajectory():
                    motion_iter = motion_group.stream_execute(
                        joint_trajectory,
                        tcp,
                        actions=actions,
                        movement_controller=trajectory_cursor,
                    )

                    # Monitor execution
                    step_count = 0
                    async for motion_state in motion_iter:
                        if step_count % 50 == 0:  # Print every 50th state to avoid spam
                            print(
                                f"  Motion state - Path parameter: {motion_state.path_parameter:.3f}"
                            )
                        step_count += 1

                execute_task = asyncio.create_task(execute_trajectory())
                if initial_location <= 1.2:
                    direction_desc = "forward (start → target)"
                    print(f"Starting from location: {initial_location:.3f}")
                    print(f"Direction: {direction_desc}")
                    await trajectory_cursor.forward_to(1.7)
                else:
                    direction_desc = "backward (target → start)"
                    print(f"Starting from location: {initial_location:.3f}")
                    print(f"Direction: {direction_desc}")
                    await trajectory_cursor.backward_to(1.2)
                await execute_task

                # Verify final position
                final_pose = await motion_group.tcp_pose(tcp)
                final_location = trajectory_cursor._current_location
                print(f"Final pose: {final_pose}")
                print(f"Final trajectory location: {final_location:.3f}")
                initial_location = final_location

                # Small pause between executions
                # await asyncio.sleep(0.5)

            print(f"\nCompleted {num_executions} trajectory executions using TrajectoryCursor!")
            print("Executions alternated between forward and backward directions.")
            print("Even executions: forward (start → target)")
            print("Odd executions: backward (target → start)")

            # Optional: Return to home position
            print("\nReturning to home position...")
            await motion_group.plan_and_execute([jnt(home_joints)], tcp)


if __name__ == "__main__":
    asyncio.run(main())
