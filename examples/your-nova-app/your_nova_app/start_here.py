"""
This example shows how to use the Python SDK to control a virtual KUKA KR16 R2010 robot.

This demonstrates:
- Setting up a virtual robot controller
- Connecting to the robot
- Planning and executing basic movements
- Using joint and point-to-point motion types

Key robotics concepts:
- Motion groups: Controllable robot parts (usually the arm)
- TCP (Tool Center Point): The point you control on the robot
- Joint movement (jnt): Move by specifying joint angles in radians
- Point-to-point movement (ptp): Move to a specific position/orientation (x,y,z, rotation angle in radians)
- Pose: Position (x,y,z) and orientation (rx,ry,rz) in 3D space
"""

import nova
from nova import api, run_program
from nova.actions import cartesian_ptp, circular, joint_ptp, linear
from nova.cell import virtual_controller
from nova.core.nova import Nova
from nova.events import Cycle
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose


# Configure the robot program
@nova.program(
    id="start_here",  # Unique identifier of the program. If not provided, the function name will be used.
    name="Start Here",  # Readable name of the program
    viewer=nova.viewers.Rerun(),  # add this line for a 3D visualization
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-kr16-r2010",
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_KR16_R2010_2,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def start():
    """Main robot control function."""
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("kuka-kr16-r2010")
        cycle = Cycle(cell=cell, extra={"program": "start_here"})

        slow = MotionSettings(tcp_velocity_limit=50)

        async with controller[0] as motion_group:
            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and create target poses
            current_pose = await motion_group.tcp_pose(tcp)
            # Define the target pose based on the current pose with an offset of 100 mm on the x-axis
            target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))
            offset = 200

            # Actions define the sequence of movements and other actions to be executed by the robot
            actions = [
                joint_ptp(home_joints, settings=slow),  # Move to home position slowly
                cartesian_ptp(target_pose),  # Move to target pose
                joint_ptp(home_joints),  # Return to home
                cartesian_ptp(
                    target_pose @ [offset, 0, 0, 0, 0, 0]
                ),  # Move 200mm in target pose's local x-axis
                joint_ptp(home_joints),
                linear(target_pose @ (offset, offset, 0, 0, 0, 0)),  # Move 200mm in local x and y axes
                joint_ptp(home_joints, settings=slow),
                cartesian_ptp(target_pose @ Pose((0, offset, 0, 0, 0, 0))),
                joint_ptp(home_joints),
                circular(
                    target_pose @ Pose((0, 200, 0, 0, 0, 0)),
                    intermediate=target_pose @ Pose((200, 0, 0, 0, 0, 0)),
                ),
                joint_ptp(home_joints),
            ]

            # Start the cycle
            await cycle.start()

            # Plan the movements (shows in 3D viewer or creates an rrd file)
            joint_trajectory = await motion_group.plan(actions, tcp)

            # OPTIONAL: Execute the planned movements
            # You can comment out the lines below to only see the plan in Rerun
            print("Executing planned movements...")
            await motion_group.execute(joint_trajectory, tcp, actions=actions)

            # Finish the cycle
            await cycle.finish()
            print("Movement execution completed!")


if __name__ == "__main__":
    run_program(start)
