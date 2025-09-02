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

import asyncio
import json

from nova.actions import lin, ptp
from nova.core.movement_controller import TrajectoryCursor
from nova.core.nova import Nova
from nova.events import nats
from nova.types import MotionSettings, Pose


# Configure the robot program
# @nova.program(
#     name="start_here",
#     viewer=nova.viewers.Rerun(),  # add this line for a 3D visualization
#     preconditions=ProgramPreconditions(
#         controllers=[
#             virtual_controller(
#                 name="kuka11",
#                 manufacturer=wbmodels.Manufacturer.KUKA,
#                 type=wbmodels.VirtualControllerTypes.KUKA_MINUS_KR16_R1610_2,
#             )
#         ],
#         cleanup_controllers=False,
#     ),
# )
async def start():
    """Main robot control function."""
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("robot")

        async with controller[0] as motion_group:
            home_joints = await motion_group.joints()
            # Custom TCP konfigurieren
            tcp_config = {
                "id": "mess_spitze",
                "readable_name": "2",
                "position": {"x": 130, "y": 1.51, "z": 320.28},  # 100mm Offset in Z
                "rotation": {"angles": [0, 0, 0], "type": "EULER_ANGLES_EXTRINSIC_XYZ"},
            }
            # Custom TCP hinzufügen

            # await nova._api_client.virtual_robot_setup_api.add_virtual_robot_tcp(
            #   cell.cell_id, controller.controller_id, 0, tcp_config
            # )
            tcp = "2"

            # Get current TCP pose and create target poses
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            # Define movement sequence
            actions = [
                ptp((727.5, -238.4, 871.9, 0.0075, 3.1358, -0.0041)),  # Move to home position
                ptp((729.1, -409.3, 442.5, 0.0074, 3.1358, -0.004)),  # Move to 1.approach pose
                lin((729.5, -409.5, 379.3, 0.0075, 3.1357, -0.004)),  # Move to 1.workPose
                lin((1005.5, -405.4, 382.3, 0.0078, 3.1357, -0.004)),  # Move to edgeWorkPose
                lin(
                    (1006, -291, 385.3, 0.0078, 3.1357, -0.0041)
                ),  # Move to 3.workPose with deviation
                lin((1005.8, -290.9, 418.7, 0.0078, 3.1357, -0.0041)),  # Move to 1.departurePose
                lin((1030.9, -272.3, 407, -0.0078, -2.8219, 0.0025)),  # Move to 2.approachPose
                lin((1023, -272.4, 383.1, -0.0078, -2.822, 0.0026)),  # Move to 4.workPose
                lin((1023.5, -176.6, 382.8, -0.0081, -2.8221, 0.0025)),  # Move to 5.workPose
                lin((1032.3, -176.7, 409.5, -0.008, -2.822, 0.0026)),  # Move to 2.departurePose
                lin((1007.8, -158.7, 411.7, -0.0082, -3.1272, 0.0041)),  # Move to 3.approachPose
                lin((1007.4, -158.7, 389.4, -0.0081, -3.1271, 0.0041)),  # Move to 6.workPose
                lin((1008, -31.6, 385, -0.0082, -3.1271, 0.004)),  # Move to 7.workPose
                lin((1008.9, -31.4, 442.3, -0.0082, -3.1272, 0.004)),  # Move to 3.departurePose
                ptp((727.5, -238.4, 871.9, 0.0075, 3.1358, -0.0041)),
            ]

            # Set motion velocity for all actions
            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=50)

            # Plan the movements (shows in 3D viewer or creates an rrd file)
            joint_trajectory = await motion_group.plan(actions, tcp)

            # OPTIONAL: Execute the planned movements
            # You can comment out the lines below to only see the plan in Rerun
            print("Executing planned movements...")

            trajectory_cursor = TrajectoryCursor(joint_trajectory)
            # for i, _ in enumerate(actions):
            #     trajectory_cursor.pause_at(i)
            motion_iter = motion_group.stream_execute(
                joint_trajectory, tcp, actions=actions, movement_controller=trajectory_cursor
            )
            # trajectory_cursor.forward()
            nats_client = await nats.get_client()

            async def cmd_sub_handler(msg):
                match json.loads(msg.data.decode()):
                    case {"command": "forward", **rest}:
                        trajectory_cursor.forward(playback_speed_in_percent=rest.get("speed", None))
                    case {"command": "step-forward", **rest}:
                        trajectory_cursor.forward_to_next_action(
                            playback_speed_in_percent=rest.get("speed", None)
                        )
                        trajectory_cursor.forward_to_next_action(
                            playback_speed_in_percent=rest.get("speed", None)
                        )
                    case {"command": "backward", **rest}:
                        trajectory_cursor.backward(
                            playback_speed_in_percent=rest.get("speed", None)
                        )
                    case {"command": "step-backward", **rest}:
                        trajectory_cursor.backward_to_previous_action(
                            playback_speed_in_percent=rest.get("speed", None)
                        )
                    case {"command": "pause", **rest}:
                        trajectory_cursor.pause()
                    case {"command": "finish", **rest}:
                        trajectory_cursor.detach()
                    case _:
                        print("Unknown command")

            sub = await nats_client.subscribe("trajectory-cursor", cb=cmd_sub_handler)

            # await motion_group.execute(joint_trajectory, tcp, actions=actions)
            async for motion_state in motion_iter:
                pass
            print("Movement execution completed!")


if __name__ == "__main__":
    asyncio.run(start())
