"""
Example: Serialize and Deserialize a Program for later Execution

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio
import json

from wandelbots_api_client.models.joint_trajectory import JointTrajectory

from nova import MotionSettings, Nova
from nova.actions import cartesian_ptp, joint_ptp
from nova.actions.base import Action
from nova.api import models
from nova.types import Pose


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
            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            actions = [
                joint_ptp(home_joints),
                cartesian_ptp(target_pose),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ [50, 0, 0, 0, 0, 0]),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ (50, 100, 0, 0, 0, 0)),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ Pose((0, 50, 0, 0, 0, 0))),
                joint_ptp(home_joints),
            ]

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=200)

        joint_trajectory = await motion_group.plan(actions, tcp)

        # Serialize the actions and joint trajectory
        serialized_actions = []
        for action in actions:
            # Get the serialized representation of each action
            action_data = action.serialize_model()
            # Ensure the type is explicitly included
            if "type" not in action_data:
                action_data["type"] = action.type
            serialized_actions.append(action_data)

        # Create a complete serializable representation
        serialized_program = {
            "joint_trajectory": joint_trajectory.to_json(),
            "tcp": tcp,
            "actions": serialized_actions,
        }

        with open("serialized_program.json", "w") as f:
            json.dump(serialized_program, f)

        # Later, to load and execute:
        with open("serialized_program.json", "r") as f:
            loaded_program = json.load(f)

        loaded_joint_trajectory = JointTrajectory.from_json(loaded_program["joint_trajectory"])
        loaded_tcp = loaded_program["tcp"]
        loaded_actions = []
        for action_data in loaded_program["actions"]:
            loaded_actions.append(Action.create_from_dict(action_data))

        # Execute with the loaded objects
        await motion_group.execute(loaded_joint_trajectory, loaded_tcp, actions=loaded_actions)

        await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
