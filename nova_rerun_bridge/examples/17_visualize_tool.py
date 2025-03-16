import asyncio
import json

from nova import MotionSettings, api
from nova.actions import cartesian_ptp, joint_ptp
from nova.api import models
from nova.core.nova import Nova
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge

TOOL_ASSET = "nova_rerun_bridge/example_data/tool.stl"
tcp_config_dict = {
    "id": "vacuum",
    "readable_name": "vacuum",
    "position": {"x": 0, "y": -80, "z": 340},
    "rotation": {"angles": [0, 0, 0, 0], "type": "ROTATION_VECTOR"},
}


async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # Connect to the controller and activate motion groups
        motion_group_idx = 0
        async with controller[motion_group_idx] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            # Define home
            home_joints = await motion_group.joints()

            # Define new TCP on virtual robot
            tcp_id = tcp_config_dict["id"]
            tcp_config = api.models.RobotTcp.from_json(json.dumps(tcp_config_dict))
            await nova._api_client.virtual_robot_setup_api.add_virtual_robot_tcp(
                cell.cell_id, controller.controller_id, motion_group_idx, tcp_config
            )

            # Wait for tcp configuration to be applied
            while True:
                try:
                    await motion_group.tcp_pose(tcp_id)
                    break
                except api.exceptions.NotFoundException:
                    await asyncio.sleep(0.5)

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp_id)
            target_pose = current_pose @ Pose((0, 0, 500, 0, -1.75, 0))

            actions = [joint_ptp(home_joints), cartesian_ptp(target_pose), joint_ptp(home_joints)]

            # you can update the settings of the action
            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            joint_trajectory = await motion_group.plan(actions, tcp_id)

            await bridge.log_actions(actions)
            await bridge.log_trajectory(
                joint_trajectory, tcp_id, motion_group, tool_asset=TOOL_ASSET
            )


if __name__ == "__main__":
    asyncio.run(test())
