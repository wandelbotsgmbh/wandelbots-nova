import asyncio
import json

import nova
from nova import api
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.core.nova import Nova
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose

TOOL_ASSET = "nova_rerun_bridge/example_data/tool.stl"
tcp_config_dict = {
    "id": "vacuum",
    "readable_name": "vacuum",
    "position": {"x": 0, "y": -80, "z": 340},
    "rotation": {"angles": [0, 0, 0], "type": "ROTATION_VECTOR"},
}


@nova.program(
    name="visualize_tool",
    viewer=nova.viewers.Rerun(application_id="visualize-tool", tcp_tools={"vacuum": TOOL_ASSET}),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=True,
    ),
)
async def test():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur10")

        # Connect to the controller and activate motion groups
        motion_group_idx = 0
        async with controller[motion_group_idx] as motion_group:
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
            target_pose = current_pose @ Pose((0, 0, 1, 0, 0, 0))

            actions = [joint_ptp(home_joints), cartesian_ptp(target_pose), joint_ptp(home_joints)]

            # you can update the settings of the action
            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            await motion_group.plan(actions, tcp_id)


if __name__ == "__main__":
    asyncio.run(test())
