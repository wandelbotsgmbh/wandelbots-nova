import asyncio
import json

import nova
from nova import api
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.core.nova import Nova
from nova.program import ProgramPreconditions
from nova.types import Pose

TOOL_ASSET = "nova_rerun_bridge/example_data/tool.stl"
robot_tcp_data = api.models.RobotTcpData(
    name="vacuum",
    position=api.models.Vector3d([0, -80, 340]),
    orientation=api.models.Orientation([0, 0, 0]),
    orientation_type=api.models.OrientationType.ROTATION_VECTOR,
)


@nova.program(
    name="visualize_tool",
    viewer=nova.viewers.Rerun(application_id="visualize-tool", tcp_tools={"vacuum": TOOL_ASSET}),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
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
            tcp_id = robot_tcp_data.name
            await nova.api.virtual_robot_setup_api.add_virtual_controller_tcp(
                cell=cell.cell_id,
                controller=controller.controller_id,
                motion_group=motion_group.motion_group_id,
                tcp=tcp_id,
                robot_tcp_data=robot_tcp_data,
            )

            tcps = await motion_group.tcps()
            print(tcps)

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
            await motion_group.plan(actions, tcp_id)


if __name__ == "__main__":
    asyncio.run(test())
