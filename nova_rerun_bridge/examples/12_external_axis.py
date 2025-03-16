import asyncio
import json
import os

from numpy import pi

from nova import Controller, Nova
from nova.actions import cartesian_ptp, joint_ptp
from nova.api import models
from nova.types import MotionSettings, Pose
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.trajectory import TimingMode

"""
Example: Show an external axis with a robot moving in a cell.

Prerequisites:
- A cell with a robot with an positioner (external) axis: setup a yaskawa-ar1440 and import the yaskawa-ar1440-with-external-axis.yaml in the settings app
- Set mounting of 16@yaskawa to yaskawa-ar1440-16-mounting
"""


async def move_robot(controller: Controller, bridge: NovaRerunBridge):
    async with controller[0] as motion_group:
        await bridge.log_saftey_zones(motion_group)

        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]
        home_joints = await motion_group.joints()

        current_pose = await motion_group.tcp_pose(tcp)

        actions = [
            joint_ptp(home_joints),
            cartesian_ptp(current_pose @ Pose((0, 0, -100, 0, -pi / 2, 0))),
            cartesian_ptp(current_pose @ Pose((-500, 0, 0, 0, -pi / 2, 0))),
            joint_ptp(home_joints),
        ]

        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=200)

        trajectory = await motion_group.plan(actions, tcp)
        await bridge.log_trajectory(trajectory, tcp, motion_group, timing_mode=TimingMode.SYNC)


async def move_positioner(controller: Controller, bridge: NovaRerunBridge):
    async with controller[16] as motion_group:
        actions = [
            joint_ptp((0, 0)),
            joint_ptp((pi / 4, pi / 4)),
            joint_ptp((-pi / 4, -pi / 4)),
            joint_ptp((0, 0)),
        ]

        trajectory = await motion_group.plan(actions, "")
        await bridge.log_trajectory(trajectory, "", motion_group, timing_mode=TimingMode.SYNC)


async def main():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()
        cell = nova.cell()

        # Load JSON configuration
        json_path = os.path.join(
            os.path.dirname(__file__), "yaskawa-ar1440-with-external-axis.json"
        )
        with open(json_path) as f:
            var_json = json.load(f)

        controller = await cell._get_controller_instance("yaskawa")
        if controller is None:
            await nova._api_client.controller_api.add_robot_controller(
                cell=cell._cell_id,
                robot_controller=models.RobotController(
                    name="yaskawa",
                    configuration=models.RobotControllerConfiguration(
                        models.VirtualController(
                            type="yaskawa-ar1440",
                            manufacturer="yaskawa",
                            position="[0,0,0,0,0,0]",
                            json=json.dumps(var_json),
                        )
                    ),
                ),
                completion_timeout=60 * 2,
            )
        controller = await cell.controller("yaskawa")

        await asyncio.gather(
            move_robot(controller, bridge=bridge), move_positioner(controller, bridge=bridge)
        )


if __name__ == "__main__":
    asyncio.run(main())
