import asyncio

import numpy as np
import rerun as rr
import trimesh

from nova import MotionSettings
from nova.actions import cartesian_ptp, joint_ptp
from nova.api import models
from nova.core.nova import Nova
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.consts import TIME_INTERVAL_NAME


async def test():
    """
    This example demonstrates how to import a PLY file and extract point cloud data.
    We choose the first green point and move the robot to it.
    """
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        # Load PLY file
        mesh = trimesh.load("nova_rerun_bridge/example_data/bin_everything_05.ply")

        # Extract vertex positions and colors
        positions = np.array(mesh.vertices)

        # Point cloud is oriented in a way that it needs to be rotated and translated
        rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        positions = positions @ rotation
        translation = np.array([0, -500, 1200])
        positions = positions + translation

        colors = mesh.visual.vertex_colors[:, :3]  # RGB only, drop alpha

        # Log point cloud
        rr.set_time_seconds(TIME_INTERVAL_NAME, 0)
        rr.log("motion/pointcloud", rr.Points3D(positions, colors=colors))

        # Find green points (high G, low R/B values)
        green_mask = (colors[:, 1] > 100) & (colors[:, 0] < 100) & (colors[:, 2] < 100)
        green_points = positions[green_mask]

        if len(green_points) == 0:
            print("No green points found!")
            return

        # Select first green point
        green_target_point = green_points[0]

        rr.log("motion/target", rr.Points3D([green_target_point], radii=[10], colors=[[0, 255, 0]]))

        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            greenTargetPose = Pose(
                (green_target_point[0], green_target_point[1], green_target_point[2], np.pi, 0, 0)
            )
            actions = [
                joint_ptp(home_joints),
                cartesian_ptp(greenTargetPose),
                joint_ptp(home_joints),
            ]

            # you can update the settings of the action
            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            joint_trajectory = await motion_group.plan(actions, tcp)

            await bridge.log_actions(actions)
            await bridge.log_trajectory(joint_trajectory, tcp, motion_group)


if __name__ == "__main__":
    asyncio.run(test())
