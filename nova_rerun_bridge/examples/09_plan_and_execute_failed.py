import asyncio

from nova import MotionSettings
from nova.actions import cartesian_ptp, joint_ptp
from nova.api import models
from nova.core.exceptions import PlanTrajectoryFailed
from nova.core.nova import Nova
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge


async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            actions = [
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ [-100, 0, 0, 0, 0, 0]),
                cartesian_ptp(target_pose @ [-200, 0, 0, 0, 0, 0]),
                cartesian_ptp(target_pose @ [-500, 0, 0, 0, 0, 0]),
                cartesian_ptp(target_pose @ [-2000, 0, 0, 0, 0, 0]),
                joint_ptp(home_joints),
            ]

            # you can update the settings of the action
            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            try:
                joint_trajectory = await motion_group.plan(actions, tcp)
                await bridge.log_actions(actions)
                await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
            except PlanTrajectoryFailed as e:
                await bridge.log_actions(actions, show_connection=True)
                await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
                await bridge.log_error_feedback(e.error.error_feedback)
                return


if __name__ == "__main__":
    asyncio.run(test())
