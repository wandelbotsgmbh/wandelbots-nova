import asyncio

from nova import MotionSettings
from nova.actions import jnt, ptp
from nova.core.nova import Nova
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge


async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()
        await bridge.log_collision_scenes()

        cell = nova.cell()
        controllers = await cell.controllers()
        controller = controllers[0]

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            actions = [
                jnt(home_joints),
                ptp(target_pose),
                jnt(home_joints),
                ptp(target_pose @ [100, 0, 0, 0, 0, 0]),
                jnt(home_joints),
                ptp(target_pose @ (100, 100, 0, 0, 0, 0)),
                jnt(home_joints),
                ptp(target_pose @ Pose((0, 100, 0, 0, 0, 0))),
                jnt(home_joints),
            ]

            # you can update the settings of the action
            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            joint_trajectory = await motion_group.plan(actions, tcp)

            await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
            await bridge.log_trajectory(joint_trajectory, tcp, motion_group)

            await motion_group.execute(joint_trajectory, tcp, actions=actions)

        await nova.close()


if __name__ == "__main__":
    asyncio.run(test())
