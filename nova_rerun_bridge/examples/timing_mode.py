import asyncio

from nova import Controller, MotionSettings, Nova
from nova.actions import cartesian_ptp, joint_ptp
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.trajectory import TimingMode

"""
Example: Move multiple robots simultaneously.

Prerequisites:
- A cell with two robots: one named "ur" and another named "kuka".
"""


async def move_robot(controller: Controller, bridge: NovaRerunBridge):
    async with controller[0] as motion_group:
        home_joints = await motion_group.joints()
        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]

        current_pose = await motion_group.tcp_pose(tcp)
        target_pose = current_pose @ (100, 0, 0, 0, 0, 0)
        actions = [joint_ptp(home_joints), cartesian_ptp(target_pose), joint_ptp(home_joints)]

        trajectory = await motion_group.plan(actions, tcp)
        await bridge.log_trajectory(trajectory, tcp, motion_group, timing_mode=TimingMode.SYNC)

        await motion_group.plan_and_execute(actions, tcp)


async def main():
    nova = Nova()
    bridge = NovaRerunBridge(nova)
    await bridge.setup_blueprint()

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
            joint_ptp(home_joints),
            cartesian_ptp(target_pose),
            joint_ptp(home_joints),
            cartesian_ptp(target_pose @ [100, 0, 0, 0, 0, 0]),
            joint_ptp(home_joints),
            cartesian_ptp(target_pose @ (100, 100, 0, 0, 0, 0)),
            joint_ptp(home_joints),
            cartesian_ptp(target_pose @ Pose((0, 100, 0, 0, 0, 0))),
            joint_ptp(home_joints),
        ]

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=200)

        joint_trajectory = await motion_group.plan(actions, tcp)

        await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
        await bridge.log_trajectory(joint_trajectory, tcp, motion_group)

        await motion_group.execute(joint_trajectory, tcp, actions=actions)

    cell = nova.cell()
    ur = await cell.controller("ur")
    kuka = await cell.controller("kuka")
    await asyncio.gather(move_robot(ur, bridge=bridge), move_robot(kuka, bridge=bridge))

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
            cartesian_ptp(target_pose @ [100, 0, 0, 0, 0, 0]),
            joint_ptp(home_joints),
            cartesian_ptp(target_pose @ (100, 100, 0, 0, 0, 0)),
            joint_ptp(home_joints),
            cartesian_ptp(target_pose @ Pose((0, 100, 0, 0, 0, 0))),
            joint_ptp(home_joints),
        ]

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=200)

        joint_trajectory = await motion_group.plan(actions, tcp)

        await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
        await bridge.log_trajectory(joint_trajectory, tcp, motion_group)

        await motion_group.execute(joint_trajectory, tcp, actions=actions)

    await nova.close()
    await bridge.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
