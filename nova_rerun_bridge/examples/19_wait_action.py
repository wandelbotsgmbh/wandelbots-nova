import asyncio

import nova
from nova.actions import cartesian_ptp, joint_ptp
from nova.actions.mock import wait
from nova.actions.motions import Motion
from nova.api import models
from nova.cell import virtual_controller
from nova.core.nova import Nova
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose
from nova_rerun_bridge import NovaRerunBridge


@nova.program(
    ProgramPreconditions(
        virtual_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
    )
)
async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()
        cell = nova.cell()
        controller = await cell.controller("ur10")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            actions = [
                wait(2),
                joint_ptp(home_joints),
                cartesian_ptp(current_pose @ [100, 0, 0, 0, 0, 0]),
                joint_ptp(home_joints),
                wait(2),
                cartesian_ptp(current_pose @ Pose((100, 100, 0, 0, 0, 0))),
                joint_ptp(home_joints),
                cartesian_ptp(current_pose @ Pose((0, 100, 0, 0, 0, 0))),
                joint_ptp(home_joints),
            ]

            # you can update the settings of the action
            for action in actions:
                if isinstance(action, Motion):
                    action.settings = MotionSettings(tcp_velocity_limit=200)

            joint_trajectory = await motion_group.plan(actions, tcp)

            await bridge.log_actions(actions)
            await bridge.log_trajectory(joint_trajectory, tcp, motion_group)


if __name__ == "__main__":
    asyncio.run(test())
