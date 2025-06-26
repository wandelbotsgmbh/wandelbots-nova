import asyncio

import nova
from nova import Nova, api
from nova.actions import cartesian_ptp, linear
from nova.cell import virtual_controller
from nova.core.exceptions import PlanTrajectoryFailed
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose
from nova_rerun_bridge import NovaRerunBridge


@nova.program(
    name="11_standard_skill",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        cell = nova.cell()
        controller = await cell.controller("ur10")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            tcp = "Flange"

            home = await motion_group.tcp_pose(tcp)

            actions = [
                action
                for _ in range(3)
                for action in [
                    cartesian_ptp(home),
                    linear(target=Pose((50, 20, 30, 0, 0, 0)) @ home),
                    linear(target=Pose((100, 20, 30, 0, 0, 0)) @ home),
                    linear(target=Pose((50, 20, 30, 0, 0, 0)) @ home),
                    cartesian_ptp(home),
                ]
            ]

            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            try:
                joint_trajectory = await motion_group.plan(actions, tcp)
                await bridge.log_actions(actions)
                await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
            except PlanTrajectoryFailed as e:
                await bridge.log_actions(actions)
                await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
                await bridge.log_error_feedback(e.error.error_feedback)
                return


if __name__ == "__main__":
    asyncio.run(test())
