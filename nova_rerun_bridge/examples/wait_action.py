import asyncio

import nova
from nova import api
from nova.actions import cartesian_ptp, joint_ptp
from nova.actions.mock import wait
from nova.actions.motions import Motion
from nova.cell import virtual_controller
from nova.core.nova import Nova
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose


@nova.program(
    name="stream-robot",
    viewer=nova.viewers.Rerun(application_id="stream-robot"),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur100",
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
        controller = await cell.controller("ur100")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
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

            await motion_group.plan(actions, tcp)


if __name__ == "__main__":
    asyncio.run(test())
