from math import pi

import nova
from nova import Nova, api, run_program
from nova.actions import jnt, ptp
from nova.cell.controllers import virtual_controller
from nova.program import ProgramPreconditions
from nova.types.pose import Pose


@nova.program(
    name="Minimal Program",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur10e")

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            home = (693.5, -174.1, 676.9, -3.1416, 0, 0)
            home_joints = (0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2)
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]
            actions = [jnt(home_joints), ptp(Pose(0, 0, 200, 0, 0, 0) @ home), jnt(home_joints)]
            await motion_group.plan_and_execute(actions, tcp)


if __name__ == "__main__":
    run_program(main)
