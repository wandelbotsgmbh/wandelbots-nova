from math import pi

import nova
from nova import api, run_program
from nova.actions import jnt, ptp
from nova.cell.controllers import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from nova.types.pose import Pose

fast = MotionSettings(tcp_velocity_limit=1000, tcp_acceleration_limit=2000)


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
async def main(ctx: nova.ProgramContext):
    cell = ctx.cell
    controller = await cell.controller("ur10e")

    # Connect to the controller and activate motion groups
    motion_group = controller[0]
    home = (693.5, -174.1, 676.9, -3.1416, 0, 0)
    home_joints = (0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2)
    tcp_names = await motion_group.tcp_names()
    tcp = tcp_names[0]
    actions = [
        jnt(home_joints, fast),
        ptp(Pose(0, 0, 200, 0, 0, 0) @ home, fast),
        jnt(home_joints, fast),
    ]
    await motion_group.plan_and_execute(actions, tcp)
    await motion_group.plan_and_execute(actions, tcp)


if __name__ == "__main__":
    run_program(main)
