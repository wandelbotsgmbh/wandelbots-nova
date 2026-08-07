from datetime import datetime

from icecream import ic

import nova
from nova import Nova, api, run_program
from nova.actions import lin
from nova.cell.controllers import virtual_controller
from nova.program import ProgramPreconditions
from nova.types.motion_settings import MotionSettings

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")


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
            home = await motion_group.tcp_pose()
            ic(home)
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]
            velocity = MotionSettings(tcp_velocity_limit=500)
            await motion_group.plan_and_execute(
                [lin(home @ (0, 0, 100), settings=velocity), lin(home)], tcp
            )


if __name__ == "__main__":
    run_program(main)
