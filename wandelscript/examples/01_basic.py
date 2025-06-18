import asyncio
from pathlib import Path

import wandelscript
from nova import Nova, api
from nova.types import Pose


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            api.models.Manufacturer.UNIVERSALROBOTS,
        )

        async with controller[0] as motion_group:
            tcp_names = await motion_group.tcp_names()
            print(tcp_names)
            tcp = tcp_names[0]

            # Current motion group state
            state = await motion_group.get_state(tcp)
            print(state)

        robot_cell = await cell.get_robot_cell()
        run = wandelscript.run_file(
            Path(__file__).parent / "01_basic.ws",
            cell=robot_cell,
            default_tcp=None,
            default_robot=None,
            run_args={
                "pose_a": Pose((0, 0, 400, 0, 3.14, 0)),
                "a_dict": {"nested": 3},
                "a_list": [1, 2, {"nested": 4}],
            },
        )
        print(run.program_run.execution_results)

        # await cell.delete_robot_controller(controller.controller_id)


if __name__ == "__main__":
    asyncio.run(main())
