import asyncio
from pathlib import Path

import nova
import wandelscript
from nova import Nova, api
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose


@nova.program(
    name="Run Wandelscript File",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=True,
    ),
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        robot_cell = await cell.get_robot_cell()

        run = wandelscript.run_file(
            Path(__file__).parent / "run_wandelscript_file.ws",
            args={
                "pose_a": Pose((0, 0, 400, 0, 3.14, 0)),
                "a_dict": {"nested": 3},
                "a_list": [1, 2, {"nested": 4}],
            },
            default_tcp=None,
            default_robot=None,
            robot_cell_override=robot_cell,
        )
        print(run.program_run.execution_results)


if __name__ == "__main__":
    asyncio.run(main())
