import asyncio
from pathlib import Path

import nova
import wandelscript
from nova import api
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose


@nova.program(
    name="Run Wandelscript File",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=True,
    ),
)
async def main():
    path = Path(__file__).parent / "run_wandelscript_file.ws"
    with open(path) as f:
        program = f.read()

    run = wandelscript.run(
        program_id="ws_program",
        program=program,
        args={
            "pose_a": Pose((0, 0, 400, 0, 3.14, 0)),
            "a_dict": {"nested": 3},
            "a_list": [1, 2, {"nested": 4}],
        },
        default_tcp=None,
        default_robot="0@ur10e",
    )
    print(run.program_run.output_data)


if __name__ == "__main__":
    asyncio.run(main())
