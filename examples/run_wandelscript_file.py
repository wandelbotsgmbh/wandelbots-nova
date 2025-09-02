import asyncio
from pathlib import Path

import nova
from nova import api
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose
from wandelscript import create_wandelscript_program


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
        cleanup_controllers=False,
    ),
)
async def main():
    path = Path(__file__).parent / "run_wandelscript_file.ws"

    # Read the file content
    with open(path) as f:
        program_code = f.read()

    program = create_wandelscript_program(
        program_id=path.stem,
        code=program_code,
        args={
            "pose_a": Pose((0, 0, 400, 0, 3.14, 0)),
            "a_dict": {"nested": 3},
            "a_list": [1, 2, {"nested": 4}],
        },
        default_robot="0@ur10e",
        default_tcp=None,
    )
    await program()


if __name__ == "__main__":
    asyncio.run(main())
