import asyncio
from pathlib import Path

from nova import Nova, api
from nova.cell import virtual_controller
from nova.types import Pose
from wandelscript import run_wandelscript_program


async def main():
    path = Path(__file__).parent / "run_wandelscript_file.ws"

    # Read the file content
    with open(path) as f:
        program_code = f.read()

    nova = Nova()
    cell = nova.cell()
    await cell.ensure_controller(
        virtual_controller(
            name="ur10e",
            manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
            type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
        )
    )

    await run_wandelscript_program(
        program_id=path.stem,
        code=program_code,
        parameters={
            "pose_a": Pose((0, 0, 400, 0, 3.14, 0)),
            "a_dict": {"nested": 3},
            "a_list": [1, 2, {"nested": 4}],
        },
        nova=nova,
        default_robot="0@ur10e",
        default_tcp=None,
    )


if __name__ == "__main__":
    asyncio.run(main())
