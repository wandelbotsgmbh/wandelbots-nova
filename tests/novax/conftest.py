import asyncio

import pytest

import nova
from nova.cell.simulation import SimulatedRobotCell
from novax import Novax


@nova.program(name="simple_program")
async def simple_program(number_of_steps: int = 30):
    print("Hello World!")

    for i in range(number_of_steps):
        print(f"Step: {i}")
        await asyncio.sleep(1)

    print("Finished Hello World!")


@pytest.fixture
def novax_app():
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.include_programs_router(app)

    # 1) Register Python programs (existing functionality)
    novax.register_program(simple_program)

    # 2) register programs from path
    # novax.register_programs(path="./programs")

    # 3) autodiscover programs
    # novax.autodiscover_programs()
    return app
