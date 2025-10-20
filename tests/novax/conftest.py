import asyncio

import pytest

import nova
from novax import Novax
from tests.novax.api.test_program_cycle import (
    program_with_cycle_data,
    program_with_cycle_extra,
    program_with_cycle_failure,
)
from tests.novax.api.test_program_run import (
    failing_program,
    long_running_program,
    sucessful_program,
)


@nova.program(name="simple_program")
async def simple_program(number_of_steps: int = 30):
    print("Hello World!")

    for i in range(number_of_steps):
        print(f"Step: {i}")
        await asyncio.sleep(1)

    print("Finished Hello World!")


@pytest.fixture
def novax_app():
    novax = Novax(app_name="novax_test")
    app = novax.create_app()
    novax.include_programs_router(app)

    novax.register_program(simple_program)

    # programs for integration tests
    novax.register_program(sucessful_program)
    novax.register_program(failing_program)
    novax.register_program(program_with_cycle_data)
    novax.register_program(program_with_cycle_failure)
    novax.register_program(program_with_cycle_extra)
    novax.register_program(long_running_program)

    yield app
