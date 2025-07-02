import asyncio
import json

import pytest

import nova
from novax import Novax
from novax.program_manager import ProgramManager

webdav_mock = {
    "key1": json.dumps({"value": "value1"}),
    "key2": json.dumps({"value": "value2"}),
    "key3": json.dumps({"value": "value3"}),
}


class SHLProgramManager(ProgramManager):
    def __init__(self):
        super().__init__()

    # GET: /programs
    async def _get_programs(self):
        for program_name in webdav_mock.keys():
            if self.has_program(program_name):
                continue

            @nova.program(name=program_name)
            async def program():
                program = webdav_mock[program_name]
                program_content = json.loads(program)

                # use SHL runner
                print(program_content["value"])
                await asyncio.sleep(1)
                print(f"Finished running {program_content}")
                return program_content

            self.register_program(program)


@nova.program(name="simple_program")
async def simple_program(number_of_steps: int = 30):
    print("Hello World!")

    for i in range(number_of_steps):
        print(f"Step: {i}")
        await asyncio.sleep(1)

    print("Finished Hello World!")


@pytest.fixture
def novax_app():
    novax = Novax()
    app = novax.create_app()
    novax.include_programs_router(app)

    # 1) Register Python programs (existing functionality)
    novax.register_program(simple_program)

    # 2) register programs from path
    # novax.register_programs(path="./programs")

    # 3) autodiscover programs
    # novax.autodiscover_programs()
    return app


@pytest.fixture
def novax_with_custom_program_manager():
    program_manager = SHLProgramManager()

    novax = Novax(program_manager_override=program_manager)
    app = novax.create_app()
    novax.include_programs_router(app)

    # 1) Register Python programs (existing functionality)
    novax.register_program(simple_program)
    return app
