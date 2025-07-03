import asyncio
import json
from pathlib import Path
from typing import AsyncIterator

import pytest

import nova
from nova.program.function import Program
from novax import Novax
from novax.program_manager import FileSystemProgramSource, ProgramManager, ProgramSource


@nova.program(name="simple_program")
async def simple_program(number_of_steps: int = 30):
    print("Hello World!")

    for i in range(number_of_steps):
        print(f"Step: {i}")
        await asyncio.sleep(1)

    print("Finished Hello World!")


webdav_mock = {
    "key1": json.dumps({"value": "value1"}),
    "key2": json.dumps({"value": "value2"}),
    "key3": json.dumps({"value": "value3"}),
}


class WebDAVProgramSource(ProgramSource):
    """Program source for SHL-style programs from WebDAV"""

    def __init__(self, webdav_data: dict[str, str]):
        self.webdav_data = webdav_data

    async def get_programs(self, program_manager: ProgramManager) -> AsyncIterator[Program]:
        """Discover and register SHL programs from WebDAV data"""
        for program_name in self.webdav_data.keys():
            if program_manager.has_program(program_name):
                continue

            @nova.program(name=program_name)
            async def program():
                program = self.webdav_data[program_name]
                program_content = json.loads(program)

                # use SHL runner
                print(program_content["value"])
                await asyncio.sleep(1)
                print(f"Finished running {program_content}")
                return program_content

            yield program


@pytest.fixture
def novax_with_program_sources():
    """Example using the new protocol-based approach"""
    # Create a standard program manager
    program_manager = ProgramManager()

    # Register program sources instead of using inheritance
    webdav_source = WebDAVProgramSource(webdav_mock)
    program_manager.register_program_source(webdav_source)

    # You can register multiple sources
    # filesystem_source = FileSystemProgramSource(Path("./programs"))
    # program_manager.register_program_source(filesystem_source)

    novax = Novax(program_manager_override=program_manager)
    app = novax.create_app()
    novax.include_programs_router(app)

    # Register Python programs (existing functionality)
    novax.register_program(simple_program)
    return app


def test_register_program_source():
    """Test registering a program source with the program manager"""
    program_manager = ProgramManager()

    # Create a mock WebDAV data source
    webdav_data = {"test_program1": '{"value": "test1"}', "test_program2": '{"value": "test2"}'}

    # Create and register a program source
    source = WebDAVProgramSource(webdav_data)
    program_manager.register_program_source(source)

    # Verify the source was registered
    assert len(program_manager._program_sources) == 1
    assert source in program_manager._program_sources


def test_deregister_program_source():
    """Test unregistering a program source"""
    program_manager = ProgramManager()

    source = WebDAVProgramSource({})
    program_manager.register_program_source(source)

    # Verify registration
    assert source in program_manager._program_sources

    # Unregister
    program_manager.deregister_program_source(source)

    # Verify unregistration
    assert source not in program_manager._program_sources


@pytest.mark.asyncio
async def test_discover_programs_from_source():
    """Test that programs are discovered and registered from a source"""
    program_manager = ProgramManager()

    # Create a mock WebDAV data source
    webdav_data = {"test_program1": '{"value": "test1"}', "test_program2": '{"value": "test2"}'}

    # Create and register a program source
    source = WebDAVProgramSource(webdav_data)
    program_manager.register_program_source(source)

    # Verify programs were registered
    programs = await program_manager.get_programs()
    assert "test_program1" in programs
    assert "test_program2" in programs


@pytest.mark.asyncio
async def test_multiple_program_sources():
    """Test using multiple program sources"""
    program_manager = ProgramManager()

    # Create multiple sources
    source1 = WebDAVProgramSource({"program1": '{"value": "test1"}'})
    source2 = WebDAVProgramSource({"program2": '{"value": "test2"}'})

    # Register both sources
    program_manager.register_program_source(source1)
    program_manager.register_program_source(source2)

    # Verify programs from both sources were registered
    programs = await program_manager.get_programs()
    assert "program1" in programs
    assert "program2" in programs


def test_filesystem_program_source_creation():
    """Test creating a filesystem program source"""
    # This test just verifies the source can be created
    # In a real scenario, you'd need actual .ws files
    test_path = Path("./test_programs")
    source = FileSystemProgramSource(test_path)

    assert source.directory_path == test_path
