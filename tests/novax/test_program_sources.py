import asyncio
from pathlib import Path
from typing import AsyncIterator

import pytest

import nova
from novax import Novax
from novax.program_manager import ProgramManager, WandelscriptProgramSource


@nova.program(name="simple_program")
async def simple_program(number_of_steps: int = 30):
    print("Hello World!")

    for i in range(number_of_steps):
        print(f"Step: {i}")
        await asyncio.sleep(1)

    print("Finished Hello World!")


@pytest.fixture
def wandelscript_source():
    return WandelscriptProgramSource(
        scan_paths=[Path("./examples/wandelscript_ffi.ws")],
        foreign_functions_paths=[Path("./examples/wandelscript_ffi.py")],
    )


@pytest.fixture
def novax_with_program_sources(wandelscript_source):
    """Example using the new protocol-based approach"""
    # Create a standard program manager
    program_manager = ProgramManager()

    # Register program sources instead of using inheritance
    program_manager.register_program_source(wandelscript_source)

    novax = Novax(program_manager_override=program_manager)
    app = novax.create_app()
    novax.include_programs_router(app)

    # Register Python programs (existing functionality)
    novax.register_program(simple_program)
    return app


def test_register_program_source(wandelscript_source):
    """Test registering a program source with the program manager"""
    program_manager = ProgramManager()
    program_manager.register_program_source(wandelscript_source)

    # Verify the source was registered
    assert len(program_manager._program_sources) == 1
    assert wandelscript_source in program_manager._program_sources


def test_deregister_program_source(wandelscript_source):
    """Test unregistering a program source"""
    program_manager = ProgramManager()
    program_manager.register_program_source(wandelscript_source)

    # Verify registration
    assert wandelscript_source in program_manager._program_sources

    # Unregister
    program_manager.deregister_program_source(wandelscript_source)

    # Verify unregistration
    assert wandelscript_source not in program_manager._program_sources


@pytest.mark.asyncio
async def test_discover_programs_from_source(wandelscript_source):
    """Test that programs are discovered and registered from a source"""
    program_manager = ProgramManager()
    program_manager.register_program_source(wandelscript_source)

    # Verify programs were registered
    programs = await program_manager.get_programs()
    assert "wandelscript_ffi" in programs


@pytest.mark.asyncio
async def test_multiple_program_sources(wandelscript_source):
    """Test using multiple program sources"""
    from novax.program_manager import Program, ProgramSource

    class TestProgramSource(ProgramSource):
        async def get_programs(self, program_manager: ProgramManager) -> AsyncIterator[Program]:
            yield simple_program

    program_manager = ProgramManager()

    # Register both sources
    program_manager.register_program_source(TestProgramSource())
    program_manager.register_program_source(wandelscript_source)

    # Verify programs from both sources were registered
    programs = await program_manager.get_programs()
    assert "simple_program" in programs
    assert "wandelscript_ffi" in programs
