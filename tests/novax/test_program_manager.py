import asyncio
import datetime as dt

import pytest

import nova
from nova.cell.simulation import SimulatedRobotCell
from novax.program_manager import ProgramManager


@nova.program(
    id="simple_program_test",
    name="Simple Program",
    description="Simple program that prints 'Hello World!' and then sleeps a bit.",
)
async def simple_program(number_of_steps: int = 30):
    """Simple program that prints 'Hello World!' and then sleeps a bit."""
    print("Hello World!")

    for i in range(number_of_steps):
        print(f"Step: {i}")
        await asyncio.sleep(1)

    print("Finished Hello World!")


@nova.program(name="long_running_program")
async def long_running_program():
    for i in range(10):
        await asyncio.sleep(0.1)
        if i == 5:
            # This will be interrupted by stop
            pass
    return "Long program completed"


@nova.program(name="parameterized_program")
async def parameterized_program(message: str = "default", count: int = 1):
    """A program that accepts parameters"""
    result = []
    for i in range(count):
        result.append(f"{message} - iteration {i}")
    return result


def test_register_program():
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register a program
    program_id = manager.register_program(simple_program)

    # Verify program is registered
    assert manager.has_program(program_id)
    assert program_id == "simple_program_test"

    # Verify program details are stored correctly
    assert program_id in manager._programs
    assert program_id in manager._program_functions

    program_details = manager._programs[program_id]
    assert program_details.name == "Simple Program"
    assert (
        program_details.description
        == "Simple program that prints 'Hello World!' and then sleeps a bit."
    )
    assert isinstance(program_details.created_at, dt.datetime)


def test_deregister_program():
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register a program
    program_id = manager.register_program(simple_program)
    assert manager.has_program(program_id)

    # Deregister the program
    manager.deregister_program(program_id)

    # Verify program is no longer registered
    assert not manager.has_program(program_id)
    assert program_id not in manager._programs
    assert program_id not in manager._program_functions


def test_deregister_nonexistent_program():
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())
    # This should not raise an error
    manager.deregister_program("nonexistent_program")


@pytest.mark.asyncio
async def test_start_program_success():
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register a program
    program_id = manager.register_program(simple_program)

    # Verify no program is running initially
    assert not manager.is_any_program_running
    assert manager.running_program is None

    # Start the program
    program_run = await manager.start_program(program_id, parameters={"number_of_steps": 5})

    # Verify program is running
    assert manager.is_any_program_running
    assert manager.running_program == program_id
    assert program_run is not None

    # Wait for program to complete
    await asyncio.sleep(10)

    # Verify program completed successfully
    assert not manager.is_any_program_running
    assert manager.running_program is None


@pytest.mark.asyncio
async def test_start_program_when_another_is_running():
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register programs
    manager.register_program(simple_program)

    # Start the first program
    await manager.start_program("simple_program_test", parameters={"number_of_steps": 5})

    # Try to start another program while one is running
    with pytest.raises(RuntimeError):
        await manager.start_program("simple_program_test")


@pytest.mark.asyncio
async def test_stop_running_program():
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register a long-running program
    program_id = manager.register_program(simple_program)

    # Start the program
    await manager.start_program(program_id)

    # Verify program is running
    assert manager.is_any_program_running
    assert manager.running_program == program_id

    # Stop the program
    await manager.stop_program(program_id)

    # Verify program is no longer running
    assert not manager.is_any_program_running
    assert manager.running_program is None


@pytest.mark.asyncio
async def test_stop_program_when_none_running():
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Try to stop a program when none is running
    with pytest.raises(RuntimeError):
        await manager.stop_program("test_program")


@pytest.mark.asyncio
async def test_stop_wrong_program():
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register programs
    program_id_1 = manager.register_program(long_running_program)
    program_id_2 = manager.register_program(simple_program)

    # Start the first program
    await manager.start_program(program_id_1)

    # Try to stop a different program
    try:
        await manager.stop_program(program_id_2)
        assert False, "Expected RuntimeError to be raised"
    except RuntimeError as e:
        assert f"Program {program_id_2} is not running" in str(e)
        assert f"Currently running: {program_id_1}" in str(e)


@pytest.mark.asyncio
async def test_get_programs():
    """Test getting all registered programs"""
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register multiple programs
    manager.register_program(simple_program)
    manager.register_program(parameterized_program)

    # Get all programs
    programs = await manager.get_programs()

    # Verify programs are returned
    assert len(programs) == 2
    assert "simple_program_test" in programs
    assert "parameterized_program" in programs

    # Verify program details
    test_prog_details = programs["simple_program_test"]
    assert test_prog_details.name == "Simple Program"
    assert (
        test_prog_details.description
        == "Simple program that prints 'Hello World!' and then sleeps a bit."
    )


@pytest.mark.asyncio
async def test_get_program():
    """Test getting a specific program by ID"""
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register a program
    program_id = manager.register_program(simple_program)

    # Get the specific program
    program_details = await manager.get_program(program_id)

    # Verify program details
    assert program_details is not None
    assert program_details.name == "Simple Program"
    assert (
        program_details.description
        == "Simple program that prints 'Hello World!' and then sleeps a bit."
    )

    # Test getting non-existent program
    non_existent = await manager.get_program("nonexistent")
    assert non_existent is None
