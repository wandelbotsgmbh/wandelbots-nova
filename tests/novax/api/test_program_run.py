import asyncio

import httpx
import pytest

import nova
from nova.core.nova import Nova
from nova.program.runner import ProgramRun, ProgramRunState


@nova.program(
    id="sucessful_program",
    name="Simple Test Program",
    description="A simple program for testing program run reporting",
)
async def sucessful_program():
    """Simple program that counts and sleeps."""
    print("simple test program run")


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_successful_run(novax_server):
    nova = Nova()
    await nova.connect()

    program_run_message = []

    async def cb(msg):
        program_run_message.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.programs", on_message=cb)

    start_program = httpx.post(
        f"{novax_server}/programs/sucessful_program/start", json={"arguments": {}}
    )
    assert start_program.status_code == 200, "Failed to start test program"

    await asyncio.sleep(10)

    assert len(program_run_message) == 3, (
        f"Expected 3 program run messages, but got {len(program_run_message)}"
    )

    models = [ProgramRun.model_validate_json(message.data) for message in program_run_message]
    assert models[0].state == ProgramRunState.PREPARING
    assert models[1].state == ProgramRunState.RUNNING
    assert models[2].state == ProgramRunState.COMPLETED


@nova.program(
    id="failing_program",
    name="Failing Test Program",
    description="A program that always fails for testing error handling",
)
async def failing_program():
    """Program that always fails."""
    raise ValueError("This program is designed to fail for testing purposes")


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_failed_run(novax_server):
    nova = Nova()
    await nova.connect()

    program_run_message = []

    async def cb(msg):
        program_run_message.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.programs", on_message=cb)

    start_program = httpx.post(
        f"{novax_server}/programs/failing_program/start", json={"arguments": {}}
    )
    assert start_program.status_code == 200, "Failed to start test program"

    await asyncio.sleep(2)

    assert len(program_run_message) == 3, (
        f"Expected 3 program run messages, but got {len(program_run_message)}"
    )

    models = [ProgramRun.model_validate_json(message.data) for message in program_run_message]
    assert models[0].state == ProgramRunState.PREPARING
    assert models[1].state == ProgramRunState.RUNNING
    assert models[2].state == ProgramRunState.FAILED


@nova.program(
    id="long_running_program",
    name="Long Running Test Program",
    description="A program that takes some time to complete for testing stop functionality",
)
async def long_running_program():
    """Program that runs for a while and can be stopped."""
    for i in range(100):
        await asyncio.sleep(1)


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_stopped_run(novax_server):
    nova = Nova()
    await nova.connect()

    program_run_message = []

    async def cb(msg):
        program_run_message.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.programs", on_message=cb)

    # Start the long-running program
    start_program = httpx.post(
        f"{novax_server}/programs/long_running_program/start", json={"arguments": {}}
    )
    assert start_program.status_code == 200, "Failed to start test program"

    # Wait for the program to start running
    await asyncio.sleep(5)

    # Verify program is running
    assert len(program_run_message) >= 2, (
        f"Expected at least 2 program run messages, but got {len(program_run_message)}"
    )
    models = [ProgramRun.model_validate_json(message.data) for message in program_run_message]
    assert models[0].state == ProgramRunState.PREPARING
    assert models[1].state == ProgramRunState.RUNNING

    # Stop the program
    stop_program = httpx.post(f"{novax_server}/programs/long_running_program/stop")
    assert stop_program.status_code == 200, "Failed to stop test program"

    # Wait for the stop event to be processed
    await asyncio.sleep(5)

    # Verify that we received the STOPPED event
    assert len(program_run_message) == 3, (
        f"Expected 3 program run messages, but got {len(program_run_message)}"
    )

    final_models = [ProgramRun.model_validate_json(message.data) for message in program_run_message]
    assert final_models[0].state == ProgramRunState.PREPARING
    assert final_models[1].state == ProgramRunState.RUNNING
    assert final_models[2].state == ProgramRunState.STOPPED
