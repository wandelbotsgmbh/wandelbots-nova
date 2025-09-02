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


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def _test_novax_program_successful_run(novax_server):
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

    await asyncio.sleep(5)

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


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def _test_novax_program_failed_run(novax_server):
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
