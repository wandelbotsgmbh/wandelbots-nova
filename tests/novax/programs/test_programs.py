"""
Simple test programs for integration testing.
"""

import asyncio

import httpx
import pytest

import nova
from nova.cell.cycle import CycleFailedEvent, CycleFinishedEvent, CycleStartedEvent
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


@pytest.mark.asyncio
async def test_novax_program_successful_run(novax_server):
    nova = Nova()
    await nova.connect()
    nats_client = nova.api_gateway._nats_client

    program_run_message = []

    async def cb(msg):
        program_run_message.append(msg)

    await nats_client.subscribe("nova.cells.cell.programs", cb)

    start_program = httpx.post(
        f"{novax_server}/programs/sucessful_program/start", json={"arguments": {}}
    )
    assert start_program.status_code == 200, "Failed to start test program"

    await asyncio.sleep(2)

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


@pytest.mark.asyncio
async def test_novax_program_failed_run(novax_server):
    nova = Nova()
    await nova.connect()
    nats_client = nova.api_gateway._nats_client

    program_run_message = []

    async def cb(msg):
        program_run_message.append(msg)

    await nats_client.subscribe("nova.cells.cell.programs", cb)

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
    id="program_with_cycle_data",
    name="Test cycle",
    description="A program sends cycle started and finished events",
)
async def program_with_cycle_data():
    async with Nova() as nova:
        cell = nova.cell()
        cycle = cell.cycle()
        await cycle.start()
        await cycle.finish()


@pytest.mark.asyncio
async def test_novax_program_cycle_data(novax_server):
    nova = Nova()
    await nova.connect()
    nats_client = nova.api_gateway._nats_client

    cycle_messages = []

    async def cb(msg):
        cycle_messages.append(msg)

    await nats_client.subscribe("nova.cells.cell.cycle", cb)

    start_program = httpx.post(
        f"{novax_server}/programs/program_with_cycle_data/start", json={"arguments": {}}
    )
    assert start_program.status_code == 200, "Failed to start test program"

    await asyncio.sleep(5)

    assert len(cycle_messages) == 2, f"Expected 2 cycle messages, but got {len(cycle_messages)}"

    cycle_started = CycleStartedEvent.model_validate_json(cycle_messages[0].data)
    cycle_finished = CycleFinishedEvent.model_validate_json(cycle_messages[1].data)

    assert cycle_started.event_type == "cycle_started"
    assert cycle_finished.event_type == "cycle_finished"
    await nova.close()


@nova.program(
    id="program_with_cycle_failure",
    name="Test cycle failed",
    description="A program that report cycle failure",
)
async def program_with_cycle_failure():
    async with Nova() as nova:
        cell = nova.cell()
        cycle = cell.cycle()
        await cycle.start()
        await cycle.fail("This cycle failed for testing purposes")


@pytest.mark.asyncio
async def test_novax_program_cycle_failure(novax_server):
    nova = Nova()
    await nova.connect()
    nats_client = nova.api_gateway._nats_client

    cycle_messages = []

    async def cb(msg):
        cycle_messages.append(msg)

    await nats_client.subscribe("nova.cells.cell.cycle", cb)

    start_program = httpx.post(
        f"{novax_server}/programs/program_with_cycle_failure/start", json={"arguments": {}}
    )
    assert start_program.status_code == 200, "Failed to start test program"

    await asyncio.sleep(5)

    assert len(cycle_messages) == 2, f"Expected 2 cycle messages, but got {len(cycle_messages)}"

    cycle_started = CycleStartedEvent.model_validate_json(cycle_messages[0].data)
    cycle_failed = CycleFailedEvent.model_validate_json(cycle_messages[1].data)

    assert cycle_started.event_type == "cycle_started"
    assert cycle_failed.event_type == "cycle_failed"
    await nova.close()
