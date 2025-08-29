"""
Simple test programs for integration testing.
"""

import asyncio

import httpx
import pytest

import nova
from nova.core.nova import Nova
from nova.events import Cycle, CycleFailedEvent, CycleFinishedEvent, CycleStartedEvent


@nova.program(
    id="program_with_cycle_data",
    name="Test cycle",
    description="A program sends cycle started and finished events",
)
async def program_with_cycle_data():
    async with Nova() as nova:
        cell = nova.cell()
        cycle = Cycle(cell=cell)
        await cycle.start()
        await cycle.finish()


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def _test_novax_program_cycle_data(novax_server):
    nova = Nova()
    await nova.connect()

    cycle_messages = []

    async def cb(msg):
        cycle_messages.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.cycle", on_message=cb)

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
        cycle = Cycle(cell=cell)
        await cycle.start()
        await cycle.fail("This cycle failed for testing purposes")


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def _test_novax_program_cycle_failure(novax_server):
    nova = Nova()
    await nova.connect()

    cycle_messages = []

    async def cb(msg):
        cycle_messages.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.cycle", on_message=cb)

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
