import asyncio

import pytest
from nats import NATS

from nova.core.nova import Nova
from nova.program.runner import ProgramRun


@pytest.mark.asyncio
async def test_novax_starts(novax_server):
    nova = Nova()
    await nova.connect()
    nats_client: NATS = nova._api_client._nats_client._nats_client

    program_run_message = []

    async def cb(msg):
        program_run_message.append(msg)

    sub = await nats_client.subscribe("nova.cells.cell.programs.*", cb=cb)

    start_program = novax_server.post("/programs/test_simple/start", json={"arguments": {}})
    assert start_program.status_code == 200, "Failed to start test program"

    program_run = ProgramRun.model_validate(start_program.json())

    await asyncio.sleep(10)
    # Assert that we received a message
    assert len(program_run_message) > 0, "No program run messages received"
