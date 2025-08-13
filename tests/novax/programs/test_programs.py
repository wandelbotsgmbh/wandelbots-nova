"""
Simple test programs for integration testing.
"""

import nova
from nova.core.nova import Nova


@nova.program(
    id="test_simple",
    name="Simple Test Program",
    description="A simple program for testing program run reporting",
)
async def test_program_run_succeded():
    """Simple program that counts and sleeps."""
    print("simple test program run")


@nova.program(
    id="test_failing",
    name="Failing Test Program",
    description="A program that always fails for testing error handling",
)
async def test_program_run_failed():
    """Program that always fails."""
    raise ValueError("This program is designed to fail for testing purposes")


@nova.program(
    id="test_cycle",
    name="Test cycle",
    description="A program sends cycle started and finished events",
)
async def test_cycle():
    async with Nova() as nova:
        cell = nova.cell()
        cycle = cell.cycle()
        await cycle.start()
        await cycle.finish()


@nova.program(
    id="test_cycle_failed",
    name="Test cycle failed",
    description="A program that report cycle failure",
)
async def test_cycle_failed():
    async with Nova() as nova:
        cell = nova.cell()
        cycle = cell.cycle()
        await cycle.start()
        await cycle.fail("This cycle failed for testing purposes")
