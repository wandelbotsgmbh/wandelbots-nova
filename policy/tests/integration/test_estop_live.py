"""Live e-stop detection against a real virtual controller.

Drives a real virtual UR10e into e-stop via ``set_estop`` and asserts the
``EstopMonitor`` fires. Requires NOVA_API / NOVA_ACCESS_TOKEN; skipped unless
run with ``-m integration``.
"""

from __future__ import annotations

import asyncio

import pytest

from nova import api
from policy.estop import EstopMonitor
from policy.types import EmergencyStopError


@pytest.mark.integration
@pytest.mark.asyncio
async def test_estop_monitor_detects_real_set_estop():
    from nova import Nova
    from nova.cell import virtual_controller

    async with Nova() as nova_instance:
        cell = nova_instance.cell()
        controller = await cell.ensure_controller(
            virtual_controller(
                name="ur10e-estop-mon",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        )
        mg = controller[0]
        monitor = EstopMonitor([mg])
        await monitor.start()
        try:
            await controller.set_estop(active=True)
            for _ in range(50):
                if monitor.error is not None:
                    break
                await asyncio.sleep(0.1)
            assert isinstance(monitor.error, EmergencyStopError)
        finally:
            await monitor.stop()
            await controller.set_estop(active=False)
