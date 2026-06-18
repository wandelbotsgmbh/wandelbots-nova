"""Live e-stop detection against a throwaway virtual controller.

Provisions its own uniquely-named virtual UR10e (the create-on-demand pattern
the core nova integration tests use) and deletes it on teardown, drives it into
e-stop via ``set_estop``, and asserts the ``EstopMonitor`` fires. Requires
NOVA_API / NOVA_ACCESS_TOKEN; skipped unless run with ``-m integration``.
"""

from __future__ import annotations

import asyncio
import contextlib
import uuid

import pytest

from novapolicy.estop import EstopMonitor
from novapolicy.types import EmergencyStopError


@pytest.mark.integration
@pytest.mark.asyncio
async def test_estop_monitor_detects_real_set_estop():
    from nova import Nova, api
    from nova.cell import virtual_controller

    name = f"policy-itest-estop-{uuid.uuid4().hex[:8]}"

    async with Nova() as nova_instance:
        cell = nova_instance.cell()
        controller = await cell.ensure_controller(
            virtual_controller(
                name=name,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        )
        try:
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
        finally:
            with contextlib.suppress(Exception):
                await cell.delete_robot_controller(name, timeout=25)
