import asyncio
import logging
from typing import AsyncGenerator

import pytest

from nova import Nova, api
from nova.cell import virtual_controller
from nova.cell.controller import Controller
from nova.cell.io import IOAccess

logger = logging.getLogger(__name__)


@pytest.fixture(scope="module")
@pytest.mark.integration
async def setup_ur() -> AsyncGenerator[tuple[Controller, Controller], None]:
    async with Nova() as nova:
        cell = nova.cell()

        ur = await cell.ensure_controller(
            virtual_controller(
                name="ur-test",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
            )
        )

        # kuka has special IO naming conventions, so we test that as well
        kuka = await cell.ensure_controller(
            virtual_controller(
                name="kuka-test",
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_KR16_R2010_2,
            )
        )

        # wait for controllers to be ready
        for i in range(10):
            try:
                await cell.controller("ur-test")
                await cell.controller("kuka-test")
                break
            except Exception:
                logger.error("Controllers not ready yet, waiting...")
                await asyncio.sleep(2)

        yield ur, kuka


@pytest.mark.asyncio
@pytest.mark.integration
async def test_io_write(setup_ur: tuple[Controller, Controller]):
    ur, kuka = setup_ur

    # Test UR IO write
    await ur.write("digital_in[0]", False)
    value = await ur.read("digital_in[0]")
    assert value is False

    await ur.write("digital_in[0]", True)
    value = await ur.read("digital_in[0]")
    assert value is True

    # Test Kuka IO write
    await kuka.write("OUT#555", False)
    value = await kuka.read("OUT#555")
    assert value is False

    await kuka.write("OUT#555", True)
    value = await kuka.read("OUT#555")
    assert value is True


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_io_descriptions(setup_ur: tuple[Controller, Controller]):
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_client=nova.apis, cell=cell.cell_id, controller_id="ur-io-test")
        io_descriptions = await io.get_io_descriptions()
        assert len(io_descriptions) > 0
        filtered_io_descriptions = IOAccess.filter_io_descriptions(
            io_descriptions,
            api.models.IOValueType.IO_VALUE_BOOLEAN,
            api.models.IODirection.IO_TYPE_INPUT,
        )
        assert len(filtered_io_descriptions) < len(io_descriptions)


@pytest.mark.asyncio
@pytest.mark.integration
async def test_read(setup_ur: tuple[Controller, Controller]):
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_client=nova.apis, cell=cell.cell_id, controller_id="ur-io-test")
        value1 = await io.read("tool_out[0]")
        assert value1 is False
        value2 = await io.read("digital_out[0]")
        assert value2 is False


@pytest.mark.asyncio
@pytest.mark.integration
async def test_write(setup_ur: tuple[Controller, Controller]):
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_client=nova.apis, cell=cell.cell_id, controller_id="ur-io-test")
        value1 = await io.read("tool_out[0]")
        assert value1 is False
        await io.write("tool_out[0]", True)
        value2 = await io.read("tool_out[0]")
        assert value2 is True
        await io.write("tool_out[0]", False)
        value3 = await io.read("tool_out[0]")
        assert value3 is False
