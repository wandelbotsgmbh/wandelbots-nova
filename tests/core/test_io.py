import pytest

from nova import Nova
from nova.core.io import IOAccess, IOType, IOValueType


@pytest.mark.skip("TODO: Setup integration tests")
@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_io_descriptions():
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_gateway=nova._api_client, cell=cell.cell_id, controller_id="ur")
        io_descriptions = await io.get_io_descriptions()
        assert len(io_descriptions) > 0
        filtered_io_descriptions = IOAccess.filter_io_descriptions(
            io_descriptions, IOValueType.IO_VALUE_DIGITAL, IOType.IO_TYPE_INPUT
        )
        assert len(filtered_io_descriptions) < len(io_descriptions)


@pytest.mark.skip("TODO: Setup integration tests")
@pytest.mark.asyncio
@pytest.mark.integration
async def test_read():
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_gateway=nova._api_client, cell=cell.cell_id, controller_id="ur")
        value1 = await io.read("tool_out[0]")
        assert value1 is False
        value2 = await io.read("digital_out[0]")
        assert value2 is False


@pytest.mark.skip("TODO: Setup integration tests")
@pytest.mark.asyncio
@pytest.mark.integration
async def test_write():
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_gateway=nova._api_client, cell=cell.cell_id, controller_id="ur")
        value1 = await io.read("tool_out[0]")
        assert value1 is False
        await io.write("tool_out[0]", True)
        value2 = await io.read("tool_out[0]")
        assert value2 is True
        await io.write("tool_out[0]", False)
        value3 = await io.read("tool_out[0]")
        assert value3 is False
