import pytest

from nova import Nova


@pytest.mark.skip
@pytest.mark.asyncio
async def test_instance(nova_api):
    nova = Nova(host=nova_api)
    cell = nova.cell()
    controller = await cell.controller("ur")
    assert controller is not None
