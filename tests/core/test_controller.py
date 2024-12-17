import pytest

from nova import api


@pytest.mark.asyncio
@pytest.mark.skip
async def test_instance(nova_api_client):
    controller_api = api.ControllerApi(api_client=nova_api_client)
    controllers = await controller_api.list_controllers(cell="cell")
    print(controllers)
    assert False
