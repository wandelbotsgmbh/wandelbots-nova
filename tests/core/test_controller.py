from nova.core.controller import Controller
import pytest

import wandelbots_api_client


@pytest.mark.asyncio
@pytest.mark.skip
async def test_instance(nova_api_client):
    controller_api = wandelbots_api_client.ControllerApi(api_client=nova_api_client)
    controllers = await controller_api.list_controllers(cell="cell")
    print(controllers)
    assert False

