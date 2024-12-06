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


@pytest.mark.asyncio
@pytest.mark.skip
async def test_controller(nova_api_client):
    controller = Controller(nova_api_client, cell="cell", controller_host="ur10e")
    async with controller:
        motion_groups = controller.get_motion_groups()
        print(motion_groups)
    assert False
