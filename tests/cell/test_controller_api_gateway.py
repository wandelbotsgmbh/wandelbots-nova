"""Controllers must reuse their cell's api gateway instead of opening a second one.

Each ApiGateway owns an api.ApiClient, which lazily creates an aiohttp.ClientSession on its
first request. A controller that builds its own gateway therefore leaks a session unless it is
explicitly closed, which the program runner never does.
"""

from nova import Nova
from nova.cell.controller import Controller
from nova.config import NovaConfig

CONFIG = NovaConfig(host="http://localhost")


def _standalone_controller() -> Controller:
    return Controller(
        configuration=Controller.Configuration(
            cell_id="cell", controller_id="demo", id="demo", config=CONFIG
        )
    )


def test_controller_from_cell_shares_the_nova_api_client():
    nova = Nova(config=CONFIG)
    controller = nova.cell()._create_controller("demo")

    assert controller._nova_api is nova.api
    assert controller._nova_api._api_client is nova.api._api_client
    assert controller._owns_api_gateway is False


def test_motion_group_shares_the_nova_api_client():
    nova = Nova(config=CONFIG)
    motion_group = nova.cell()._create_controller("demo")[0]

    assert motion_group._api_client._api_client is nova.api._api_client


async def test_closing_a_controller_does_not_close_the_shared_gateway():
    """Nova outlives the controllers handed out by its cell, so its session must survive."""
    nova = Nova(config=CONFIG)
    controller = nova.cell()._create_controller("demo")

    closed = False

    async def spy():
        nonlocal closed
        closed = True

    nova.api.close = spy  # ty: ignore[invalid-assignment]

    await controller.close()
    assert closed is False, "controller closed a gateway it does not own"

    await nova.api.close()
    assert closed is True


async def test_standalone_controller_owns_and_closes_its_gateway():
    controller = _standalone_controller()
    gateway = controller._nova_api

    assert controller._owns_api_gateway is True

    await controller.close()

    rest_client = gateway._api_client.rest_client
    assert rest_client.pool_manager is None or rest_client.pool_manager.closed
