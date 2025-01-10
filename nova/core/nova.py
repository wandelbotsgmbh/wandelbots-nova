import wandelbots_api_client as wb
from decouple import config

from nova.core.controller import Controller
from nova.core.exceptions import ControllerNotFoundException
from nova.core.logging_setup import configure_logging
from nova.gateway import ApiGateway


class Nova:
    def __init__(
        self,
        *,
        host: str | None = None,
        username: str | None = None,
        password: str | None = None,
        access_token: str | None = None,
        version: str = "v1",
        verify_ssl: bool = True,
        log_level: str = "INFO",
    ):
        configure_logging(log_level)
        self._api_client = ApiGateway(
            host=host,
            username=username,
            password=password,
            access_token=access_token,
            version=version,
            verify_ssl=verify_ssl,
        )

    def cell(self, cell_id: str = config("CELL_NAME", default="cell")) -> "Cell":
        return Cell(self._api_client, cell_id)


class Cell:
    def __init__(self, api_gateway: ApiGateway, cell_id: str):
        self._api_gateway = api_gateway
        self._cell_id = cell_id

    async def _get_controllers(self) -> list[wb.models.ControllerInstance]:
        response = await self._api_gateway.controller_api.list_controllers(cell=self._cell_id)
        return response.instances

    async def controllers(self) -> list["Controller"]:
        controllers = await self._get_controllers()
        return [
            Controller(api_gateway=self._api_gateway, cell=self._cell_id, controller_host=c.host)
            for c in controllers
        ]

    async def controller(self, controller_host: str) -> "Controller":
        controllers = await self._get_controllers()
        found_controller = next((c for c in controllers if c.host == controller_host), None)

        if found_controller is None:
            raise ControllerNotFoundException(controller=controller_host)

        return Controller(
            api_gateway=self._api_gateway, cell=self._cell_id, controller_host=found_controller.host
        )
