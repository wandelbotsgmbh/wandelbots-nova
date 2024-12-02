import wandelbots_api_client as wb
from decouple import config
from wandelbots.core.controller import Controller
from wandelbots.core.exceptions import ControllerNotFoundException

NOVA_HOST = config("NOVA_HOST")
NOVA_USERNAME = config("NOVA_USERNAME", default=None)
NOVA_PASSWORD = config("NOVA_PASSWORD", default=None)
NOVA_ACCESS_TOKEN = config("NOVA_ACCESS")


def use_nova(
    host: str = NOVA_HOST,
    username: str | None = NOVA_USERNAME,
    password: str | None = NOVA_PASSWORD,
    access_token: str | None = NOVA_ACCESS_TOKEN,
    version: str = "v1",
) -> wb.ApiClient:
    config = wb.Configuration(
        host=f"http://{host}/api/{version}",
        username=username,
        password=password,
        access_token=access_token,
        ssl_ca_cert=False,
    )
    return wb.ApiClient(config)


def use_nova_basic_auth(
    host: str = NOVA_HOST,
    username: str | None = NOVA_USERNAME,
    password: str | None = NOVA_PASSWORD,
    version: str = "v1",
) -> wb.ApiClient:
    return use_nova(host=host, username=username, password=password, access_token=None, version=version)


def use_nova_access_token(
    host: str = NOVA_HOST, access_token: str | None = NOVA_ACCESS_TOKEN, version: str = "v1"
) -> wb.ApiClient:
    return use_nova(host=host, username=None, password=None, access_token=access_token, version=version)


class Nova:
    def __init__(
        self,
        *,
        host: str | None = None,
        username: str | None = None,
        password: str | None = None,
        access_token: str | None = None,
        version: str = "v1",
    ):
        self._api_client = self._init_api_client(host, username, password, access_token, version)


    def _init_api_client(self,
        host: str | None = None,
        username: str | None = None,
        password: str | None = None,
        access_token: str | None = None,
        version: str = "v1",
    ) -> wb.ApiClient:
        if host is None:
            host = config("NOVA_HOST")

        if username is None:
            username = config("NOVA_USERNAME", default=None)

        if password is None:
            password = config("NOVA_PASSWORD", default=None)

        if access_token is None:
            access_token = config("NOVA_ACCESS", default=None)

        api_client_config = wb.Configuration(
            host=f"http://{host}/api/{version}",
            username=username,
            password=password,
            access_token=access_token,
            ssl_ca_cert=False,
        )
        return wb.ApiClient(api_client_config)


    def cell(self, cell_id: str = "cell") -> "Cell":
        # TODO check if the cell exists
        return Cell(self._api_client, cell_id)

class Cell:
    def __init__(self, api_client: wb.ApiClient, cell_id: str):
        self._api_client = api_client
        self._controller_api = wb.ControllerApi(api_client=self._api_client)
        self._cell_id = cell_id

    async def controller(self, controller_host: str = None) -> "Controller":
        controller_list = await self._controller_api.list_controllers(cell=self._cell_id)
        found_controller = next((c for c in controller_list.instances if c.host == controller_host), None)

        if found_controller is None:
            raise ControllerNotFoundException(controller=controller_host)

        return Controller(api_client=self._api_client, cell=self._cell_id, controller_host=found_controller.host)