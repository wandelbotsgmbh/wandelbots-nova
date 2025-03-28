from typing import Any

import wandelbots_api_client.v2 as wb

from nova.api.types import ApiInterface, ControllerIO
from nova.version import version as pkg_version


class ApiClient(ApiInterface):
    """V2 implementation of the API client"""

    def __init__(
        self,
        host: str,
        username: str = None,
        password: str = None,
        access_token: str = None,
        verify_ssl: bool = True,
    ):
        self._host = host
        self._username = username
        self._password = password
        self._access_token = access_token
        self._verify_ssl = verify_ssl

        # Initialize V2 client
        self._api_client = None  # Initialize first to avoid reference error
        self._init_v2_client()
        self.controller_io = wb.ControllerInputsOutputsApi(api_client=self._api_client)

    def _init_v2_client(self):
        api_client_config = wb.Configuration(
            host=f"{self._host}/api/v2",
            username=self._username,
            password=self._password,
            access_token=self._access_token,
        )
        api_client_config.verify_ssl = self._verify_ssl
        self._api_client = wb.ApiClient(configuration=api_client_config)
        self._api_client.user_agent = f"Wandelbots-Nova-Python-SDK/{pkg_version}"

    async def list_io_values(
        self, cell: str, controller: str, ios: list[str]
    ) -> list[ControllerIO]:
        response: wb.models.ListIOValuesResponse = await self.controller_io.list_io_values(
            cell_id=cell, controller_id=controller, io_names=ios
        )

        # Convert the response to the expected ControllerIO format
        result = []
        for io_item in response.io_values:
            value = (
                io_item.actual_instance.integer_value
                or io_item.actual_instance.float_value
                or io_item.actual_instance.boolean_value
            )
            result.append(ControllerIO(key=io_item.actual_instance.io, value=value))
        return result

    async def set_io_value(self, cell: str, controller: str, io_name: str, value: Any) -> None:
        # Create IO value payload
        io_value = wb.ControllerIOValue(name=io_name, value=value)

        # Call the API to set the value
        await self.controller_io.set_controller_input_output(
            cell_id=cell, controller_id=controller, controller_io_value=io_value
        )

    async def close(self) -> None:
        if self._api_client:
            await self._api_client.close()
