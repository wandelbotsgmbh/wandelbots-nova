from typing import Any

import wandelbots_api_client as wb

from nova.api.types import ApiInterface, ControllerIO
from nova.version import version as pkg_version


class ApiClient(ApiInterface):
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

        self._init_v1_client()
        self.controller_io = wb.ControllerIOsApi(api_client=self._api_client)

    def _init_v1_client(self):
        api_client_config = wb.Configuration(
            host=f"{self._host}/api/v1",
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
        """Get the current values of specified IOs"""
        response = await self.controller_io.list_io_values(
            cell=cell, controller=controller, ios=ios
        )

        result = []
        for io_data in response.io_values:
            result.append(ControllerIO(name=io_data.name, type=io_data.type, value=io_data.value))
        return result

    async def set_io_value(self, cell: str, controller: str, io_name: str, value: Any) -> None:
        """Set the value of a specific IO"""
        io_value = wb.models.IOValue(name=io_name, value=value)
        await self.controller_io.set(cell=cell, controller=controller, io_value=io_value)

    async def close(self) -> None:
        """Close the API client session"""
        await self._api_client.close()
