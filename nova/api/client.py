from typing import Any

from nova.api.types import ControllerIO

from .types import ApiInterface
from .v1 import client as v1_client
from .v2 import client as v2_client


class ApiClient(ApiInterface):
    def __init__(
        self,
        host: str,
        username: str,
        password: str,
        access_token: str,
        verify_ssl: bool = True,
        version: str = "v1",
    ):
        if version == "v1":
            self.client = v1_client.ApiClient()
        elif version == "v2":
            self.client = v2_client.ApiClient()
        else:
            raise ValueError(f"Unsupported API version: {version}")

    async def list_io_values(
        self, cell: str, controller: str, ios: list[str]
    ) -> list[ControllerIO]:
        return await self.client.list_io_values(cell, controller, ios)

    async def set_io_value(self, cell: str, controller: str, io_name: str, value: Any) -> None:
        return await self.client.set_io_value(cell, controller, io_name, value)

    async def close(self) -> None:
        return await self.client.close()
