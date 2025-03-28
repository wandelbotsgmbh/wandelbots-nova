# nova/experimental/api/client.py

from typing import Any

import wandelbots_api_client as wb_v1
import wandelbots_api_client.v2 as wb_v2

from nova.api.types import ApiInterface, ControllerIO
from nova.version import version as pkg_version

from .operation_registry import OPERATIONS


class DeclarativeApiClient(ApiInterface):
    """
    A single API client that declaratively supports v1 and v2.
    Each operation is looked up in an operation registry, which
    holds separate v1/v2 logic.
    """

    def __init__(
        self,
        host: str,
        username: str = None,
        password: str = None,
        access_token: str = None,
        verify_ssl: bool = True,
        version: str = "v1",
    ):
        self._host = host
        self._username = username
        self._password = password
        self._access_token = access_token
        self._verify_ssl = verify_ssl
        self._version = version

        # Sub-clients for v1 or v2
        self._api_client_v1 = None
        self._controller_v1 = None

        self._api_client_v2 = None
        self._controller_v2 = None

        if self._version == "v1":
            self._init_v1()
        elif self._version == "v2":
            self._init_v2()
        else:
            raise ValueError(f"Unsupported API version: {self._version}")

        # Build a dispatch map from operation name -> the chosen (v1 or v2) implementation
        self._impl_map = {}
        for op_name, operation in OPERATIONS.items():
            if self._version == "v1":
                self._impl_map[op_name] = operation.v1_impl
            else:
                self._impl_map[op_name] = operation.v2_impl

    def _init_v1(self):
        config = wb_v1.Configuration(
            host=f"{self._host}/api/v1",
            username=self._username,
            password=self._password,
            access_token=self._access_token,
            verify_ssl=self._verify_ssl,
        )
        self._api_client_v1 = wb_v1.ApiClient(configuration=config)
        self._api_client_v1.user_agent = f"Wandelbots-Nova-Python-SDK/{pkg_version}"
        self._controller_v1 = wb_v1.ControllerIOsApi(api_client=self._api_client_v1)

    def _init_v2(self):
        config = wb_v2.Configuration(
            host=f"{self._host}/api/v2",
            username=self._username,
            password=self._password,
            access_token=self._access_token,
            verify_ssl=self._verify_ssl,
        )
        self._api_client_v2 = wb_v2.ApiClient(configuration=config)
        self._api_client_v2.user_agent = f"Wandelbots-Nova-Python-SDK/{pkg_version}"
        self._controller_v2 = wb_v2.ControllerInputsOutputsApi(api_client=self._api_client_v2)

    # ----------------------------------------------------------------
    # ApiInterface implementations
    # ----------------------------------------------------------------

    async def list_io_values(
        self, cell: str, controller: str, ios: list[str]
    ) -> list[ControllerIO]:
        """
        Calls the chosen v1/v2 impl from _impl_map.
        """
        return await self._impl_map["list_io_values"](self, cell, controller, ios)

    async def set_io_value(self, cell: str, controller: str, io_name: str, value: Any) -> None:
        """
        Calls the chosen v1/v2 impl from _impl_map.
        """
        return await self._impl_map["set_io_value"](self, cell, controller, io_name, value)

    async def close(self) -> None:
        """
        Closes whichever subclient(s) is active.
        """
        if self._api_client_v1:
            await self._api_client_v1.close()
        if self._api_client_v2:
            await self._api_client_v2.close()
