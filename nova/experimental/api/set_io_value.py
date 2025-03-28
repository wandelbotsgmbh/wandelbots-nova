# nova/experimental/api/set_io_value.py

from typing import Any

import wandelbots_api_client as wb_v1
import wandelbots_api_client.v2 as wb_v2

from nova.experimental.api.client import DeclarativeApiClient

from .operation_types import APIOperation


async def set_io_value_v1(
    client: DeclarativeApiClient, cell: str, controller: str, io_name: str, value: Any
) -> None:
    """
    Same logic as your original V1 code in ApiClient.set_io_value.
    """
    io_value = wb_v1.models.IOValue(name=io_name, value=value)
    await client._controller_v1.set(cell=cell, controller=controller, io_value=io_value)


async def set_io_value_v2(
    client: DeclarativeApiClient, cell: str, controller: str, io_name: str, value: Any
) -> None:
    """
    Same logic as your original V2 code in ApiClient.set_io_value.
    """
    io_value = wb_v2.ControllerIOValue(name=io_name, value=value)
    await client._controller_v2.set_controller_input_output(
        cell_id=cell, controller_id=controller, controller_io_value=io_value
    )


SET_IO_VALUE_OPERATION = APIOperation(
    name="set_io_value", v1_impl=set_io_value_v1, v2_impl=set_io_value_v2
)
