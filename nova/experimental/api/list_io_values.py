# nova/experimental/api/list_io_values.py

import wandelbots_api_client.v2 as wb_v2

from nova.api.types import ControllerIO
from nova.experimental.api.client import DeclarativeApiClient

from .operation_types import APIOperation


# ---------------------
# v1 logic
# ---------------------
async def list_io_values_v1(
    client: DeclarativeApiClient, cell: str, controller: str, ios: list[str]
) -> list[ControllerIO]:
    """
    Same logic as your original V1 code in ApiClient.list_io_values.
    """
    response = await client._controller_v1.list_io_values(cell=cell, controller=controller, ios=ios)

    result = []
    for io_data in response.io_values:
        # The original v1 code had name=io_data.name, value=io_data.value
        result.append(ControllerIO(name=io_data.name, value=io_data.value))
    return result


# ---------------------
# v2 logic
# ---------------------
async def list_io_values_v2(
    client: DeclarativeApiClient, cell: str, controller: str, ios: list[str]
) -> list[ControllerIO]:
    """
    Same logic as your original V2 code in ApiClient.list_io_values.
    """
    response: wb_v2.models.ListIOValuesResponse = await client._controller_v2.list_io_values(
        cell_id=cell, controller_id=controller, io_names=ios
    )
    result = []
    for io_item in response.io_values:
        actual = io_item.actual_instance
        value = actual.integer_value or actual.float_value or actual.boolean_value
        result.append(ControllerIO(name=actual.io, value=value))
    return result


# ---------------------
# Operation definition
# ---------------------
LIST_IO_VALUES_OPERATION = APIOperation(
    name="list_io_values", v1_impl=list_io_values_v1, v2_impl=list_io_values_v2
)
