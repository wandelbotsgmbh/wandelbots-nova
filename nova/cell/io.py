from __future__ import annotations

import asyncio

from nova import api
from nova.cell.robot_cell import Device, ValueType
from nova.core.gateway import ApiGateway


class IOAccess(Device):
    """Provides access to input and outputs via a dictionary-like style

    TODO:
        - Add listener to value changes
        - Handle integer masks
        - Read and check types based on the description
    """

    io_descriptions_cache: dict[str, dict[str, api.models.IODescription]] = {}

    def __init__(self, api_client: ApiGateway, cell: str, controller_id: str):
        super().__init__()
        self._api_client = api_client
        self._cell = cell
        self._controller_id = controller_id
        self._io_operation_in_progress = asyncio.Lock()

    async def get_io_descriptions(self) -> dict[str, api.models.IODescription]:
        cache = self.__class__.io_descriptions_cache
        if self._controller_id not in cache:
            io_descriptions = await self._api_client.controller_ios_api.list_io_descriptions(
                cell=self._cell, controller=self._controller_id, ios=[]
            )

            cache[self._controller_id] = {
                description.io: description for description in io_descriptions
            }
        return cache[self._controller_id]

    @staticmethod
    def filter_io_descriptions(
        io_descriptions: dict[str, api.models.IODescription],
        filter_value_type: api.models.IOValueType | None = None,
        filter_io_direction: api.models.IODirection | None = None,
    ) -> list[str]:
        return [
            io_description.io
            for io_description in io_descriptions.values()
            if filter_value_type is None
            or (
                api.models.IOValueType(io_description.value_type) == filter_value_type
                and api.models.IODirection(io_description.direction) == filter_io_direction
            )
        ]

    async def read(self, key: str) -> bool | int | float:
        """Reads a value from a given IO"""
        response = await self._api_client.controller_ios_api.list_io_values(
            cell=self._cell, controller=self._controller_id, ios=[key]
        )

        input_output = response[0]

        if isinstance(input_output.root, api.models.IOBooleanValue):
            return bool(input_output.root.value)
        elif isinstance(input_output.root, api.models.IOIntegerValue):
            return int(input_output.root.value)
        elif isinstance(input_output.root, api.models.IOFloatValue):
            return float(input_output.root.value)

        raise ValueError(
            f"IO value for {key} is of an unexpected type. Expected bool, int or float. Got: {type(input_output)}"
        )

    async def write(self, key: str, value: ValueType):
        """Set a value asynchronously (So a direct read after setting might return still the old value)"""
        await self._ensure_value_type(key, value)

        async with self._io_operation_in_progress:
            io_value: (
                api.models.IOBooleanValue | api.models.IOIntegerValue | api.models.IOFloatValue
            )

            if isinstance(value, bool):
                io_value = api.models.IOBooleanValue(io=key, value=value)
            elif isinstance(value, int):
                io_value = api.models.IOIntegerValue(io=key, value=str(value))
            elif isinstance(value, float):
                io_value = api.models.IOFloatValue(io=key, value=value)
            else:
                raise ValueError(f"Invalid value type {type(value)}. Expected bool, int or float.")

            await self._api_client.controller_ios_api.set_output_values(
                cell=self._cell, controller=self._controller_id, io_value=[io_value]
            )

    async def _ensure_value_type(self, key: str, value: ValueType):
        """Checks if the provided value matches the expected type of the IO"""
        io_descriptions = await self.get_io_descriptions()
        io_description = io_descriptions[key]
        io_value_type = api.models.IOValueType(io_description.value_type)
        if isinstance(value, bool):
            if io_value_type is not api.models.IOValueType.IO_VALUE_BOOLEAN:
                raise ValueError(
                    f"Boolean value can only be set at an IO_VALUE_BOOLEAN IO and not to {io_value_type}"
                )
        elif isinstance(value, int):
            if io_value_type is not api.models.IOValueType.IO_VALUE_ANALOG_INTEGER:
                raise ValueError(
                    f"Integer value can only be set at an IO_VALUE_ANALOG_INTEGER IO and not to {io_value_type}"
                )
        elif isinstance(value, float):
            if io_value_type is not api.models.IOValueType.IO_VALUE_ANALOG_FLOAT:
                raise ValueError(
                    f"Float value can only be set at an IO_VALUE_ANALOG_FLOAT IO and not to {io_value_type}"
                )
        else:
            raise ValueError(f"Unexpected type {type(value)}")

    async def wait_for_bool_io(self, io: str, value: bool):
        """Blocks until the requested IO equals the provided value."""
        # TODO proper implementation utilising also the comparison operators
        io_value = api.models.IOBooleanValue(io=io, value=value)

        wait_request = api.models.WaitForIOEventRequest(
            io=api.models.IOValue(io_value), comparator=api.models.Comparator.COMPARATOR_EQUALS
        )
        await self._api_client.controller_ios_api.wait_for_io_event(
            cell=self._cell, controller=self._controller_id, wait_for_io_event_request=wait_request
        )
