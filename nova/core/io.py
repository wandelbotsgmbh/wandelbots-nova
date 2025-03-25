from __future__ import annotations

import asyncio
from enum import Enum

from nova.api import models
from nova.core.gateway import ApiGateway
from nova.core.robot_cell import Device, ValueType


class IOType(Enum):
    IO_TYPE_INPUT = "IO_TYPE_INPUT"
    IO_TYPE_OUTPUT = "IO_TYPE_OUTPUT"


class IOValueType(Enum):
    IO_VALUE_ANALOG_INTEGER = "IO_VALUE_ANALOG_INTEGER"
    IO_VALUE_ANALOG_FLOATING = "IO_VALUE_ANALOG_FLOAT"
    IO_VALUE_DIGITAL = "IO_VALUE_BOOLEAN"


class ComparisonType(Enum):
    COMPARISON_TYPE_EQUAL = "COMPARATOR_EQUALS"
    COMPARISON_TYPE_GREATER = "COMPARATOR_GREATER"
    COMPARISON_TYPE_LESS = "COMPARATOR_LESS"


class IOAccess(Device):
    """Provides access to input and outputs via a dictionary-like style

    TODO:
        - Add listener to value changes
        - Handle integer masks
        - Read and check types based on the description
    """

    io_descriptions_cache: dict[str, dict[str, models.IODescription]] = {}

    def __init__(self, api_gateway: ApiGateway, cell: str, controller_id: str):
        super().__init__()
        self._api_gateway = api_gateway
        self._controller_ios_api = api_gateway.controller_ios_api
        self._cell = cell
        self._controller_id = controller_id
        self._io_operation_in_progress = asyncio.Lock()

    async def get_io_descriptions(self) -> dict[str, models.IODescription]:
        cache = self.__class__.io_descriptions_cache
        if self._controller_id not in cache:
            # empty list fetches all
            response = await self._controller_ios_api.list_io_descriptions(
                cell=self._cell, controller=self._controller_id, ios=None
            )
            cache[self._controller_id] = {
                description.id: description for description in response.io_descriptions
            }
        return cache[self._controller_id]

    @staticmethod
    def filter_io_descriptions(
        io_descriptions: dict[str, models.IODescription],
        filter_value_type: IOValueType | None = None,
        filter_type: IOType | None = None,
    ) -> list[str]:
        return [
            io.id
            for io in io_descriptions.values()
            if filter_value_type is None
            or (IOValueType(io.value_type) == filter_value_type and IOType(io.type) == filter_type)
        ]

    async def read(self, key: str) -> bool | int | float:
        """Reads a value from a given IO"""
        async with self._io_operation_in_progress:
            values: models.ListIOValuesResponse = await self._controller_ios_api.list_io_values(
                cell=self._cell, controller=self._controller_id, ios=[key]
            )
            io_value: models.SetOutputValuesRequestInner = values.io_values[0]

        if io_value.actual_instance is None:
            raise ValueError(f"Error while reading the IO value")

        if isinstance(io_value.actual_instance, models.IOBooleanValue):
            return io_value.actual_instance.boolean_value
        if isinstance(io_value.actual_instance, models.IOFloatValue):
            return io_value.actual_instance.floating_value
        if isinstance(io_value.actual_instance, models.IOIntegerValue):
            return io_value.actual_instance.integer_value

    async def write(self, key: str, value: ValueType):
        """Set a value asynchronously (So a direct read after setting might return still the old value)"""
        io_descriptions = await self.get_io_descriptions()
        io_description = io_descriptions[key]
        io_value_type = IOValueType(io_description.value_type)
        if isinstance(value, bool):
            if io_value_type is not IOValueType.IO_VALUE_DIGITAL:
                raise ValueError(
                    f"Boolean value can only be set at an IO_VALUE_DIGITAL IO and not to {io_value_type}"
                )
            io_value = models.IOBooleanValue(io=key, boolean_value=value)
        elif isinstance(value, int):
            if io_value_type is not IOValueType.IO_VALUE_ANALOG_INTEGER:
                raise ValueError(
                    f"Integer value can only be set at an IO_VALUE_ANALOG_INTEGER IO and not to {io_value_type}"
                )
            io_value = models.IOIntegerValue(io=key, integer_value=str(value))  # TODO: handle mask
        elif isinstance(value, float):
            if io_value_type is not IOValueType.IO_VALUE_ANALOG_FLOATING:
                raise ValueError(
                    f"Float value can only be set at an IO_VALUE_ANALOG_FLOATING IO and not to {io_value_type}"
                )
            io_value = models.IOFloatValue(io=key, floating_value=value)
        else:
            raise ValueError(f"Unexpected type {type(value)}")

        async with self._io_operation_in_progress:
            await self._controller_ios_api.set_output_values(
                cell=self._cell,
                controller=self._controller_id,
                set_output_values_request_inner=[models.SetOutputValuesRequestInner(io_value)],
            )

    async def wait_for_bool_io(self, io_id: str, value: bool):
        """Blocks until the requested IO equals the provided value."""
        # TODO proper implementation utilising also the comparison operators
        await self._controller_ios_api.wait_for_io_event(
            cell=self._cell,
            controller=self._controller_id,
            io=io_id,
            comparison_type=ComparisonType.COMPARISON_TYPE_EQUAL,
            boolean_value=value,
        )
