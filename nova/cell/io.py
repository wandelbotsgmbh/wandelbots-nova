from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Any, Callable

from pydantic import TypeAdapter

from nova import api
from nova.cell.robot_cell import Device, ValueType
from nova.core.gateway import ApiGateway

from nova.core.nova import Nova

logger = logging.getLogger(__name__)


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


@dataclass
class IOChange:
    """
    Represents a change in an IO value.

    :param old_value: The previous value of the IO. None if it did not exist.
    :param new_value: The new value of the IO. None if the IO is no longer present
    """

    old_value: bool | float | int | None
    new_value: bool | float | int | None


def _convert_value(value: bool | str | float | None) -> bool | int | float | None:
    """Convert string values to integers, pass through other types."""
    if isinstance(value, str):
        return int(value)
    return value


async def get_bus_io_value(
    nova: Nova, ios: str | list[str], cell: str = "cell"
) -> dict[str, bool | int | float]:
    """Reads values from the given bus IOs."""
    io_list = [ios] if isinstance(ios, str) else list(ios)
    values = await nova.api.bus_ios_api.get_bus_io_values(cell=cell, ios=io_list)
    if not values:
        raise ValueError(f"No bus IO values returned for {io_list}")

    result: dict[str, bool | int | float] = {}
    for io_value in values:
        input_output = io_value.root
        if isinstance(input_output, api.models.IOBooleanValue):
            result[input_output.io] = bool(input_output.value)
        elif isinstance(input_output, api.models.IOIntegerValue):
            result[input_output.io] = int(input_output.value)
        elif isinstance(input_output, api.models.IOFloatValue):
            result[input_output.io] = float(input_output.value)
        else:
            raise ValueError(
                "Bus IO value for "
                f"{input_output.io} is of an unexpected type. Expected bool, int or float. "
                f"Got: {type(input_output)}"
            )

    missing = [io for io in io_list if io not in result]
    if missing:
        raise ValueError(f"No bus IO values returned for {missing}")

    return result


async def set_bus_io_value(
    nova: Nova,
    io_values: dict[str, bool | int | float],
    cell: str = "cell",
) -> None:
    """Sets values for the given bus IOs."""
    io_value_list: list[api.models.IOValue] = []
    for io, value in io_values.items():
        io_value: api.models.IOBooleanValue | api.models.IOIntegerValue | api.models.IOFloatValue
        if isinstance(value, bool):
            io_value = api.models.IOBooleanValue(io=io, value=value)
        elif isinstance(value, int):
            io_value = api.models.IOIntegerValue(io=io, value=str(value))
        elif isinstance(value, float):
            io_value = api.models.IOFloatValue(io=io, value=value)
        else:
            raise ValueError(f"Invalid value type {type(value)}. Expected bool, int or float.")
        io_value_list.append(api.models.IOValue(io_value))

    await nova.api.bus_ios_api.set_bus_io_values(
        cell=cell, io_value=io_value_list
    )


async def wait_for_io(
    nova: Nova,
    bus_ios: list[str],
    on_change: Callable[[dict[str, IOChange]], bool],
    cell: str = "cell",
    nats_subscription_kwargs: dict[str, Any] = {},
) -> None:
    """
    Wait for changes on specified bus IOs and call the on_change callback when changes occur.
    The function will continue to listen for changes until the on_change callback returns True.

    Note that this function does not guarantee that all IO changes are captured.
    Underlying NATS subscription guarantees at-most-once delivery when the consumer is healty.

    Provided nats_subscription_kwargs should be used to tune the subscription behavior.
    Additional tuning behaviour can be achieved by adjusting the NATS client configuration in the Nova instance.


    :param nova: The Nova instance to use. It should have a connected NATS client.
    :param bus_ios: A list of bus IO names to monitor for changes.
    :param on_change: A callback function that takes a dictionary of IO changes and returns a boolean.
                      If the callback returns True, the function will stop listening for changes.
    :param cell: The cell name to monitor (default is "cell").
    :param nats_subscription_kwargs: Additional keyword arguments to pass to the NATS subscription.
                                     Refer to nats.subscribe() for available options.
    """
    if not nova.nats.is_connected:
        raise RuntimeError("NATS client is not connected")

    if "subject" in nats_subscription_kwargs:
        raise ValueError("wait_for_io does not allow overriding the subscription subject")
    if "cb" in nats_subscription_kwargs or "future" in nats_subscription_kwargs:
        raise ValueError("wait_for_io does not support cb or future subscriptions")

    sub = await nova.nats.subscribe(f"nova.v2.cells.{cell}.bus-ios.ios", **nats_subscription_kwargs)

    try:
        old_values = await nova.api.bus_ios_api.get_bus_io_values(cell=cell, ios=bus_ios)
        old_values_dict = {value.root.io: value for value in old_values}

        async for message in sub.messages:
            logger.debug("Received bus IO update message: %s", message.data.decode())

            new_values = TypeAdapter(list[api.models.IOValue]).validate_json(message.data)
            new_values_dict = {value.root.io: value for value in new_values}

            should_terminate = on_change(
                {
                    io: IOChange(
                        old_value=_convert_value(
                            old_values_dict[io].root.value if io in old_values_dict else None
                        ),
                        new_value=_convert_value(
                            new_values_dict[io].root.value if io in new_values_dict else None
                        ),
                    )
                    for io in bus_ios
                }
            )

            if should_terminate:
                break

            old_values_dict = new_values_dict
    finally:
        await sub.unsubscribe()
