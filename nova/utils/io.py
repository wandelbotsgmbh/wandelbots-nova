import logging
from dataclasses import dataclass
from typing import Any, Callable

from pydantic.type_adapter import TypeAdapter

from nova import Nova, api, get_current_program_context

logger = logging.getLogger(__name__)


@dataclass
class IOChange:
    """
    Represents a change in an IO value.

    :param old_value: The previous value of the IO. None if it did not exist.
    :param new_value: The new value of the IO. None if the IO is no longer present
    """

    old_value: bool | float | int | None
    new_value: bool | float | int | None


def _resolve_nova(nova: Nova | None) -> Nova:
    """Resolve the Nova instance to use."""
    if nova is not None:
        return nova

    context = get_current_program_context()
    if context is None:
        raise RuntimeError("No Nova instance available in the current context")

    return context.nova


def _convert_value(value: bool | str | float | None) -> bool | int | float | None:
    """Convert string values to integers, pass through other types."""
    if isinstance(value, str):
        return int(value)
    return value


async def get_bus_io_value(
    ios: list[str], *, nova: Nova | None = None, cell: str = "cell"
) -> dict[str, bool | int | float]:
    """Reads values from the given bus IOs."""
    nova = _resolve_nova(nova)

    values = await nova.api.bus_ios_api.get_bus_io_values(cell=cell, ios=ios)

    values = [value for value in values if value.root.value is not None]
    return {value.root.io: _convert_value(value.root.value) for value in values}  # type: ignore


async def set_bus_io_value(
    io_values: dict[str, bool | int | float], *, nova: Nova | None = None, cell: str = "cell"
) -> None:
    """Sets values for the given bus IOs."""
    nova = _resolve_nova(nova)

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

    await nova.api.bus_ios_api.set_bus_io_values(cell=cell, io_value=io_value_list)


async def wait_for_bus_io(
    bus_ios: list[str],
    *,
    on_change: Callable[[dict[str, IOChange]], bool],
    nova: Nova | None = None,
    cell: str = "cell",
    nats_subscription_kwargs: dict[str, Any] | None = None,
) -> None:
    """
    Wait for changes on specified bus IOs and call the on_change callback when changes occur.
    The function will continue to listen for changes until the on_change callback returns True.

    Note that this function does not guarantee that all IO changes are captured.
    Underlying NATS subscription guarantees at-most-once delivery when the consumer is healthy.

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
    nova = _resolve_nova(nova)

    if not nova.nats.is_connected:
        raise RuntimeError("NATS client is not connected")

    if nats_subscription_kwargs is None:
        nats_subscription_kwargs = {}

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
