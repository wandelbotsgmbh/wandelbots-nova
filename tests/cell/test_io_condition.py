"""Unit tests for the SDK-side IO condition evaluation used to resume IO pauses."""

from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from nova import api
from nova.cell.io_condition import (
    IOConditionWatcher,
    condition_holds,
    io_name,
    io_scalar,
    motion_enable_signal,
)

pytestmark = pytest.mark.asyncio

C = api.models.Comparator


@pytest.mark.parametrize(
    ("measured", "comparator", "expected", "holds"),
    [
        (True, C.COMPARATOR_EQUALS, True, True),
        (False, C.COMPARATOR_EQUALS, True, False),
        (False, C.COMPARATOR_NOT_EQUALS, True, True),
        (5, C.COMPARATOR_GREATER, 3, True),
        (3, C.COMPARATOR_GREATER, 3, False),
        (3, C.COMPARATOR_GREATER_EQUAL, 3, True),
        (2.5, C.COMPARATOR_LESS, 3.0, True),
        (3.0, C.COMPARATOR_LESS_EQUAL, 3.0, True),
    ],
)
def test_condition_holds_follows_the_api_comparator_semantics(
    measured, comparator, expected, holds
):
    assert condition_holds(measured, comparator, expected) is holds


def test_io_scalar_unwraps_wire_values():
    assert io_scalar(api.models.IOBooleanValue(io="a", value=True)) is True
    # Integers travel as strings "to avoid precision loss".
    assert io_scalar(api.models.IOIntegerValue(io="a", value="7")) == 7
    assert io_scalar(api.models.IOFloatValue(io="a", value=1.5)) == 1.5
    # A oneOf wrapper exposes the concrete value on ``actual_instance``.
    wrapped = SimpleNamespace(actual_instance=api.models.IOBooleanValue(io="a", value=False))
    assert io_scalar(wrapped) is False
    assert io_name(wrapped) == "a"


def _pause_on(io: str, origin: api.models.IOOrigin, value: bool = True) -> api.models.PauseOnIO:
    return api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io=io, value=value),
        comparator=C.COMPARATOR_EQUALS,
        io_origin=origin,
    )


def _stream(*frames):
    """A ``stream_io_values`` double: yields the given frames (lists of IO values) then ends
    like the generated client does when the socket closes."""

    def stream_io_values(cell, controller, ios):
        async def gen():
            for frame in frames:
                if isinstance(frame, Exception):
                    raise frame
                yield SimpleNamespace(io_values=frame)

        return gen()

    return stream_io_values


def _gateway(controller_values=(), bus_values=(), stream=None):
    return SimpleNamespace(
        controller_ios_api=SimpleNamespace(
            list_io_values=AsyncMock(side_effect=list(controller_values)),
            stream_io_values=stream or _stream(),
        ),
        bus_ios_api=SimpleNamespace(get_bus_io_values=AsyncMock(side_effect=list(bus_values))),
    )


def _value(io: str, value: bool) -> list:
    return [api.models.IOBooleanValue(io=io, value=value)]


async def test_holds_reads_the_origin_the_condition_names():
    gateway = _gateway(
        controller_values=[_value("OUT#1", True)], bus_values=[_value("hold", False)]
    )
    watcher = IOConditionWatcher(gateway, "cell", "ctrl")

    assert await watcher.holds(_pause_on("OUT#1", api.models.IOOrigin.CONTROLLER)) is True
    assert await watcher.holds(_pause_on("hold", api.models.IOOrigin.BUS_IO)) is False

    gateway.controller_ios_api.list_io_values.assert_awaited_once_with(
        cell="cell", controller="ctrl", ios=["OUT#1"]
    )
    gateway.bus_ios_api.get_bus_io_values.assert_awaited_once_with(cell="cell", ios=["hold"])


async def test_controller_io_release_is_observed_on_the_value_stream_not_by_polling():
    """The controller-IO REST endpoint answers 429 to a write while a read is in
    flight (measured), so the watcher must not poll it: the value stream carries
    the release."""
    stream = _stream(_value("OUT#1", True), _value("OUT#1", True), _value("OUT#1", False))
    gateway = _gateway(stream=stream)
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", poll_interval_secs=0)

    await watcher.wait_until_released(_pause_on("OUT#1", api.models.IOOrigin.CONTROLLER))

    gateway.controller_ios_api.list_io_values.assert_not_awaited()


async def test_controller_io_stream_is_reopened_and_then_falls_back_to_polling():
    """A stream that keeps closing (the generated client returns silently on
    ConnectionClosed) is reopened a few times, then the poll fallback takes over."""
    gateway = _gateway(
        controller_values=[_value("OUT#1", True), _value("OUT#1", False)],
        stream=_stream(),  # ends immediately every time
    )
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", poll_interval_secs=0)

    await watcher.wait_until_released(_pause_on("OUT#1", api.models.IOOrigin.CONTROLLER))

    assert gateway.controller_ios_api.list_io_values.await_count == 2


async def test_bus_io_release_is_polled_and_retries_transient_read_errors():
    reads = [
        _value("hold", True),
        RuntimeError("transient"),
        _value("hold", True),
        _value("hold", False),
    ]
    gateway = _gateway(bus_values=reads)
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", poll_interval_secs=0)

    await watcher.wait_until_released(_pause_on("hold", api.models.IOOrigin.BUS_IO))

    assert gateway.bus_ios_api.get_bus_io_values.await_count == 4


async def test_polling_gives_up_after_a_long_run_of_errors():
    gateway = _gateway(bus_values=[RuntimeError("down")] * 60)
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", poll_interval_secs=0)

    with pytest.raises(RuntimeError, match="down"):
        await watcher.wait_until_released(_pause_on("hold", api.models.IOOrigin.BUS_IO))


async def test_motion_enable_signal_pauses_while_the_signal_is_low():
    """True = allowed to move; False (or a lost signal, which reads False) = pause."""
    condition = motion_enable_signal("enable", api.models.IOOrigin.BUS_IO)
    gateway = _gateway(bus_values=[_value("enable", False), _value("enable", True)])
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", poll_interval_secs=0)

    assert condition.io_origin == api.models.IOOrigin.BUS_IO
    assert await watcher.holds(condition) is True, "signal low -> pause condition holds"
    assert await watcher.holds(condition) is False, "signal high -> free to move"
