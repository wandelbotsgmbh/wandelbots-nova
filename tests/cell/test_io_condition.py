"""Unit tests for the SDK-side IO condition evaluation used to resume IO pauses.

Nothing may poll the API: controller IOs come from the value-stream websocket, bus IOs
from NATS pushes (one initial read after subscribing), and an unobservable source fails.
"""

import asyncio
import json
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from nova import api
from nova.cell.io_condition import (
    IOConditionUnavailable,
    IOConditionWatcher,
    condition_holds,
    io_name,
    io_scalar,
    motion_enable_signal,
)
from tests.cell.fake_nats import FakeNats

pytestmark = pytest.mark.asyncio

C = api.models.Comparator
VALUES = "nova.v2.cells.cell.bus-ios.ios"
STATUS = "nova.v2.cells.cell.bus-ios.status"


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


def _value(io: str, value: bool) -> list:
    return [api.models.IOBooleanValue(io=io, value=value)]


def _pushed(io: str, value: bool) -> bytes:
    return json.dumps([{"io": io, "value": value, "value_type": "boolean"}]).encode()


def _status(state: str) -> bytes:
    return json.dumps({"state": state}).encode()


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


def _gateway(controller_values=(), bus_values=(), bus_states=(), stream=None):
    return SimpleNamespace(
        controller_ios_api=SimpleNamespace(
            list_io_values=AsyncMock(side_effect=list(controller_values)),
            stream_io_values=stream or _stream(),
        ),
        bus_ios_api=SimpleNamespace(
            get_bus_io_values=AsyncMock(side_effect=list(bus_values)),
            get_bus_io_state=AsyncMock(side_effect=list(bus_states)),
        ),
    )


def _bus_state(state: api.models.BusIOsStateEnum):
    return SimpleNamespace(state=state)


CONNECTED = api.models.BusIOsStateEnum.BUS_IOS_STATE_CONNECTED
DISCONNECTED = api.models.BusIOsStateEnum.BUS_IOS_STATE_DISCONNECTED


async def _settle(rounds: int = 10) -> None:
    for _ in range(rounds):
        await asyncio.sleep(0)


# -- single reads ---------------------------------------------------------------


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


async def test_motion_enable_signal_pauses_while_the_signal_is_low():
    """True = allowed to move; False (or a lost signal, which reads False) = pause."""
    condition = motion_enable_signal("enable", api.models.IOOrigin.BUS_IO)
    gateway = _gateway(bus_values=[_value("enable", False), _value("enable", True)])
    watcher = IOConditionWatcher(gateway, "cell", "ctrl")

    assert condition.io_origin == api.models.IOOrigin.BUS_IO
    assert await watcher.holds(condition) is True, "signal low -> pause condition holds"
    assert await watcher.holds(condition) is False, "signal high -> free to move"


# -- controller IOs: value stream -------------------------------------------------


async def test_controller_io_release_is_observed_on_the_value_stream_not_by_polling():
    """The controller-IO REST endpoint answers 429 to a write while a read is in
    flight (measured), so the watcher must not poll it: the value stream carries
    the release."""
    stream = _stream(_value("OUT#1", True), _value("OUT#1", True), _value("OUT#1", False))
    gateway = _gateway(stream=stream)
    watcher = IOConditionWatcher(gateway, "cell", "ctrl")

    await watcher.wait_until_released(_pause_on("OUT#1", api.models.IOOrigin.CONTROLLER))

    gateway.controller_ios_api.list_io_values.assert_not_awaited()


async def test_controller_io_stream_that_keeps_closing_fails_instead_of_polling(monkeypatch):
    monkeypatch.setattr("nova.cell.io_condition._STREAM_REOPEN_DELAY_SECS", 0)
    gateway = _gateway(controller_values=[_value("OUT#1", False)] * 10, stream=_stream())
    watcher = IOConditionWatcher(gateway, "cell", "ctrl")

    with pytest.raises(IOConditionUnavailable):
        await watcher.wait_until_released(_pause_on("OUT#1", api.models.IOOrigin.CONTROLLER))

    gateway.controller_ios_api.list_io_values.assert_not_awaited()


# -- bus IOs: NATS push -------------------------------------------------------------


async def test_bus_io_release_reads_once_then_waits_for_pushed_values():
    nats = FakeNats()
    gateway = _gateway(bus_values=[_value("enable", False)])  # signal low -> still paused
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=nats)
    condition = motion_enable_signal("enable")

    wait = asyncio.create_task(watcher.wait_until_released(condition))
    await nats.subscribed.wait()
    await _settle()
    assert not wait.done()
    assert nats.subjects() == {VALUES, STATUS}

    await nats.publish(VALUES, _pushed("other", True))  # unrelated IO
    await nats.publish(VALUES, b"null")  # the bus went away
    await _settle()
    assert not wait.done()

    await nats.publish(VALUES, _pushed("enable", True))  # allowed to move again
    async with asyncio.timeout(2):
        await wait

    gateway.bus_ios_api.get_bus_io_values.assert_awaited_once()
    assert nats.subscriptions == [], "subscriptions are released"


async def test_bus_io_release_returns_immediately_when_the_initial_read_already_allows():
    nats = FakeNats()
    gateway = _gateway(bus_values=[_value("enable", True)])
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=nats)

    async with asyncio.timeout(2):
        await watcher.wait_until_released(motion_enable_signal("enable"))
    assert nats.subscriptions == []


async def test_bus_io_release_re_reads_once_when_the_bus_comes_back():
    """While the bus is gone the initial read fails; when the status subject reports
    CONNECTED the value is asked for once more (values are not necessarily re-published)."""
    nats = FakeNats()
    gateway = _gateway(bus_values=[RuntimeError("404 BusIONotFound"), _value("enable", True)])
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=nats)

    wait = asyncio.create_task(watcher.wait_until_released(motion_enable_signal("enable")))
    await nats.subscribed.wait()
    await _settle()
    assert not wait.done()

    await nats.publish(STATUS, _status(""))  # still down
    await _settle()
    assert not wait.done()

    await nats.publish(STATUS, _status(CONNECTED.value))
    async with asyncio.timeout(2):
        await wait
    assert gateway.bus_ios_api.get_bus_io_values.await_count == 2


async def test_bus_io_condition_without_nats_fails_instead_of_polling():
    gateway = _gateway(bus_values=[_value("enable", False)] * 5)
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=FakeNats(connected=False))

    with pytest.raises(IOConditionUnavailable):
        await watcher.wait_until_released(motion_enable_signal("enable"))
    gateway.bus_ios_api.get_bus_io_values.assert_not_awaited()


# -- bus loss -------------------------------------------------------------------


async def test_bus_io_loss_is_reported_from_the_status_subject():
    nats = FakeNats()
    gateway = _gateway(bus_states=[_bus_state(CONNECTED)])
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=nats)

    wait = asyncio.create_task(watcher.wait_until_bus_io_lost())
    await nats.subscribed.wait()
    await _settle()
    assert not wait.done()

    await nats.publish(STATUS, _status(CONNECTED.value))
    await _settle()
    assert not wait.done()

    await nats.publish(STATUS, _status(""))  # what the service publishes when removed
    async with asyncio.timeout(2):
        await wait
    assert nats.subscriptions == []


async def test_bus_io_loss_is_immediate_when_the_bus_is_already_down():
    nats = FakeNats()
    gateway = _gateway(bus_states=[_bus_state(DISCONNECTED)])
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=nats)
    async with asyncio.timeout(2):
        await watcher.wait_until_bus_io_lost()

    gateway = _gateway(bus_states=[RuntimeError("404 BusIONotFound")])
    watcher = IOConditionWatcher(gateway, "cell", "ctrl", nats_client=nats)
    async with asyncio.timeout(2):
        await watcher.wait_until_bus_io_lost()
