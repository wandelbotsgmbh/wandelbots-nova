"""Tests for IOSyncDriver bus-IO confirmation: clear polls until the value is
observed (correctness, before arming), while release re-reads exactly once off
the arm→move window. The controller-origin path is exercised through the executor
session tests."""

from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.actions.io import WriteAction, io_write
from nova.cell import multi_trajectory_cursor as mtc
from nova.cell.multi_trajectory_cursor import IOSyncDriver

pytestmark = pytest.mark.asyncio


def _bus_write(value: bool) -> WriteAction:
    return io_write("sync-io", value, origin=api.models.IOOrigin.BUS_IO)


def _watch() -> api.models.StartOnIO:
    return api.models.StartOnIO(
        io=api.models.IOBooleanValue(io="sync-io", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.BUS_IO,
    )


def _driver(api_client) -> IOSyncDriver:
    return IOSyncDriver(
        clear=_bus_write(False),
        release=_bus_write(True),
        watch={"a": _watch()},
        api_client=api_client,
        cell="cell",
    )


def _bus_gateway(get_values: list) -> MagicMock:
    gateway = MagicMock()
    gateway.bus_ios_api = MagicMock()
    gateway.bus_ios_api.set_bus_io_values = AsyncMock()
    gateway.bus_ios_api.get_bus_io_values = AsyncMock(side_effect=get_values)
    return gateway


@pytest.fixture(autouse=True)
def _no_release_delay(monkeypatch):
    # The deferred re-read timing is irrelevant to the assertions; skip the wait.
    monkeypatch.setattr(mtc, "_RELEASE_VERIFY_DELAY", 0.0)


async def test_release_bus_confirmed_by_a_single_read():
    # release writes, then re-reads exactly once (no poll loop) to confirm.
    io_value = _bus_write(True).to_api_model()
    gateway = _bus_gateway([[io_value]])

    await _driver(gateway).release()

    gateway.bus_ios_api.set_bus_io_values.assert_awaited_once()
    gateway.bus_ios_api.get_bus_io_values.assert_awaited_once()


async def test_release_bus_raises_on_dropped_write():
    # A release value that never reads back surfaces as an error, not a hang.
    stale = _bus_write(False).to_api_model()
    gateway = _bus_gateway([[stale]])

    with pytest.raises(RuntimeError, match="did not take the release value"):
        await _driver(gateway).release()


async def test_clear_bus_polls_until_observed():
    # clear is correctness: it blocks until the not-released value reads back.
    io_value = _bus_write(False).to_api_model()
    gateway = _bus_gateway([[], [io_value]])  # miss, then hit

    await _driver(gateway).clear()

    assert gateway.bus_ios_api.get_bus_io_values.await_count == 2
