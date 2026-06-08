"""Tests for policy.io.IOWriter."""

from __future__ import annotations

import asyncio
import types
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.cell.io import IOAccess
from policy.io import IOWriter

_BOOL = api.models.IOValueType.IO_VALUE_BOOLEAN
_INT = api.models.IOValueType.IO_VALUE_ANALOG_INTEGER
_FLOAT = api.models.IOValueType.IO_VALUE_ANALOG_FLOAT


@pytest.fixture(autouse=True)
def _clear_io_description_cache():
    """IOAccess caches IO descriptions per controller at class level."""
    IOAccess.io_descriptions_cache.clear()
    yield
    IOAccess.io_descriptions_cache.clear()


def _mock_mg(descriptions: dict[str, object] | None = None):
    """Mock motion group whose controller reports the given IO value types.

    ``descriptions`` maps IO key -> IOValueType so IOAccess.write's validation
    roundtrip (``list_io_descriptions``) is satisfied.
    """
    mg = MagicMock()
    mg.id = "0@ur10e"
    mg._api_client = MagicMock()
    mg._cell = "cell"
    mg._controller_id = "ur10e"
    mg._api_client.controller_ios_api.set_output_values = AsyncMock()
    descs = descriptions or {}
    mg._api_client.controller_ios_api.list_io_descriptions = AsyncMock(
        return_value=[types.SimpleNamespace(io=k, value_type=v) for k, v in descs.items()]
    )
    return mg


@pytest.mark.asyncio
async def test_deduplication_skips_unchanged():
    """Writing the same value twice should only call the API once."""
    mg = _mock_mg({"digital_out[0]": _BOOL})
    writer = IOWriter(mg)
    await writer.write({"digital_out[0]": True})
    await writer.write({"digital_out[0]": True})
    assert mg._api_client.controller_ios_api.set_output_values.call_count == 1


@pytest.mark.asyncio
async def test_writes_changed_value():
    """Changing a value should call the API again."""
    mg = _mock_mg({"digital_out[0]": _BOOL})
    writer = IOWriter(mg)
    await writer.write({"digital_out[0]": True})
    await writer.write({"digital_out[0]": False})
    assert mg._api_client.controller_ios_api.set_output_values.call_count == 2


@pytest.mark.asyncio
async def test_value_type_dispatch_matches_core():
    """int -> IOIntegerValue, float -> IOFloatValue, bool -> IOBooleanValue.

    Delegated to nova IOAccess.write so analog-integer IOs aren't sent a float.
    """
    mg = _mock_mg({"b": _BOOL, "i": _INT, "f": _FLOAT})
    writer = IOWriter(mg)
    await writer.write({"b": True, "i": 7, "f": 1.5})

    sent = {
        v.io: v
        for call in mg._api_client.controller_ios_api.set_output_values.call_args_list
        for v in call.kwargs["io_value"]
    }
    assert isinstance(sent["b"], api.models.IOBooleanValue)
    assert isinstance(sent["i"], api.models.IOIntegerValue)
    assert isinstance(sent["f"], api.models.IOFloatValue)


@pytest.mark.asyncio
async def test_concurrent_writes_are_serialized_not_interleaved():
    """The per-instance lock serializes writes so they never overlap.

    IOAccess writes one key per API call, so without the lock two concurrent
    write() calls could interleave and leave the outputs in a mixed state.
    Here we assert the real guarantee directly: at most one write() is ever in
    flight at the API at a time.
    """
    mg = _mock_mg({"key_a": _BOOL, "key_b": _BOOL})
    in_flight = 0
    max_in_flight = 0

    async def record_call(**_kwargs):
        nonlocal in_flight, max_in_flight
        in_flight += 1
        max_in_flight = max(max_in_flight, in_flight)
        await asyncio.sleep(0.01)  # hold the API "open" to expose any overlap
        in_flight -= 1

    mg._api_client.controller_ios_api.set_output_values = AsyncMock(side_effect=record_call)
    writer = IOWriter(mg)

    await asyncio.gather(
        writer.write({"key_a": True, "key_b": False}),
        writer.write({"key_a": False, "key_b": True}),
    )

    assert max_in_flight == 1  # writes were serialized, never overlapping
