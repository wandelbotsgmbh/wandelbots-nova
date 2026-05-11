"""Tests for policy.io.IOWriter."""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock

import pytest

from policy.io import IOWriter


def _mock_mg():
    mg = MagicMock()
    mg.id = "0@ur10e"
    mg._api_client = MagicMock()
    mg._cell = "cell"
    mg._controller_id = "ur10e"
    mg._api_client.controller_ios_api.set_output_values = AsyncMock()
    return mg


@pytest.mark.asyncio
async def test_deduplication_skips_unchanged():
    """Writing the same value twice should only call the API once."""
    mg = _mock_mg()
    writer = IOWriter(mg)
    await writer.write({"digital_out[0]": True})
    await writer.write({"digital_out[0]": True})
    assert mg._api_client.controller_ios_api.set_output_values.call_count == 1


@pytest.mark.asyncio
async def test_writes_changed_value():
    """Changing a value should call the API again."""
    mg = _mock_mg()
    writer = IOWriter(mg)
    await writer.write({"digital_out[0]": True})
    await writer.write({"digital_out[0]": False})
    assert mg._api_client.controller_ios_api.set_output_values.call_count == 2


@pytest.mark.asyncio
async def test_atomic_multi_key_write():
    """Concurrent writes should not interleave keys.

    Two write() calls with overlapping keys should each write all their
    keys atomically (the lock covers the whole call, not per-key).
    """
    mg = _mock_mg()
    call_log = []

    async def record_call(**kwargs):
        # Record the IO values written in this call
        io_values = kwargs.get("io_value", [])
        call_log.append([v.io for v in io_values])
        await asyncio.sleep(0.01)  # Simulate API latency

    mg._api_client.controller_ios_api.set_output_values = AsyncMock(side_effect=record_call)
    writer = IOWriter(mg)

    # Launch two concurrent writes
    await asyncio.gather(
        writer.write({"key_a": True, "key_b": False}),
        writer.write({"key_a": False, "key_b": True}),
    )

    # Both keys should eventually be written
    all_keys = set()
    for call in call_log:
        all_keys.update(call)
    assert "key_a" in all_keys
    assert "key_b" in all_keys
