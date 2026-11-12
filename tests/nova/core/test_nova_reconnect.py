import asyncio
from unittest.mock import AsyncMock

import pytest

from nova.config import NovaConfig
from nova.core.nova import Nova


def _make_nova(**config_kwargs) -> Nova:
    config = NovaConfig(host="http://localhost:8080", **config_kwargs)
    nova = Nova(config)
    nova.nats.connect = AsyncMock()
    return nova


@pytest.fixture
def connected_flag():
    """Lets the test control what `nats.is_connected` reports after `connect()`."""
    return {"value": False}


async def test_open_registers_default_connection_callbacks(connected_flag, monkeypatch):
    nova = _make_nova()
    monkeypatch.setattr(
        type(nova.nats), "is_connected", property(lambda self: connected_flag["value"])
    )

    await nova.open()

    kwargs = nova.nats.connect.call_args.kwargs
    assert kwargs["error_cb"] == nova._on_nats_error
    assert kwargs["disconnected_cb"] == nova._on_nats_disconnected
    assert kwargs["reconnected_cb"] == nova._on_nats_reconnected
    assert kwargs["closed_cb"] == nova._on_nats_closed


async def test_open_does_not_override_explicit_callbacks(connected_flag, monkeypatch):
    custom_closed_cb = AsyncMock()
    nova = _make_nova(
        nats_client_config={"servers": "nats://x:4222", "closed_cb": custom_closed_cb}
    )
    monkeypatch.setattr(
        type(nova.nats), "is_connected", property(lambda self: connected_flag["value"])
    )

    await nova.open()

    assert nova.nats.connect.call_args.kwargs["closed_cb"] is custom_closed_cb


async def test_closed_callback_reopens_the_connection(connected_flag, monkeypatch):
    nova = _make_nova()
    monkeypatch.setattr(
        type(nova.nats), "is_connected", property(lambda self: connected_flag["value"])
    )

    async def connect_side_effect(*args, **kwargs):
        connected_flag["value"] = True

    nova.nats.connect.side_effect = connect_side_effect

    await nova.open()
    assert nova.nats.connect.call_count == 1

    # Simulate nats-py giving up on its own reconnect attempts: the server (or network)
    # has dropped the connection and all built-in reconnect attempts failed.
    connected_flag["value"] = False
    await nova._on_nats_closed()
    assert nova._nats_reopen_task is not None

    await asyncio.wait_for(nova._nats_reopen_task, timeout=5)

    assert nova.nats.connect.call_count == 2


async def test_close_prevents_reopen_after_closed_callback(connected_flag, monkeypatch):
    nova = _make_nova()
    monkeypatch.setattr(
        type(nova.nats), "is_connected", property(lambda self: connected_flag["value"])
    )

    await nova.open()
    await nova.close()

    await nova._on_nats_closed()

    assert nova._nats_reopen_task is None
    assert nova.nats.connect.call_count == 1


async def test_is_connected_holds_while_nats_py_reconnects(monkeypatch):
    """A reconnecting client queues calls, so the SDK keeps using it instead of failing."""
    nova = _make_nova()
    monkeypatch.setattr(type(nova.nats), "is_connected", property(lambda self: False))
    monkeypatch.setattr(type(nova.nats), "is_reconnecting", property(lambda self: True))

    assert nova.is_connected()

    monkeypatch.setattr(type(nova.nats), "is_closed", property(lambda self: True))
    assert not nova.is_connected()
