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


async def test_reopen_waits_are_jittered_and_capped(connected_flag, monkeypatch):
    """Each wait is drawn from [0, delay]; the delay doubles per failure up to the cap."""
    nova = _make_nova()
    monkeypatch.setattr(
        type(nova.nats), "is_connected", property(lambda self: connected_flag["value"])
    )
    windows: list[tuple[float, float]] = []
    monkeypatch.setattr(
        "nova.core.nova.random.uniform", lambda lo, hi: windows.append((lo, hi)) or 0.0
    )
    monkeypatch.setattr("nova.core.nova._NATS_REOPEN_INITIAL_DELAY_SECS", 2.0)
    monkeypatch.setattr("nova.core.nova._NATS_REOPEN_MAX_DELAY_SECS", 8.0)

    failures = 4

    async def connect_side_effect(*args, **kwargs):
        if len(windows) <= failures:
            raise OSError("server unreachable")
        connected_flag["value"] = True

    nova.nats.connect.side_effect = connect_side_effect

    await asyncio.wait_for(nova._reopen_with_backoff(), timeout=5)

    assert windows == [(0, 2.0), (0, 4.0), (0, 8.0), (0, 8.0), (0, 8.0)]
    assert connected_flag["value"]


async def test_reopen_fires_reconnected(connected_flag, monkeypatch):
    """Consumers learn about a fallback reopen through the same signal as a reconnect."""
    nova = _make_nova()
    monkeypatch.setattr(
        type(nova.nats), "is_connected", property(lambda self: connected_flag["value"])
    )
    monkeypatch.setattr("nova.core.nova.random.uniform", lambda lo, hi: 0.0)

    async def connect_side_effect(*args, **kwargs):
        connected_flag["value"] = True

    nova.nats.connect.side_effect = connect_side_effect
    before = nova.nats.reconnected.generation

    await asyncio.wait_for(nova._reopen_with_backoff(), timeout=5)

    assert nova.nats.reconnected.generation == before + 1


async def test_reopen_task_stops_on_close(connected_flag, monkeypatch):
    nova = _make_nova()
    monkeypatch.setattr(
        type(nova.nats), "is_connected", property(lambda self: connected_flag["value"])
    )
    nova.nats.connect.side_effect = OSError("server unreachable")

    await nova._on_nats_closed()
    task = nova._nats_reopen_task
    assert task is not None

    await nova.close()
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(task, timeout=5)
    assert task.cancelled()


async def test_repeated_connects_keep_one_server_in_the_pool(monkeypatch):
    """nats-py appends to the pool on every connect(); a reopen must not grow it."""
    import nats

    client = Nova(NovaConfig(host="http://localhost:8080")).nats

    async def fake_connect(self, *args, servers=("nats://x:4222",), **kwargs):
        self._setup_server_pool(list(servers))

    monkeypatch.setattr(nats.NATS, "connect", fake_connect)

    for _ in range(3):
        await client.connect(servers=["nats://x:4222"])

    assert len(client._server_pool) == 1
