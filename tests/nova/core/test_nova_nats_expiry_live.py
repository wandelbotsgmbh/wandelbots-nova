"""Live-broker tests for ``Nova`` surviving an expired NATS login (marked ``nats``).

NOVA's auth callout issues user JWTs with a lifetime. When one runs out the server sends
``-ERR 'User Authentication Expired'`` and drops the socket. Stock nats-py treats that
``-ERR`` as fatal and closes the client for good; ``Nova`` must instead reconnect like the
other NATS clients in a cell do, without losing subscriptions or failing calls made while
the connection is down.

The server's part is replayed through nats-py's own read loop against a throwaway
``nats-server`` on a free port, so everything from the ``-ERR`` onwards (error handling,
reconnect, re-subscribe, flushing buffered publishes) is the real client code. The module
is skipped when the binary is not installed.
"""

import asyncio

import pytest

from nova.config import NovaConfig
from nova.core.nova import Nova
from tests.nats_live import (
    AUTHORIZATION_VIOLATION,
    expire_login,
    nats_server_url,
    server_sends,
    wait_connected,
)

pytestmark = pytest.mark.nats


@pytest.fixture(scope="module")
def nats_url():
    with nats_server_url() as url:
        yield url


@pytest.fixture
async def nova(nats_url):
    config = NovaConfig(
        host="http://127.0.0.1:9",  # never contacted: these tests only use NATS
        nats_client_config={
            "servers": nats_url,
            "reconnect_time_wait": 0.1,
            "max_reconnect_attempts": 20,
        },
    )
    client = Nova(config)
    await client.open()
    yield client
    await client.close()


async def test_expired_login_reconnects_and_keeps_subscriptions(nova):
    received: list[bytes] = []

    async def on_message(msg) -> None:
        received.append(msg.data)

    await nova.nats.subscribe("expiry.probe", cb=on_message)

    await expire_login(nova.nats)

    assert not nova.nats.is_closed, "the expiry must not close the client for good"
    assert "authentication expired" in str(nova.nats.last_error).lower()

    await wait_connected(nova.nats)

    await nova.nats.publish("expiry.probe", b"after")
    await nova.nats.flush(timeout=2)
    await asyncio.sleep(0.1)
    assert received == [b"after"], "the pre-expiry subscription must survive the re-login"


async def test_calls_made_while_reconnecting_go_through(nova):
    async def respond(msg) -> None:
        await msg.respond(b"pong")

    await nova.nats.subscribe("expiry.echo", cb=respond)

    await expire_login(nova.nats)

    # Neither raises: nats-py buffers both until the connection is back.
    await nova.nats.publish("expiry.noise", b"during gap")
    reply = await nova.nats.request("expiry.echo", b"ping", timeout=5)

    assert reply.data == b"pong"
    assert nova.is_connected()


async def test_subscriptions_made_while_reconnecting_are_live_afterwards(nova):
    """What the SDK's guards rely on when a wait starts inside the reconnect window."""
    received: list[bytes] = []

    async def on_message(msg) -> None:
        received.append(msg.data)

    await expire_login(nova.nats)
    assert nova.is_connected(), "a reconnecting client counts as usable"
    await nova.nats.subscribe("expiry.late", cb=on_message)

    await wait_connected(nova.nats)
    await nova.nats.publish("expiry.late", b"hello")
    await nova.nats.flush(timeout=2)
    await asyncio.sleep(0.1)
    assert received == [b"hello"]


async def test_reconnected_signal_fires_once_per_completed_reconnect(nova):
    signal = nova.nats.reconnected
    before = signal.generation
    waiter = asyncio.create_task(signal.wait())

    await expire_login(nova.nats)
    async with asyncio.timeout(5):
        await waiter
    assert signal.generation == before + 1
    assert nova.nats.is_connected, "the signal fires after the connection is usable again"


async def test_login_can_expire_more_than_once(nova):
    for _ in range(3):
        await expire_login(nova.nats)
        await wait_connected(nova.nats)
    assert not nova.nats.is_closed
    assert len(nova.nats._server_pool) == 1, "reconnecting must not grow the server pool"


async def test_other_server_errors_still_close_the_client(nova):
    await server_sends(nova.nats, AUTHORIZATION_VIOLATION)
    await asyncio.sleep(0.1)  # nats-py closes in a task of its own

    assert nova.nats.is_closed
    assert not nova.is_connected()
