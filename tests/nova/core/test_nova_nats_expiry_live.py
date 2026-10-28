"""Live-broker tests for ``Nova`` surviving an expired NATS login (marked ``nats``).

NOVA's auth callout issues user JWTs with a lifetime. When one runs out the server sends
``-ERR 'User Authentication Expired'`` and drops the socket. Stock nats-py treats that
``-ERR`` as fatal and closes the client for good; ``Nova`` must instead reconnect like the
other NATS clients in a cell do, without losing subscriptions or failing calls made while
the connection is down.

The server's part is replayed through nats-py's own protocol parser against a throwaway
``nats-server`` on a free port, so everything from the ``-ERR`` onwards (error handling,
reconnect, re-subscribe, flushing buffered publishes) is the real client code. The module
is skipped when the binary is not installed.
"""

import asyncio
import contextlib
import shutil
import socket
import subprocess
import time

import pytest

from nova.config import NovaConfig
from nova.core.nova import Nova

pytestmark = pytest.mark.nats

NATS_SERVER = shutil.which("nats-server")

AUTH_EXPIRED = b"-ERR 'User Authentication Expired'\r\n"
AUTHORIZATION_VIOLATION = b"-ERR 'Authorization Violation'\r\n"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def nats_url():
    if NATS_SERVER is None:
        pytest.skip("nats-server binary not installed")
    port = _free_port()
    proc = subprocess.Popen(
        [NATS_SERVER, "-a", "127.0.0.1", "-p", str(port)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 10
        while True:
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                    break
            except OSError:
                if time.monotonic() > deadline or proc.poll() is not None:
                    pytest.fail("nats-server did not start")
                time.sleep(0.05)
        yield f"nats://127.0.0.1:{port}"
    finally:
        proc.terminate()
        proc.wait(timeout=5)


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


async def server_sends(nova: Nova, line: bytes) -> None:
    """Deliver ``line`` to the client as if the server had sent it.

    The bytes are handed to nats-py's read loop as the result of its next socket read, so
    they take the same path as traffic from the wire: parser, error handling, and the
    read loop's own exit once a reconnect starts. (Feeding the parser directly from the
    test would leave the read loop running, and it would then report the reconnect's own
    socket close as a second failure.)
    """
    nc = nova.nats
    transport = nc._transport
    assert transport is not None
    real_read = transport.read

    async def read_once(buffer_size: int) -> bytes:
        transport.read = real_read  # ty: ignore[invalid-assignment]
        return line

    reconnects_before = nc.stats["reconnects"]
    transport.read = read_once  # ty: ignore[invalid-assignment]
    # A message to ourselves wakes the read loop; its next read then returns ``line``.
    wake = await nc.subscribe("expiry.wake")
    await nc.publish("expiry.wake", b"")
    for _ in range(2000):
        if nc.is_closed or nc.is_reconnecting or nc.stats["reconnects"] > reconnects_before:
            break
        await asyncio.sleep(0)
    else:
        pytest.fail("the client did not react to the server error")
    with contextlib.suppress(Exception):
        await wake.unsubscribe()


async def expire_login(nova: Nova) -> None:
    """What the server does when the connection's user JWT runs out.

    It then drops the socket as well; the client's reconnect closes it from this side
    first, so that part needs no replay.
    """
    await server_sends(nova, AUTH_EXPIRED)


async def wait_connected(nova: Nova, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not nova.is_connected():
        if time.monotonic() > deadline:
            pytest.fail("NATS did not reconnect")
        await asyncio.sleep(0.02)


async def test_expired_login_reconnects_and_keeps_subscriptions(nova):
    received: list[bytes] = []

    async def on_message(msg) -> None:
        received.append(msg.data)

    await nova.nats.subscribe("expiry.probe", cb=on_message)

    await expire_login(nova)

    assert not nova.nats.is_closed, "the expiry must not close the client for good"
    assert "authentication expired" in str(nova.nats.last_error).lower()

    await wait_connected(nova)

    await nova.nats.publish("expiry.probe", b"after")
    await nova.nats.flush(timeout=2)
    await asyncio.sleep(0.1)
    assert received == [b"after"], "the pre-expiry subscription must survive the re-login"


async def test_calls_made_while_reconnecting_go_through(nova):
    async def respond(msg) -> None:
        await msg.respond(b"pong")

    await nova.nats.subscribe("expiry.echo", cb=respond)

    await expire_login(nova)

    # Neither raises: nats-py buffers both until the connection is back.
    await nova.nats.publish("expiry.noise", b"during gap")
    reply = await nova.nats.request("expiry.echo", b"ping", timeout=5)

    assert reply.data == b"pong"
    assert nova.is_connected()


async def test_login_can_expire_more_than_once(nova):
    for _ in range(3):
        await expire_login(nova)
        await wait_connected(nova)
    assert not nova.nats.is_closed
    assert len(nova.nats._server_pool) == 1, "reconnecting must not grow the server pool"


async def test_other_server_errors_still_close_the_client(nova):
    await server_sends(nova, AUTHORIZATION_VIOLATION)
    await asyncio.sleep(0.1)  # nats-py closes in a task of its own

    assert nova.nats.is_closed
