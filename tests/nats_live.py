"""Helpers for tests against a throwaway ``nats-server`` (tests marked ``nats``).

``nats_server_url`` starts one on a free port, and ``server_sends`` lets a test replay a
protocol line from the server, such as the ``-ERR`` that ends an expired login, through
the real nats-py read loop.
"""

from __future__ import annotations

import asyncio
import contextlib
import shutil
import socket
import subprocess
import time
from collections.abc import Iterator

import pytest

NATS_SERVER = shutil.which("nats-server")

AUTH_EXPIRED = b"-ERR 'User Authentication Expired'\r\n"
AUTHORIZATION_VIOLATION = b"-ERR 'Authorization Violation'\r\n"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@contextlib.contextmanager
def nats_server_url(*args: str) -> Iterator[str]:
    """Run ``nats-server`` on a free port for the duration; skip if it is not installed."""
    if NATS_SERVER is None:
        pytest.skip("nats-server binary not installed")
    port = _free_port()
    proc = subprocess.Popen(
        [NATS_SERVER, "-a", "127.0.0.1", "-p", str(port), *args],
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


async def server_sends(nc, line: bytes) -> None:
    """Deliver ``line`` to the nats-py client ``nc`` as if the server had sent it.

    The bytes are handed to nats-py's read loop as the result of its next socket read, so
    they take the same path as traffic from the wire: parser, error handling, and the
    read loop's own exit once a reconnect starts. (Feeding the parser directly from the
    test would leave the read loop running, and it would then report the reconnect's own
    socket close as a second failure.) Returns once the client has reacted.
    """
    transport = nc._transport
    assert transport is not None
    real_read = transport.read

    async def read_once(buffer_size: int) -> bytes:
        transport.read = real_read
        return line

    reconnects_before = nc.stats["reconnects"]
    transport.read = read_once
    # A message to ourselves wakes the read loop; its next read then returns ``line``.
    wake = await nc.subscribe("live.wake")
    await nc.publish("live.wake", b"")
    for _ in range(2000):
        if nc.is_closed or nc.is_reconnecting or nc.stats["reconnects"] > reconnects_before:
            break
        await asyncio.sleep(0)
    else:
        pytest.fail("the client did not react to the server error")
    with contextlib.suppress(Exception):
        await wake.unsubscribe()


async def expire_login(nc) -> None:
    """What the server does when the connection's user JWT runs out.

    It then drops the socket as well; the client's reconnect closes it from this side
    first, so that part needs no replay.
    """
    await server_sends(nc, AUTH_EXPIRED)


async def wait_connected(nc, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not nc.is_connected:
        if time.monotonic() > deadline:
            pytest.fail("NATS did not reconnect")
        await asyncio.sleep(0.02)
