"""Live-broker tests for the interlock.

These verify the JetStream semantics the unit-test FakeKV merely assumes:
atomic create-if-absent, revision-guarded delete (and the error nats-py raises
when it fails), recreate after a delete tombstone, watch-based wake-up, and the
bucket-creation race between concurrently starting robot processes.

Two backends, picked per run:

- **NOVA** — when ``NOVA_API`` is set (the CI ``integration`` job, or a local
  ``.env``), every connection is a ``Nova()`` client's ``nats``: the same NATS a
  robot program reaches through ``ctx.nova.nats``.
- **local** — otherwise a throwaway ``nats-server -js`` on a free port, if the
  binary is installed; without it the module is skipped.

Each test uses a fresh cell name, hence its own KV bucket, deleted afterwards.
"""

import asyncio
import contextlib
import os
import shutil
import socket
import subprocess
import time
import uuid
from collections.abc import AsyncIterator, Awaitable, Callable

import nats
import pytest

from nova import Nova
from nova.interlock import (
    BUCKET_TEMPLATE,
    ForeignRelease,
    InterlockClient,
    InterlockTimeout,
    LockId,
)

pytestmark = [pytest.mark.integration, pytest.mark.nats]

NATS_SERVER = shutil.which("nats-server")

Connect = Callable[[], Awaitable[nats.NATS]]


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def local_nats_url(tmp_path_factory):
    """A throwaway JetStream server, only started when no NOVA instance is configured."""
    if os.environ.get("NOVA_API"):
        yield None
        return
    if NATS_SERVER is None:
        pytest.skip("neither NOVA_API nor a nats-server binary is available")
    port = _free_port()
    store = tmp_path_factory.mktemp("jetstream")
    proc = subprocess.Popen(
        [NATS_SERVER, "-js", "-p", str(port), "-sd", str(store)],
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
async def connect(local_nats_url) -> AsyncIterator[Connect]:
    """Factory for NATS connections — one per simulated robot process."""
    novas: list[Nova] = []
    clients: list[nats.NATS] = []

    async def open_connection() -> nats.NATS:
        if local_nats_url is None:
            nova = Nova()
            await nova.open()
            novas.append(nova)
            return nova.nats
        client = await nats.connect(local_nats_url)
        clients.append(client)
        return client

    yield open_connection
    for client in clients:
        with contextlib.suppress(Exception):
            await client.drain()
    for nova in novas:
        with contextlib.suppress(Exception):
            await nova.close()


@pytest.fixture
async def cell(connect) -> AsyncIterator[str]:
    """A fresh cell (hence a fresh KV bucket) per test; the bucket is removed afterwards."""
    name = f"livetest-{uuid.uuid4().hex[:8]}"
    yield name
    with contextlib.suppress(Exception):
        await (await connect()).jetstream().delete_key_value(BUCKET_TEMPLATE.format(cell=name))


@pytest.fixture
async def nc(connect) -> nats.NATS:
    return await connect()


async def test_roundtrip_against_jetstream(nc, cell):
    r01 = InterlockClient(nc, cell=cell, robot="ir340r01")
    lock = LockId.of("ir340r01", "ir340r02", 9)

    grant = await r01.acquire([lock], label="live roundtrip")
    assert r01.held == [lock.key]
    state = await r01.inspect()
    assert state[lock.key].holder == "ir340r01"

    await r01.release(grant)
    assert await r01.inspect() == {}


async def test_mutual_exclusion_and_watch_wakeup(nc, cell):
    r01 = InterlockClient(nc, cell=cell, robot="ir340r01")
    r02 = InterlockClient(nc, cell=cell, robot="ir340r02")
    lock = LockId.of("ir340r01", "ir340r02", 9)

    grant = await r01.acquire([lock])
    waiter = asyncio.create_task(r02.acquire([lock], timeout=10, poll_interval=0.05))
    await asyncio.sleep(0.3)
    assert not waiter.done(), "r02 must not enter while r01 holds the zone"

    released_at = asyncio.get_running_loop().time()
    await r01.release(grant)
    second = await asyncio.wait_for(waiter, timeout=5)
    handover = asyncio.get_running_loop().time() - released_at
    assert second.holder == "ir340r02"
    assert handover < 2.0, f"hand-over took {handover:.2f}s — watch wake-up seems dead"


async def test_recreate_after_delete_tombstone(nc, cell):
    """JetStream delete leaves a tombstone in the stream; create must keep
    working across repeated hold/release cycles."""
    r01 = InterlockClient(nc, cell=cell, robot="ir340r01")
    lock = LockId.of("ir340r01", "ir340r02", 9)
    for _ in range(5):
        grant = await r01.acquire([lock])
        await r01.release(grant)


async def test_lock_changing_hands_mid_release_is_foreign(nc, cell, monkeypatch):
    """Against a real server a revision-guarded delete on a changed key raises
    nats-py's untranslated BadRequestError (err_code 10071), not
    KeyWrongLastSequenceError; release() must still report ForeignRelease."""
    r01 = InterlockClient(nc, cell=cell, robot="ir340r01")
    r02 = InterlockClient(nc, cell=cell, robot="ir340r02")
    operator = InterlockClient(nc, cell=cell, robot="operator")
    lock = LockId.of("ir340r01", "ir340r02", 9)
    grant = await r01.acquire([lock])

    read = r01._read

    async def read_then_hand_over(kv, key):
        result = await read(kv, key)
        monkeypatch.setattr(r01, "_read", read)
        assert await operator.force_release(key, reason="live test")
        await r02.acquire([lock], timeout=5)
        return result

    monkeypatch.setattr(r01, "_read", read_then_hand_over)
    with pytest.raises(ForeignRelease):
        await r01.release(grant)
    assert (await r01.inspect())[lock.key].holder == "ir340r02"


async def test_bucket_creation_race(connect, cell):
    """The cell starts up to 13 robot processes at once; concurrent first use
    must not crash on 'stream name already in use'."""

    async def first_use(robot: str) -> None:
        locks = InterlockClient(await connect(), cell=cell, robot=robot)
        grant = await locks.acquire([LockId.of(robot, "partner", 1)])
        await locks.release(grant)

    await asyncio.gather(*(first_use(f"rob{i:02d}") for i in range(8)))


async def test_crashed_holder_lock_survives_disconnect(connect, cell):
    """A holder that vanishes without releasing must keep blocking its partner
    (no TTL), and the restarted process recovers via include_previous_runs."""
    lock = LockId.of("ir340r01", "ir340r02", 9)

    nc1 = await connect()
    r01 = InterlockClient(nc1, cell=cell, robot="ir340r01")
    await r01.acquire([lock])
    await nc1.close()  # process dies without releasing

    nc2 = await connect()
    r02 = InterlockClient(nc2, cell=cell, robot="ir340r02")
    with pytest.raises(InterlockTimeout):
        await r02.acquire([lock], timeout=1.0, poll_interval=0.05)

    # recovery: the restarted r01 asserts the robot is physically clear
    r01_restarted = InterlockClient(nc2, cell=cell, robot="ir340r01")
    released = await r01_restarted.release_all(include_previous_runs=True)
    assert released == [lock.key]

    grant = await r02.acquire([lock], timeout=5)
    assert grant.holder == "ir340r02"
