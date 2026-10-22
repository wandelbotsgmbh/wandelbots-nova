"""Live-broker tests for the interlock (marked ``nats``).

These verify the JetStream semantics the unit-test FakeKV merely assumes:
atomic create-if-absent, revision-guarded delete, recreate after a delete
tombstone, watch-based wake-up, and the bucket-creation race between
concurrently starting robot processes.  They spin up a throwaway
``nats-server -js`` on a free port; if the binary is not installed the whole
module is skipped.
"""

import asyncio
import shutil
import socket
import subprocess
import time
import uuid

import nats
import pytest

from nova.interlock import InterlockClient, InterlockTimeout, LockId

pytestmark = pytest.mark.nats

NATS_SERVER = shutil.which("nats-server")


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def nats_url(tmp_path_factory):
    if NATS_SERVER is None:
        pytest.skip("nats-server binary not installed")
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
def cell() -> str:
    """A fresh cell (hence a fresh KV bucket) per test."""
    return f"livetest-{uuid.uuid4().hex[:8]}"


@pytest.fixture
async def nc(nats_url):
    client = await nats.connect(nats_url)
    yield client
    await client.drain()


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


async def test_bucket_creation_race(nats_url, cell):
    """The cell starts up to 13 robot processes at once; concurrent first use
    must not crash on 'stream name already in use'."""
    conns: list[nats.NATS] = []

    async def first_use(robot: str) -> None:
        client = await nats.connect(nats_url)
        conns.append(client)
        locks = InterlockClient(client, cell=cell, robot=robot)
        grant = await locks.acquire([LockId.of(robot, "partner", 1)])
        await locks.release(grant)

    try:
        await asyncio.gather(*(first_use(f"rob{i:02d}") for i in range(8)))
    finally:
        for client in conns:
            await client.drain()


async def test_crashed_holder_lock_survives_disconnect(nats_url, cell):
    """A holder that vanishes without releasing must keep blocking its partner
    (no TTL), and the restarted process recovers via include_previous_runs."""
    lock = LockId.of("ir340r01", "ir340r02", 9)

    nc1 = await nats.connect(nats_url)
    r01 = InterlockClient(nc1, cell=cell, robot="ir340r01")
    await r01.acquire([lock])
    await nc1.close()  # process dies without releasing

    nc2 = await nats.connect(nats_url)
    try:
        r02 = InterlockClient(nc2, cell=cell, robot="ir340r02")
        with pytest.raises(InterlockTimeout):
            await r02.acquire([lock], timeout=1.0, poll_interval=0.05)

        # recovery: the restarted r01 asserts the robot is physically clear
        r01_restarted = InterlockClient(nc2, cell=cell, robot="ir340r01")
        released = await r01_restarted.release_all(include_previous_runs=True)
        assert released == [lock.key]

        grant = await r02.acquire([lock], timeout=5)
        assert grant.holder == "ir340r02"
    finally:
        await nc2.drain()
