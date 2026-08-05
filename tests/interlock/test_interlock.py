"""Tests for the NATS-backed robot↔robot interlock.

The unit tests run against an in-memory fake KV that reproduces the JetStream
semantics we rely on: ``create`` fails if the key exists, ``get`` raises
``KeyNotFoundError`` if it does not.  Tests marked ``nats`` need a live broker
with JetStream enabled (``nats-server -js``).
"""

import asyncio

import pytest
from nats.js.errors import KeyNotFoundError, KeyWrongLastSequenceError

from nova.interlock import Grant, InterlockClient, InterlockTimeout, LockId, LockRecord

GEO40 = {
    "r01_r02": LockId.of("ir340r01", "ir340r02", 9),
    "r01_r03": LockId.of("ir340r01", "ir340r03", 11),
    "r02_r03": LockId.of("ir340r02", "ir340r03", 1),
}


# --------------------------------------------------------------------- fakes


class _Entry:
    def __init__(self, value: bytes, revision: int):
        self.value = value
        self.revision = revision


class FakeKV:
    """Minimal JetStream KV stand-in with the atomicity guarantees we depend on."""

    def __init__(self):
        self._data: dict[str, _Entry] = {}
        self._rev = 0
        self.create_calls = 0

    async def get(self, key: str) -> _Entry:
        if key not in self._data:
            raise KeyNotFoundError()
        return self._data[key]

    async def create(self, key: str, value: bytes, **_) -> int:
        self.create_calls += 1
        if key in self._data:
            raise KeyWrongLastSequenceError()
        self._rev += 1
        self._data[key] = _Entry(value, self._rev)
        return self._rev

    async def delete(self, key: str, last: int | None = None, **_) -> bool:
        return self._data.pop(key, None) is not None

    async def keys(self) -> list[str]:
        if not self._data:
            raise Exception("no keys")
        return list(self._data)


def make_client(kv: FakeKV, robot: str) -> InterlockClient:
    client = InterlockClient.__new__(InterlockClient)
    client._nats = None
    client._cell = "testcell"
    client._robot = robot
    client._run_id = f"run-{robot}"
    client._bucket_name = "nova_cells_testcell_interlocks"
    client._create_bucket = False
    client._kv = kv
    client._kv_lock = asyncio.Lock()
    client._held = {}
    return client


# ---------------------------------------------------------------- lock identity


def test_lock_id_is_symmetric():
    """Both partners must derive the same key without coordinating."""
    for a, b, slot in [("ir340r01", "ir340r02", 9), ("ir340r01", "ir340r03", 11)]:
        assert LockId.of(a, b, slot).key == LockId.of(b, a, slot).key


def test_lock_id_distinguishes_slots():
    """Two robots can share several zones; each is its own lock."""
    assert LockId.of("ir360r01", "ir360r02", 1).key != LockId.of("ir360r01", "ir360r02", 2).key


def test_lock_id_rejects_self_interlock():
    with pytest.raises(ValueError):
        LockId.of("ir340r01", "ir340r01", 9)


def test_lock_id_rejects_out_of_range_slot():
    for slot in (0, 17):
        with pytest.raises(ValueError):
            LockId.of("ir340r01", "ir340r02", slot)


def test_geo40_keys_are_distinct():
    assert len({lock.key for lock in GEO40.values()}) == 3


# -------------------------------------------------------------- acquire/release


async def test_acquire_and_release_roundtrip():
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")

    grant = await r01.acquire([GEO40["r01_r02"], GEO40["r01_r03"]], label="GEO40")
    assert set(grant.keys) == {GEO40["r01_r02"].key, GEO40["r01_r03"].key}
    assert len(r01.held) == 2

    await r01.release(grant)
    assert r01.held == []
    assert kv._data == {}, "released locks must leave the bucket empty"


async def test_second_robot_is_blocked_then_proceeds():
    """The core mutual-exclusion property."""
    kv = FakeKV()
    r01, r02 = make_client(kv, "ir340r01"), make_client(kv, "ir340r02")

    grant = await r01.acquire([GEO40["r01_r02"]])

    waiter = asyncio.create_task(r02.acquire([GEO40["r01_r02"]], timeout=5, poll_interval=0.01))
    await asyncio.sleep(0.1)
    assert not waiter.done(), "r02 must not enter while r01 holds the zone"

    await r01.release(grant)
    second = await asyncio.wait_for(waiter, timeout=3)
    assert second.holder == "ir340r02"


async def test_acquire_is_all_or_nothing():
    """A partial set must never be held while waiting — this is what prevents deadlock."""
    kv = FakeKV()
    r01, r02, r03 = (make_client(kv, r) for r in ("ir340r01", "ir340r02", "ir340r03"))

    # r03 holds the r02<->r03 zone
    held_by_r03 = await r03.acquire([GEO40["r02_r03"]])

    # r02 wants {r02_r03, r01_r02}; it must end up holding NEITHER while blocked
    waiter = asyncio.create_task(
        r02.acquire([GEO40["r02_r03"], GEO40["r01_r02"]], timeout=5, poll_interval=0.01)
    )
    await asyncio.sleep(0.15)
    assert not waiter.done()
    assert r02.held == [], "r02 must not hold a partial set"

    # so r01 can still take the zone r02 would otherwise have been squatting on
    grant_r01 = await asyncio.wait_for(r01.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)
    await r01.release(grant_r01)

    await r03.release(held_by_r03)
    await asyncio.wait_for(waiter, timeout=3)


async def test_geo40_triangle_does_not_deadlock():
    """All three GEO40 robots contending concurrently, repeatedly.

    Each robot takes its full slot set in one acquire, exactly as the KRL does
    with a single ``SPSMAKRO20``.  With no partial holds there is no hold-and-wait,
    so the triangle cannot deadlock.
    """
    kv = FakeKV()
    plans = {
        "ir340r01": [GEO40["r01_r02"], GEO40["r01_r03"]],
        "ir340r02": [GEO40["r01_r02"], GEO40["r02_r03"]],
        "ir340r03": [GEO40["r01_r03"], GEO40["r02_r03"]],
    }
    occupancy: dict[str, str] = {}
    violations: list[str] = []

    async def cycle(robot: str, rounds: int = 12):
        client = make_client(kv, robot)
        for _ in range(rounds):
            grant = await client.acquire(plans[robot], timeout=20, poll_interval=0.005)
            for key in grant.keys:
                if key in occupancy:
                    violations.append(f"{key}: {occupancy[key]} and {robot}")
                occupancy[key] = robot
            await asyncio.sleep(0.002)
            for key in grant.keys:
                occupancy.pop(key, None)
            await client.release(grant)

    await asyncio.wait_for(
        asyncio.gather(*(cycle(r) for r in plans)), timeout=60
    )
    assert violations == []


async def test_partial_slot_release():
    """The KRL releases some slots before others (ir360r01 drops slot 7 early)."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    grant = await r01.acquire([GEO40["r01_r02"], GEO40["r01_r03"]])

    await r01.release(grant, slots=[9])  # r01's own slot number for the r02 zone
    assert r01.held == [GEO40["r01_r03"].key]

    r02 = make_client(kv, "ir340r02")
    freed = await asyncio.wait_for(r02.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)
    assert freed.holder == "ir340r02"


async def test_release_all_clears_everything():
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    await r01.acquire([GEO40["r01_r02"], GEO40["r01_r03"]])
    await r01.release_all()
    assert r01.held == []


async def test_timeout_leaves_nothing_held():
    """On timeout the caller holds nothing and must not proceed."""
    kv = FakeKV()
    r01, r02 = make_client(kv, "ir340r01"), make_client(kv, "ir340r02")
    await r01.acquire([GEO40["r01_r02"]])

    with pytest.raises(InterlockTimeout) as excinfo:
        await r02.acquire([GEO40["r01_r02"]], timeout=0.3, poll_interval=0.01)

    assert r02.held == []
    assert "ir340r01" in str(excinfo.value)


async def test_foreign_release_is_refused():
    """A stale grant must not clobber a lock someone else now holds."""
    kv = FakeKV()
    r01, r02 = make_client(kv, "ir340r01"), make_client(kv, "ir340r02")
    grant_r01 = await r01.acquire([GEO40["r01_r02"]])
    await r01.release(grant_r01)
    grant_r02 = await r02.acquire([GEO40["r01_r02"]])

    await r01.release(grant_r01)  # stale — must be a no-op
    still = await kv.get(GEO40["r01_r02"].key)
    assert LockRecord.model_validate_json(still.value).holder == "ir340r02"
    await r02.release(grant_r02)


async def test_crashed_holder_keeps_the_lock():
    """Deliberate: a stuck lock stops a robot; an auto-expiring one is a collision."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    await r01.acquire([GEO40["r01_r02"]])
    del r01  # process dies without releasing

    r02 = make_client(kv, "ir340r02")
    with pytest.raises(InterlockTimeout):
        await r02.acquire([GEO40["r01_r02"]], timeout=0.3, poll_interval=0.01)


async def test_force_release_recovers_a_stale_lock():
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    await r01.acquire([GEO40["r01_r02"]])

    admin = make_client(kv, "operator")
    assert await admin.force_release(GEO40["r01_r02"].key, reason="robot confirmed clear")

    r02 = make_client(kv, "ir340r02")
    grant = await asyncio.wait_for(r02.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)
    assert grant.holder == "ir340r02"


async def test_inspect_reports_holders():
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    await r01.acquire([GEO40["r01_r02"]], label="GEO40 entnehmen")
    state = await r01.inspect()
    record = state[GEO40["r01_r02"].key]
    assert record.holder == "ir340r01"
    assert record.slot == 9
    assert record.label == "GEO40 entnehmen"


async def test_hold_context_manager_releases_on_exception():
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    with pytest.raises(RuntimeError):
        async with r01.hold([GEO40["r01_r02"]]):
            raise RuntimeError("folge failed")
    assert r01.held == []

    r02 = make_client(kv, "ir340r02")
    grant = await asyncio.wait_for(r02.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)
    assert grant.holder == "ir340r02"


async def test_empty_acquire_is_a_noop():
    kv = FakeKV()
    grant = await make_client(kv, "ir340r01").acquire([])
    assert grant.revisions == {}


async def test_grant_is_immutable():
    grant = Grant(holder="ir340r01", run_id="x", revisions={"k": 1})
    with pytest.raises(Exception):
        grant.holder = "ir340r02"
