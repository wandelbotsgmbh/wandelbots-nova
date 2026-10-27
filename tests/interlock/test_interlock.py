"""Tests for the NATS-backed robot↔robot interlock.

The unit tests run against an in-memory fake KV that reproduces the JetStream
semantics we rely on: ``create`` fails if the key exists, ``get`` raises
``KeyNotFoundError`` if it does not, ``delete(last=...)`` is revision-guarded,
and ``watch`` delivers DEL notifications.  The errors are the ones nats-py
raises against a real server: ``create`` on an existing key gives
``KeyWrongLastSequenceError``, but ``delete(last=...)`` on a changed key gives
the untranslated ``BadRequestError`` (err_code 10071).  ``test_interlock_live.py``
runs the same protocol against a real broker (marked ``nats``; needs a local
``nats-server`` binary).
"""

import asyncio
from collections.abc import Callable

import pytest
from nats.js.errors import BadRequestError, KeyNotFoundError, KeyWrongLastSequenceError

from nova.interlock import (
    CORRUPT_HOLDER,
    AlreadyHeldError,
    ForeignRelease,
    Grant,
    InterlockClient,
    InterlockTimeout,
    LockId,
    LockRecord,
)

GEO40 = {
    "r01_r02": LockId.of("ir340r01", "ir340r02", 9),
    "r01_r03": LockId.of("ir340r01", "ir340r03", 11),
    "r02_r03": LockId.of("ir340r02", "ir340r03", 1),
}


# --------------------------------------------------------------------- fakes


class _Entry:
    def __init__(self, value: bytes | None, revision: int, operation: str | None = None):
        self.value = value
        self.revision = revision
        self.operation = operation


class _FakeWatcher:
    def __init__(self, initial: list[_Entry]):
        self._q: asyncio.Queue[_Entry | None] = asyncio.Queue()
        for entry in initial:
            self._q.put_nowait(entry)
        self._q.put_nowait(None)  # end-of-initial-replay marker, as nats-py sends

    def push(self, entry: _Entry) -> None:
        self._q.put_nowait(entry)

    async def updates(self, timeout: float = 5.0) -> _Entry | None:
        return await asyncio.wait_for(self._q.get(), timeout)

    async def stop(self) -> None:
        pass


class FakeKV:
    """Minimal JetStream KV stand-in with the atomicity guarantees we depend on."""

    def __init__(self):
        self._data: dict[str, _Entry] = {}
        self._rev = 0
        self.create_calls = 0
        self._watchers: dict[str, list[_FakeWatcher]] = {}
        # Fault injection: raise on create of a key, block a create until an
        # event is set, raise on every delete, or run a hook just before a delete.
        self.fail_create: dict[str, BaseException] = {}
        self.block_create: dict[str, asyncio.Event] = {}
        self.fail_delete: BaseException | None = None
        self.before_delete: Callable[[str], None] | None = None

    def _notify(self, key: str, entry: _Entry) -> None:
        for watcher in self._watchers.get(key, []):
            watcher.push(entry)

    def put(self, key: str, value: bytes) -> int:
        """Overwrite an entry directly — another client acting between our calls."""
        self._rev += 1
        self._data[key] = _Entry(value, self._rev)
        self._notify(key, self._data[key])
        return self._rev

    async def get(self, key: str) -> _Entry:
        if key not in self._data:
            raise KeyNotFoundError()
        return self._data[key]

    async def create(self, key: str, value: bytes, **_) -> int:
        self.create_calls += 1
        await asyncio.sleep(0)  # a real create is a network round trip: let others run
        if key in self.block_create:
            await self.block_create[key].wait()
        if key in self.fail_create:
            raise self.fail_create[key]
        if key in self._data:
            raise KeyWrongLastSequenceError()
        return self.put(key, value)

    async def delete(self, key: str, last: int | None = None, **_) -> bool:
        if self.fail_delete is not None:
            raise self.fail_delete
        if self.before_delete is not None:
            self.before_delete(key)
        entry = self._data.get(key)
        if last is not None and (entry is None or entry.revision != last):
            # What nats-py raises: KeyValue.delete does not translate 10071.
            raise BadRequestError(code=400, err_code=10071, description="wrong last sequence")
        if entry is None:
            return False
        del self._data[key]
        self._rev += 1
        self._notify(key, _Entry(None, self._rev, operation="DEL"))
        return True

    async def keys(self) -> list[str]:
        if not self._data:
            raise Exception("no keys")
        return list(self._data)

    async def watch(self, key: str, **_) -> _FakeWatcher:
        initial = [self._data[key]] if key in self._data else []
        watcher = _FakeWatcher(initial)
        self._watchers.setdefault(key, []).append(watcher)
        return watcher


def make_client(kv: FakeKV, robot: str) -> InterlockClient:
    client = InterlockClient(None, cell="testcell", robot=robot, create_bucket=False)
    client._run_id = f"run-{robot}"
    client._kv = kv  # ty: ignore[invalid-assignment]
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


def test_lock_id_key_does_not_depend_on_how_each_side_spells_the_ids():
    """One side may use the raw NOVA id ("0@a"), the other the cleaned id it read
    from a key ("0-a"). The canonical order must follow the cleaned ids, or the two
    partners derive different keys, both get a grant and neither waits."""
    assert LockId.of("0@a", "0-b", 1).key == LockId.of("0@b", "0-a", 1).key


def test_lock_id_rejects_a_pair_that_cleans_to_the_same_robot():
    with pytest.raises(ValueError):
        LockId.of("0@a", "0-a", 1)


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

    await asyncio.wait_for(asyncio.gather(*(cycle(r) for r in plans)), timeout=60)
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


async def _partial_release_then_partner_takes_it():
    """r01 holds both zones, drops the r02 zone early, and r02 moves in."""
    kv = FakeKV()
    r01, r02 = make_client(kv, "ir340r01"), make_client(kv, "ir340r02")
    grant = await r01.acquire([GEO40["r01_r02"], GEO40["r01_r03"]])
    await r01.release(grant, slots=[9])
    await asyncio.wait_for(r02.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)
    return kv, r01, grant


async def test_second_partial_release_skips_slot_the_partner_took():
    """Audi 360 pair, 2026-09-10: the second partial release out of the same grant
    re-read the first one's key, now legitimately the partner's, as foreign."""
    kv, r01, grant = await _partial_release_then_partner_takes_it()

    await r01.release(grant, slots=[11])

    assert r01.held == []
    still = await kv.get(GEO40["r01_r02"].key)
    assert LockRecord.model_validate_json(still.value).holder == "ir340r02"


async def test_full_release_after_partial_skips_slot_the_partner_took():
    kv, r01, grant = await _partial_release_then_partner_takes_it()

    await r01.release(grant)

    assert r01.held == []
    still = await kv.get(GEO40["r01_r02"].key)
    assert LockRecord.model_validate_json(still.value).holder == "ir340r02"


async def test_stale_grant_does_not_release_a_newer_grant():
    """Fencing: the same key re-acquired by the same client is a different lock."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    old = await r01.acquire([GEO40["r01_r02"]])
    await r01.release(old)
    await r01.acquire([GEO40["r01_r02"]])

    with pytest.raises(ForeignRelease):
        await r01.release(old)
    assert r01.held == [GEO40["r01_r02"].key]
    assert await kv.get(GEO40["r01_r02"].key)


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
    """A stale grant must not clobber a lock someone else now holds — and the
    divergence must be loud, not a silent no-op."""
    kv = FakeKV()
    r01, r02 = make_client(kv, "ir340r01"), make_client(kv, "ir340r02")
    grant_r01 = await r01.acquire([GEO40["r01_r02"]])
    await r01.release(grant_r01)
    grant_r02 = await r02.acquire([GEO40["r01_r02"]])

    with pytest.raises(ForeignRelease):
        await r01.release(grant_r01)  # stale — must not touch r02's lock
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


async def test_hold_keeps_locks_on_exception():
    """A failed Folge may have stopped the robot *inside* the zone — the lock
    must survive the error (fail-to-wait), never be released by control flow."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    with pytest.raises(RuntimeError):
        async with r01.hold([GEO40["r01_r02"]]):
            raise RuntimeError("folge failed")

    assert r01.held == [GEO40["r01_r02"].key], "abnormal exit must keep the lock"
    r02 = make_client(kv, "ir340r02")
    with pytest.raises(InterlockTimeout):
        await r02.acquire([GEO40["r01_r02"]], timeout=0.3, poll_interval=0.01)

    # recovery: once the robot is confirmed clear, release_all frees the partner
    released = await r01.release_all()
    assert released == [GEO40["r01_r02"].key]
    grant = await asyncio.wait_for(r02.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)
    assert grant.holder == "ir340r02"


async def test_hold_keeps_locks_on_cancellation():
    """Cancellation decelerates the robot past the cancellation point — it may
    rest inside the zone, so a cancelled hold must keep the lock."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    entered = asyncio.Event()

    async def folge():
        async with r01.hold([GEO40["r01_r02"]]):
            entered.set()
            await asyncio.sleep(60)  # "motion" that gets cancelled

    task = asyncio.create_task(folge())
    await asyncio.wait_for(entered.wait(), timeout=2)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task

    assert r01.held == [GEO40["r01_r02"].key]
    assert GEO40["r01_r02"].key in kv._data, "the KV entry must survive cancellation"


async def test_hold_releases_on_clean_exit():
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    async with r01.hold([GEO40["r01_r02"]]):
        assert r01.held == [GEO40["r01_r02"].key]
    assert r01.held == []
    assert kv._data == {}


async def test_acquire_while_holding_is_refused():
    """Hold-and-wait is the deadlock ingredient; the single-acquire rule is
    enforced, not documented."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    grant = await r01.acquire([GEO40["r01_r02"], GEO40["r01_r03"]])

    with pytest.raises(AlreadyHeldError):
        await r01.acquire([GEO40["r02_r03"]])
    # also for an overlapping set — re-acquiring a held key must not alias grants
    with pytest.raises(AlreadyHeldError):
        await r01.acquire([GEO40["r01_r02"]])

    # a partial release still leaves the client "holding"
    await r01.release(grant, slots=[9])
    with pytest.raises(AlreadyHeldError):
        await r01.acquire([GEO40["r01_r02"]])

    await r01.release(grant)
    assert r01.held == []
    await r01.acquire([GEO40["r01_r02"]])  # free to acquire again


async def test_release_all_recovers_locks_from_a_previous_run():
    """A restarted process gets a fresh run_id; once the program asserts the
    robot is physically clear it can reclaim its predecessor's locks."""
    kv = FakeKV()
    crashed = make_client(kv, "ir340r01")
    crashed._run_id = "run-before-crash"
    await crashed.acquire([GEO40["r01_r02"], GEO40["r01_r03"]])
    del crashed

    restarted = make_client(kv, "ir340r01")  # fresh run_id, empty _held
    assert await restarted.release_all() == [], "default must not touch a foreign run"
    assert len(kv._data) == 2

    released = await restarted.release_all(include_previous_runs=True)
    assert sorted(released) == sorted([GEO40["r01_r02"].key, GEO40["r01_r03"].key])
    assert kv._data == {}


async def test_release_all_previous_runs_spares_other_robots():
    kv = FakeKV()
    r02 = make_client(kv, "ir340r02")
    await r02.acquire([GEO40["r02_r03"]])

    r01 = make_client(kv, "ir340r01")
    await r01.release_all(include_previous_runs=True)
    assert GEO40["r02_r03"].key in kv._data, "another robot's lock must be untouched"


async def test_corrupt_record_blocks_and_is_visible():
    """An unparseable KV entry must behave like a held lock and show up in
    inspect() so an operator can find and force_release it."""
    kv = FakeKV()
    key = GEO40["r01_r02"].key
    kv._rev += 1
    kv._data[key] = _Entry(b"not json", kv._rev)

    r02 = make_client(kv, "ir340r02")
    with pytest.raises(InterlockTimeout):
        await r02.acquire([GEO40["r01_r02"]], timeout=0.3, poll_interval=0.01)

    state = await r02.inspect()
    assert state[key].holder == CORRUPT_HOLDER

    assert await r02.force_release(key, reason="confirmed clear")
    grant = await asyncio.wait_for(r02.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)
    assert grant.holder == "ir340r02"


async def test_empty_acquire_is_a_noop():
    kv = FakeKV()
    grant = await make_client(kv, "ir340r01").acquire([])
    assert grant.revisions == {}


async def test_grant_is_immutable():
    grant = Grant(holder="ir340r01", run_id="x", revisions={"k": 1})
    with pytest.raises(Exception):
        grant.holder = "ir340r02"  # ty: ignore[invalid-assignment]


# --------------------------------------------------- failure paths (PR #544 review)


async def test_acquire_rolls_back_keys_created_before_a_transport_error():
    """A NATS error partway through a multi-key acquire must not leave the keys
    created so far in the bucket: nothing tracks them, so the partner would wait
    forever and release_all() could not find them."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    locks = [GEO40["r01_r02"], GEO40["r01_r03"]]
    kv.fail_create[max(lock.key for lock in locks)] = ConnectionError("nats: connection closed")

    with pytest.raises(ConnectionError):
        await r01.acquire(locks)

    assert kv._data == {}, "the key created before the error must be rolled back"
    assert r01.held == []


async def test_acquire_rolls_back_keys_created_before_a_cancellation():
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    locks = [GEO40["r01_r02"], GEO40["r01_r03"]]
    second = max(lock.key for lock in locks)
    kv.block_create[second] = asyncio.Event()  # never set: the create hangs

    task = asyncio.create_task(r01.acquire(locks))
    while kv.create_calls < 2:
        await asyncio.sleep(0)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task

    assert kv._data == {}, "a cancelled acquire must roll back what it created"
    assert r01.held == []


async def test_acquire_keeps_tracking_keys_it_could_not_roll_back():
    """If the rollback itself fails, the leftover keys stay in ``held`` so
    release_all() can still free them — never silently orphaned."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    locks = [GEO40["r01_r02"], GEO40["r01_r03"]]
    first, second = sorted(lock.key for lock in locks)
    kv.fail_create[second] = ConnectionError("nats: connection closed")
    kv.fail_delete = ConnectionError("nats: connection closed")

    with pytest.raises(ConnectionError):
        await r01.acquire(locks)

    assert first in kv._data
    assert r01.held == [first]
    kv.fail_delete = None
    assert await r01.release_all() == [first]
    assert kv._data == {}


async def test_hold_after_releasing_everything_early_exits_cleanly():
    """A Folge that frees the zone as soon as it has retreated, then keeps moving
    inside the block: the partner moves in meanwhile, and the clean exit of the
    block must not turn that into ForeignRelease."""
    kv = FakeKV()
    r01, r02 = make_client(kv, "ir340r01"), make_client(kv, "ir340r02")
    async with r01.hold([GEO40["r01_r02"]]) as grant:
        await r01.release(grant)
        await asyncio.wait_for(r02.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)

    record = LockRecord.model_validate_json((await kv.get(GEO40["r01_r02"].key)).value)
    assert record.holder == "ir340r02", "the partner's lock must be untouched"
    assert r01.held == []


async def test_release_all_reports_only_what_it_actually_deleted():
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    await r01.acquire([GEO40["r01_r02"]])
    kv.fail_delete = ConnectionError("nats: connection closed")

    assert await r01.release_all() == []
    assert r01.held == [GEO40["r01_r02"].key], "a lock still in the bucket stays tracked"
    assert GEO40["r01_r02"].key in kv._data


async def test_release_reports_a_lock_that_changes_hands_mid_release_as_foreign():
    """Operator force_release plus partner acquire between our read and our delete:
    the revision-guarded delete fails with nats-py's untranslated BadRequestError
    (err_code 10071), which must become ForeignRelease, not escape as-is."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    grant = await r01.acquire([GEO40["r01_r02"]])
    partner = LockRecord(holder="ir340r02", run_id="run-ir340r02", slot=9)

    def hand_over(key: str) -> None:
        kv.before_delete = None
        kv.put(key, partner.model_dump_json().encode())

    kv.before_delete = hand_over
    with pytest.raises(ForeignRelease):
        await r01.release(grant)
    record = LockRecord.model_validate_json((await kv.get(GEO40["r01_r02"].key)).value)
    assert record.holder == "ir340r02"


async def test_force_release_returns_false_when_the_lock_changes_hands():
    kv = FakeKV()
    await make_client(kv, "ir340r01").acquire([GEO40["r01_r02"]])
    partner = LockRecord(holder="ir340r02", run_id="run-ir340r02", slot=9)

    def hand_over(key: str) -> None:
        kv.before_delete = None
        kv.put(key, partner.model_dump_json().encode())

    kv.before_delete = hand_over
    admin = make_client(kv, "operator")
    assert await admin.force_release(GEO40["r01_r02"].key, reason="test") is False
    assert GEO40["r01_r02"].key in kv._data


async def test_concurrent_acquires_on_one_client_grant_once():
    """Two coroutines sharing a client must not both get a grant for the same
    lock: the second sees the first's hold and is refused."""
    kv = FakeKV()
    r01 = make_client(kv, "ir340r01")
    results = await asyncio.gather(
        r01.acquire([GEO40["r01_r02"]], timeout=0.3, poll_interval=0.01),
        r01.acquire([GEO40["r01_r02"]], timeout=0.3, poll_interval=0.01),
        return_exceptions=True,
    )
    grants = [r for r in results if isinstance(r, Grant)]
    refused = [r for r in results if isinstance(r, AlreadyHeldError)]
    assert len(grants) == 1 and len(refused) == 1, results


async def test_cancelling_a_waiting_acquire_leaves_nothing_behind():
    """A program cancels an acquire that is waiting for the partner: the keys it
    took and rolled back must be gone, nothing phantom may stay in ``held``, and
    the next acquire must work."""
    kv = FakeKV()
    r01, r02 = make_client(kv, "ir340r01"), make_client(kv, "ir340r02")
    partner = await r02.acquire([GEO40["r01_r02"]])

    waiting = asyncio.create_task(
        r01.acquire([GEO40["r01_r02"], GEO40["r01_r03"]], timeout=10, poll_interval=0.01)
    )
    await asyncio.sleep(0.05)
    waiting.cancel()
    with pytest.raises(asyncio.CancelledError):
        await waiting

    assert r01.held == []
    assert set(kv._data) == {GEO40["r01_r02"].key}, "only the partner's lock may remain"
    await r02.release(partner)
    grant = await asyncio.wait_for(r01.acquire([GEO40["r01_r02"]], timeout=2), timeout=3)
    assert grant.holder == "ir340r01"
