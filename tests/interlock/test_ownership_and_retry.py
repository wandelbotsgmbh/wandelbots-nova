"""Robot id ownership (one live process per robot) and idempotent acquire retries."""

import asyncio

import pytest

from nova.interlock import (
    AlreadyHeldError,
    GrantLost,
    OwnerRecord,
    OwnershipConflict,
    OwnershipLost,
    OwnershipRequired,
)
from tests.interlock.test_interlock import TRIO, FakeKV, FakeNats, make_client, make_owner

SET = [TRIO["r01_r02"], TRIO["r01_r03"]]


# ------------------------------------------------------- idempotent acquire


async def test_retry_with_same_request_id_returns_the_same_grant():
    """A step whose acquire succeeded but which failed afterwards is retried by
    the fault handler; the retry must not get stuck on AlreadyHeldError."""
    kv = FakeKV()
    r01 = make_client(kv, "robot-a")
    grant = await r01.acquire(SET, request_id="P42")
    creates = kv.create_calls

    again = await r01.acquire(SET, request_id="P42")

    assert again == grant
    assert kv.create_calls == creates, "a retry must not touch the bucket's locks"
    await r01.release(again)
    assert kv._data == {}


async def test_retry_needs_the_same_request_id():
    r01 = make_client(kv := FakeKV(), "robot-a")
    await r01.acquire(SET, request_id="P42")
    with pytest.raises(AlreadyHeldError):
        await r01.acquire(SET)
    with pytest.raises(AlreadyHeldError):
        await r01.acquire(SET, request_id="P56")
    assert len(kv._data) == 2


async def test_retry_with_a_different_slot_set_is_refused():
    """Same id, more slots: that would be hold-and-wait, not a retry."""
    r01 = make_client(FakeKV(), "robot-a")
    await r01.acquire([TRIO["r01_r02"]], request_id="P42")
    with pytest.raises(AlreadyHeldError):
        await r01.acquire(SET, request_id="P42")


async def test_retry_after_a_partial_release_is_refused():
    """A partial release means the robot moved on; the request is no longer a retry."""
    r01 = make_client(FakeKV(), "robot-a")
    grant = await r01.acquire(SET, request_id="P42")
    await r01.release(grant, slots=[TRIO["r01_r02"].slot])
    with pytest.raises(AlreadyHeldError):
        await r01.acquire(SET, request_id="P42")


async def test_retry_detects_a_grant_lost_behind_its_back():
    """An operator force_release between the acquire and its retry: the retry
    must not hand out a grant the bucket no longer backs."""
    kv = FakeKV()
    r01 = make_client(kv, "robot-a")
    await r01.acquire(SET, request_id="P42")
    operator = make_client(kv, "operator")
    await operator.force_release(TRIO["r01_r02"].key, reason="test")
    r02 = make_client(kv, "robot-b")
    await r02.acquire([TRIO["r01_r02"]])

    with pytest.raises(GrantLost) as info:
        await r01.acquire(SET, request_id="P42")
    assert info.value.records[TRIO["r01_r02"].key].holder == "robot-b"
    assert kv._data[TRIO["r01_r02"].key] is not None, "the partner's lock is untouched"


async def test_same_request_id_after_release_acquires_afresh():
    kv = FakeKV()
    r01 = make_client(kv, "robot-a")
    first = await r01.acquire(SET, request_id="P42")
    await r01.release(first)
    second = await r01.acquire(SET, request_id="P42")
    assert second.revisions != first.revisions
    assert len(kv._data) == 2


# ---------------------------------------------------------------- ownership


async def test_claim_records_the_session():
    kv, nats = FakeKV(), FakeNats()
    owner = make_owner(kv, nats, "robot-a")
    await owner.claim()
    record = OwnerRecord.model_validate_json(kv._data["owners.robot-a"].value)
    assert record.session == owner.session
    assert owner.claimed


async def test_second_live_process_for_the_same_robot_is_refused():
    """The gap this closes: a dev PC dispatcher next to the cell app."""
    kv, nats = FakeKV(), FakeNats()
    cell_app = make_owner(kv, nats, "robot-a")
    await cell_app.claim()

    dev_pc = make_owner(kv, nats, "robot-a")
    with pytest.raises(OwnershipConflict) as info:
        await dev_pc.claim()
    assert info.value.record.session == cell_app.session
    assert not dev_pc.claimed
    assert f"nova.interlock.testcell.owners.{dev_pc.session}" not in nats.handlers
    await cell_app.verify()  # untouched


async def test_second_process_cannot_free_the_first_ones_locks():
    locks_kv, owners_kv, nats = FakeKV(), FakeKV(), FakeNats()
    cell_app = make_owner(owners_kv, nats, "robot-a")
    await cell_app.claim()
    in_zone = make_client(locks_kv, "robot-a", owner=cell_app)
    await in_zone.acquire(SET)

    dev_pc = make_client(locks_kv, "robot-a")
    dev_pc._run_id = "run-dev-pc"
    with pytest.raises(OwnershipRequired):
        await dev_pc.release_all(include_previous_runs=True)
    dev_owner = make_owner(owners_kv, nats, "robot-a")
    with pytest.raises(OwnershipConflict):
        await dev_owner.claim()
    assert len(locks_kv._data) == 2


async def test_crashed_owner_is_taken_over_and_its_locks_recovered():
    """The routine restart: the old process is gone, nobody answers the ping."""
    locks_kv, owners_kv, nats = FakeKV(), FakeKV(), FakeNats()
    crashed = make_owner(owners_kv, nats, "robot-a")
    await crashed.claim()
    before = make_client(locks_kv, "robot-a", owner=crashed)
    before._run_id = "run-before-crash"
    await before.acquire(SET)
    await crashed._stop_answering()  # the process died without releasing

    restarted = make_owner(owners_kv, nats, "robot-a")
    await restarted.claim()
    locks = restarted.client()
    locks._kv = locks_kv  # ty: ignore[invalid-assignment]
    released = await locks.release_all(include_previous_runs=True)  # robot at home

    assert sorted(released) == sorted(lock.key for lock in SET)
    record = OwnerRecord.model_validate_json(owners_kv._data["owners.robot-a"].value)
    assert record.session == restarted.session


async def test_unresponsive_owner_learns_it_was_taken_over():
    """A live owner that looked dead (frozen loop, NATS gap) must stop acting:
    its next acquire raises and creates nothing."""
    locks_kv, owners_kv, nats = FakeKV(), FakeKV(), FakeNats()
    old = make_owner(owners_kv, nats, "robot-a")
    await old.claim()
    old_locks = make_client(locks_kv, "robot-a", owner=old)
    nats.frozen.add(f"nova.interlock.testcell.owners.{old.session}")

    new = make_owner(owners_kv, nats, "robot-a")
    await new.claim()

    with pytest.raises(OwnershipLost):
        await old_locks.acquire(SET)
    assert locks_kv._data == {}
    assert not old.claimed


async def test_waiting_acquire_stops_when_ownership_is_lost():
    locks_kv, owners_kv, nats = FakeKV(), FakeKV(), FakeNats()
    partner = make_client(locks_kv, "robot-b")
    await partner.acquire([TRIO["r01_r02"]])

    old = make_owner(owners_kv, nats, "robot-a")
    await old.claim()
    waiting = asyncio.create_task(
        make_client(locks_kv, "robot-a", owner=old).acquire(SET, timeout=5, poll_interval=0.01)
    )
    await asyncio.sleep(0.05)
    await old._stop_answering()
    await make_owner(owners_kv, nats, "robot-a").claim()
    partner_grant = partner._grant
    assert partner_grant is not None
    await partner.release(partner_grant)  # wakes the waiter

    with pytest.raises(OwnershipLost):
        await asyncio.wait_for(waiting, 2)
    assert locks_kv._data == {}, "the waiter must not have taken anything"


async def test_racing_claimants_for_a_dead_owner_produce_one_winner():
    owners_kv, nats = FakeKV(), FakeNats()
    dead = make_owner(owners_kv, nats, "robot-a")
    await dead.claim()
    await dead._stop_answering()

    a, b = make_owner(owners_kv, nats, "robot-a"), make_owner(owners_kv, nats, "robot-a")
    results = await asyncio.gather(a.claim(), b.claim(), return_exceptions=True)

    assert sorted(type(r).__name__ for r in results) == ["NoneType", "OwnershipConflict"]
    assert a.claimed != b.claimed


async def test_release_frees_the_robot_id_without_a_liveness_wait():
    owners_kv, nats = FakeKV(), FakeNats()
    async with make_owner(owners_kv, nats, "robot-a"):
        pass
    assert owners_kv._data == {}
    successor = make_owner(owners_kv, nats, "robot-a")
    await successor.claim()
    assert nats.requests == 0


async def test_lock_records_carry_the_owner_session():
    locks_kv, owners_kv, nats = FakeKV(), FakeKV(), FakeNats()
    owner = make_owner(owners_kv, nats, "robot-a")
    await owner.claim()
    await make_client(locks_kv, "robot-a", owner=owner).acquire(SET)
    records = await make_client(locks_kv, "operator").inspect()
    assert {r.owner for r in records.values()} == {owner.session}


async def test_unreadable_owner_record_blocks_the_claim():
    owners_kv, nats = FakeKV(), FakeNats()
    owners_kv.put("owners.robot-a", b"not json")
    with pytest.raises(OwnershipConflict):
        await make_owner(owners_kv, nats, "robot-a").claim()


async def test_owner_must_match_the_client_robot():
    owner = make_owner(FakeKV(), FakeNats(), "robot-a")
    with pytest.raises(ValueError):
        make_client(FakeKV(), "robot-b", owner=owner)


async def test_unclaimed_owner_refuses_to_act():
    owner = make_owner(FakeKV(), FakeNats(), "robot-a")
    with pytest.raises(OwnershipLost):
        await make_client(FakeKV(), "robot-a", owner=owner).acquire(SET)
