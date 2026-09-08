"""NATS-backed robot↔robot interlock ("Roboterverriegelung" / MAKRO 20).

Replaces the PLC signal handshake with NATS JetStream KV as the atomic locking
layer.  The mapping to the original mechanism:

===========================  ==================================================
VASS / FB207                 here
===========================  ==================================================
``A81..A96`` request bit      :meth:`InterlockClient.acquire`
``E81..E96`` grant (ack)      the KV ``create`` succeeding
``E41..E56`` partner clear    (not modelled — see "Single-stage" below)
``A41..A56 = EIN`` release    :meth:`InterlockClient.release`
``ST_ROBVER`` shared table    the KV bucket ``nova_cells_{cell}_interlocks``
``MAKRO 20`` 5-phase call     one :meth:`acquire` over the whole slot set
===========================  ==================================================

**Single-stage.**  FB207 arbitrates a *reservation* (``Quitt_Anf``/``E8x``) and
separately broadcasts a *position-derived clearance* (``Frg_an``/``E4x``,
computed from ``Stell_Frg``/``PF0``).  A robot had to wait on both.  Here the KV
entry is the reservation *and* the occupancy: it exists exactly while a robot is
inside, because it is created before the move and deleted after the retreat.
That collapses the handshake into one atomic operation.

    The cost is that a crashed holder leaves the lock held.  That is deliberate:
    a stuck lock stops a robot (safe), whereas a lock that expires while a robot
    is physically still in the zone is a collision (unsafe).  There is therefore
    **no TTL by default**.  Use :meth:`InterlockClient.inspect` to find stale
    locks and :meth:`force_release` to clear one, after confirming the robot is
    physically clear.

    The same fail-to-wait rule governs program failures: :meth:`hold` releases
    only on a **clean** exit of the guarded block.  On an exception or a
    cancellation the robot may have decelerated to a stop *inside* the zone, so
    the locks are kept.  Recovery is :meth:`release_all` once the robot is back
    in a position known to be clear (e.g. at home, the start-of-Folge
    ``A41..A56 = EIN`` situation), or an operator :meth:`force_release`.

**Atomicity.**  A robot typically needs several zones at once (in the GEO40 trio
each robot takes its full set in a single ``MAKRO 20`` call).  :meth:`acquire`
takes them in sorted key order and rolls back completely on any conflict, so a
caller never holds a partial set while waiting.  Rollback is free here because
acquisition happens with the robot stationary, before any motion is planned or
started — nothing has to be undone physically.  With no partial holds there is
no hold-and-wait, hence no deadlock — and *single acquire* is **enforced**, not
assumed: :meth:`acquire` raises :class:`AlreadyHeldError` while anything is
held.  Staged/nested acquisition (the VASS same-partner pattern) is therefore
not supported by this prototype; take the complete set for a step in one call.

.. warning::
    **Experimental.**  This is a prototype replacement for a PLC function.  It is
    not a safety function and carries no safety rating; the cell's safety-rated
    zone monitoring remains responsible for preventing collisions.  The API will
    change.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import random
import uuid
from collections.abc import AsyncIterator, Iterable, Sequence

import nats
from nats.js.api import KeyValueConfig
from nats.js.client import KeyValue
from nats.js.errors import (
    BadRequestError,
    KeyNotFoundError,
    KeyWrongLastSequenceError,
    NotFoundError,
)
from nats.js.kv import KV_DEL, KV_PURGE
from pydantic import ValidationError

from nova.interlock.models import (
    BUCKET_TEMPLATE,
    AlreadyHeldError,
    ForeignRelease,
    Grant,
    InterlockTimeout,
    LockId,
    LockRecord,
)

logger = logging.getLogger(__name__)

_MAX_VALUE_SIZE = 4 * 1024
_MAX_BUCKET_BYTES = _MAX_VALUE_SIZE * 512

#: Upper bound on a single watch-based wait.  A missed wake-up costs at most
#: this before the acquire loop re-attempts; a delete wakes waiters immediately.
_WATCH_WAIT_CAP = 5.0

#: Placeholder holder id for KV entries whose payload does not parse.  Such an
#: entry blocks acquisition (treated as held) and is surfaced by
#: :meth:`InterlockClient.inspect` so an operator can ``force_release`` it.
CORRUPT_HOLDER = "<unparseable>"


class InterlockClient:
    """Per-robot client for the cell's interlock bucket.

    One instance per robot process.  Cheap to construct; the bucket handle is
    resolved lazily on first use.

    Args:
        nats_client: a connected ``nats.NATS`` — typically ``ctx.nova.nats``.
        cell: cell id, e.g. ``"urw1-agr3-sk1"``.
        robot: this robot's id, e.g. ``"ir340r01"``.  Used as the holder identity.
        create_bucket: create the bucket if missing.  Safe for every process to
            set (creation is idempotent), but one owner is tidier.
    """

    def __init__(
        self, nats_client: nats.NATS, *, cell: str, robot: str, create_bucket: bool = True
    ):
        self._nats = nats_client
        self._cell = cell
        self._robot = robot
        self._run_id = uuid.uuid4().hex[:12]
        self._bucket_name = BUCKET_TEMPLATE.format(cell=cell)
        self._create_bucket = create_bucket
        self._kv: KeyValue | None = None
        self._kv_lock = asyncio.Lock()
        self._held: dict[str, int] = {}

    @property
    def robot(self) -> str:
        return self._robot

    @property
    def run_id(self) -> str:
        return self._run_id

    @property
    def held(self) -> list[str]:
        """Keys currently held by this client, for diagnostics."""
        return sorted(self._held)

    async def _bucket(self) -> KeyValue:
        async with self._kv_lock:
            if self._kv is not None:
                return self._kv
            if not self._nats.is_connected:
                raise RuntimeError(f"NATS client is not connected for: {self._nats.connected_url}")
            js = self._nats.jetstream()
            try:
                self._kv = await js.key_value(self._bucket_name)
            except NotFoundError:
                if not self._create_bucket:
                    raise RuntimeError(
                        f"Interlock bucket {self._bucket_name} missing and create_bucket=False"
                    ) from None
                logger.info("Creating interlock bucket %s", self._bucket_name)
                try:
                    self._kv = await js.create_key_value(
                        KeyValueConfig(
                            bucket=self._bucket_name,
                            max_value_size=_MAX_VALUE_SIZE,
                            max_bytes=_MAX_BUCKET_BYTES,
                            history=1,
                        )
                    )
                except BadRequestError:
                    # Lost the creation race to a sibling robot process — the
                    # whole cell starts its processes at once, so this is routine.
                    self._kv = await js.key_value(self._bucket_name)
            return self._kv

    async def _read(self, kv: KeyValue, key: str) -> tuple[LockRecord | None, int]:
        """Return the current record and revision, or ``(None, 0)`` if free.

        An unparseable payload is returned as a record held by
        :data:`CORRUPT_HOLDER`: it blocks acquisition like any foreign lock and
        stays visible to :meth:`inspect`, so the one tool an operator uses to
        find a lock that needs :meth:`force_release` can actually see it.
        """
        try:
            entry = await kv.get(key)
        except KeyNotFoundError:
            return None, 0
        if entry.value is None:
            return None, entry.revision or 0
        try:
            return LockRecord.model_validate_json(entry.value), entry.revision or 0
        except ValidationError:
            logger.warning("Corrupt interlock record at %s; treating as held", key)
            corrupt = LockRecord(
                holder=CORRUPT_HOLDER,
                run_id="",
                slot=0,
                label="unparseable record; force_release after confirming the zone is clear",
            )
            return corrupt, entry.revision or 0

    # ------------------------------------------------------------------ acquire

    async def acquire(
        self,
        locks: Sequence[LockId],
        *,
        timeout: float = 120.0,
        label: str = "",
        poll_interval: float = 0.1,
    ) -> Grant:
        """Acquire every lock in *locks*, or none.  Blocks until granted.

        This is the ``MAKRO 20`` equivalent: pass the complete set of slots the
        step needs, exactly as the KRL sets ``A89``/``A91`` and then calls
        ``SPSMAKRO20`` once.

        Call it with the robot stationary, before planning or executing motion.

        Acquiring while already holding locks is refused (:class:`AlreadyHeldError`):
        the no-deadlock argument of this prototype is *no hold-and-wait*, so each
        step must take its complete slot set in one call.

        Raises:
            InterlockTimeout: nothing was acquired and the robot has not moved.
                Abort the program; never proceed.
            AlreadyHeldError: this client still holds locks from a previous
                acquire.  Release them first.
        """
        if not locks:
            return Grant(holder=self._robot, run_id=self._run_id, revisions={})
        if self._held:
            raise AlreadyHeldError(
                self._robot, sorted(self._held), sorted({lock.key for lock in locks})
            )

        kv = await self._bucket()
        by_key = {lock.key: lock for lock in locks}
        ordered = sorted(by_key)
        deadline = asyncio.get_running_loop().time() + timeout
        attempt = 0

        while True:
            taken: dict[str, int] = {}
            blocked_key: str | None = None

            for key in ordered:
                record = LockRecord(
                    holder=self._robot, run_id=self._run_id, slot=by_key[key].slot, label=label
                )
                payload = record.model_dump_json().encode()
                try:
                    taken[key] = await kv.create(key, payload)
                except (KeyWrongLastSequenceError, Exception) as exc:  # noqa: B014
                    if (
                        isinstance(exc, KeyWrongLastSequenceError)
                        or "wrong last sequence" in str(exc).lower()
                    ):
                        existing, revision = await self._read(kv, key)
                        if (
                            existing is not None
                            and existing.holder == self._robot
                            and existing.run_id == self._run_id
                        ):
                            # Our own lock from a previous partial attempt — adopt it.
                            taken[key] = revision
                            continue
                        blocked_key = key
                        break
                    raise

            if blocked_key is None:
                self._held.update(taken)
                logger.info(
                    "[%s] interlock acquired %s%s",
                    self._robot,
                    ordered,
                    f" ({label})" if label else "",
                )
                return Grant(holder=self._robot, run_id=self._run_id, revisions=taken)

            # Roll back completely.  Safe: the robot has not moved.
            await self._rollback(kv, taken)

            if asyncio.get_running_loop().time() >= deadline:
                blocked: dict[str, LockRecord | None] = {}
                for key in ordered:
                    record, _ = await self._read(kv, key)
                    if record is not None and record.holder != self._robot:
                        blocked[key] = record
                raise InterlockTimeout(self._robot, blocked, timeout)

            attempt += 1
            if attempt == 1 or attempt % 50 == 0:
                record, _ = await self._read(kv, blocked_key)
                logger.info(
                    "[%s] waiting for %s (held by %s)",
                    self._robot,
                    blocked_key,
                    record.holder if record else "<unknown>",
                )
            remaining = deadline - asyncio.get_running_loop().time()
            await self._wait_for_change(
                kv,
                blocked_key,
                max_wait=min(remaining, _WATCH_WAIT_CAP),
                poll_interval=poll_interval,
            )
            # Jitter breaks livelock between two robots contending for the same set.
            await asyncio.sleep(random.random() * poll_interval)

    async def _wait_for_change(
        self, kv: KeyValue, key: str, *, max_wait: float, poll_interval: float
    ) -> None:
        """Block until *key* changes, best-effort, at most *max_wait* seconds.

        Uses a KV watch when the bucket supports it, so a release wakes waiters
        immediately instead of on the next poll tick — contended hand-overs are
        on the cycle-time critical path.  Correctness never depends on the
        wake-up: on any watch failure this degrades to a jittered sleep, and the
        caller re-attempts the full acquire either way.
        """
        if max_wait <= 0:
            return
        fallback = min(poll_interval * (1.0 + random.random()), max_wait)
        watch = getattr(kv, "watch", None)
        if watch is None:
            await asyncio.sleep(fallback)
            return
        try:
            watcher = await watch(key)
        except Exception:
            await asyncio.sleep(fallback)
            return
        try:
            loop = asyncio.get_running_loop()
            deadline = loop.time() + max_wait
            while True:
                remaining = deadline - loop.time()
                if remaining <= 0:
                    return
                try:
                    entry = await watcher.updates(timeout=remaining)
                except Exception:
                    return  # timed out — re-attempt from the acquire loop
                if entry is None:
                    # End of the initial replay.  The key may have been freed
                    # before the watch started — re-check instead of waiting
                    # for an update that will never come.
                    record, _ = await self._read(kv, key)
                    if record is None:
                        return
                    continue
                if getattr(entry, "operation", None) in (KV_DEL, KV_PURGE):
                    return
        finally:
            with contextlib.suppress(Exception):
                await watcher.stop()

    async def _rollback(self, kv: KeyValue, taken: dict[str, int]) -> None:
        # Guarded on the revision we created: if the entry changed hands in the
        # meantime (operator force_release + partner acquire) it is not ours to
        # delete, and the guard makes the delete a no-op instead of a clobber.
        for key, revision in taken.items():
            with contextlib.suppress(Exception):
                await kv.delete(key, last=revision)

    # ------------------------------------------------------------------ release

    async def release(self, grant: Grant, *, slots: Iterable[int] | None = None) -> None:
        """Release locks held under *grant*.

        This is the ``Freigabe Verriegelung`` equivalent — the KRL's
        ``A41/A49/A51 = EIN`` after the robot has retreated clear of the zone.
        **Call it only once the robot is physically out**, and as an awaited call,
        never as an ``io_write`` embedded in an action list.

        Args:
            slots: release only these of the holder's own slot numbers.  Omit to
                release everything in the grant.  Mirrors the KRL releasing a
                subset early (e.g. ``ir360r01`` drops slot 7 before the rest).
                Note that a partial release keeps the client "holding", so a
                further :meth:`acquire` is still refused until the rest is
                released too.

        Raises:
            ForeignRelease: some lock in the grant is now held by someone else.
                Everything that was still legitimately ours has been released;
                the foreign locks were left untouched.  Treat as a program
                abort — this robot's view of the cell has diverged.
        """
        kv = await self._bucket()
        wanted = set(slots) if slots is not None else None
        foreign: dict[str, LockRecord] = {}

        for key, _ in sorted(grant.revisions.items()):
            record, current_rev = await self._read(kv, key)
            if record is None:
                self._held.pop(key, None)
                continue
            if record.holder != grant.holder or record.run_id != grant.run_id:
                logger.error(
                    "[%s] refusing to release %s: held by %s/%s",
                    self._robot,
                    key,
                    record.holder,
                    record.run_id,
                )
                foreign[key] = record
                continue
            if wanted is not None and record.slot not in wanted:
                continue
            try:
                await kv.delete(key, last=current_rev)
            except KeyWrongLastSequenceError:
                # Changed hands between read and delete — re-classify.
                current, _ = await self._read(kv, key)
                if current is not None:
                    foreign[key] = current
                    continue
            self._held.pop(key, None)
            logger.info("[%s] interlock released %s", self._robot, key)

        if foreign:
            raise ForeignRelease(grant.holder, foreign)

    async def release_all(self, *, include_previous_runs: bool = False) -> list[str]:
        """Release everything this robot holds.  Returns the released keys.

        Mirrors the start-of-Folge ``A41..A56 = EIN`` block.  **Call it only when
        the robot is physically clear of every zone** — at home is the canonical
        case; in the incumbent the start-of-Folge release happens exactly there.
        Do *not* call it blindly from a ``finally:`` after an aborted motion: the
        robot may have stopped inside a zone, and releasing then is the collision
        scenario the no-TTL rule exists to prevent.  On abnormal exits, leave the
        locks held (see :meth:`hold`) and release from a known-clear position.

        Args:
            include_previous_runs: also release locks left behind by an earlier
                run of this robot's process (a crashed or restarted holder gets
                a fresh ``run_id``, so by default its locks are untouchable).
                This is the programmatic recovery for the routine restart case:
                by calling it the program asserts the robot is physically clear.
                Cross-robot cleanup remains an operator :meth:`force_release`.
        """
        kv = await self._bucket()
        keys = set(self._held)
        if include_previous_runs:
            with contextlib.suppress(Exception):
                keys.update(await kv.keys())

        released: list[str] = []
        for key in sorted(keys):
            record, revision = await self._read(kv, key)
            if record is None:
                self._held.pop(key, None)
                continue
            if record.holder != self._robot:
                continue
            if not include_previous_runs and record.run_id != self._run_id:
                continue
            with contextlib.suppress(Exception):
                await kv.delete(key, last=revision)
            self._held.pop(key, None)
            released.append(key)
            logger.info("[%s] interlock released %s (release_all)", self._robot, key)
        return released

    # ------------------------------------------------------------- context mgr

    @contextlib.asynccontextmanager
    async def hold(
        self, locks: Sequence[LockId], *, timeout: float = 120.0, label: str = ""
    ) -> AsyncIterator[Grant]:
        """``async with`` wrapper around :meth:`acquire` / :meth:`release`.

        Releases **only on a clean exit** of the block.  On an exception or a
        cancellation the locks are kept: a cancelled trajectory decelerates the
        robot over roughly a second and it may come to rest *inside* the shared
        zone, so releasing on the error path is exactly the "release a zone the
        robot is still in" hazard this design forbids.  The lock outliving a
        failed program is the safe direction — the partner waits.

        Recovery after an abnormal exit: move/confirm the robot clear (e.g. at
        home), then :meth:`release_all` — or ``release_all(include_previous_runs=True)``
        from the restarted process — or an operator :meth:`force_release`.
        """
        grant = await self.acquire(locks, timeout=timeout, label=label)
        try:
            yield grant
        except BaseException:
            logger.warning(
                "[%s] abnormal exit of interlocked block %s — keeping %s held; "
                "release_all() once the robot is confirmed clear",
                self._robot,
                f"({label})" if label else "",
                grant.keys,
            )
            raise
        else:
            await self.release(grant)

    # ---------------------------------------------------------------- admin

    async def inspect(self) -> dict[str, LockRecord]:
        """Every currently-held lock in the cell.  For diagnostics and tooling."""
        kv = await self._bucket()
        out: dict[str, LockRecord] = {}
        try:
            keys = await kv.keys()
        except Exception:
            return out
        for key in keys:
            record, _ = await self._read(kv, key)
            if record is not None:
                out[key] = record
        return out

    async def force_release(self, key: str, *, reason: str) -> bool:
        """Clear a lock regardless of holder.

        Only for stale locks left by a crashed process, **after a human has
        confirmed the robot is physically clear of the zone**.  Loudly logged.

        Returns ``False`` if the key was already free, or if the lock changed
        hands between inspection and delete — in that case the operator's
        confirmation no longer describes the current holder, so nothing is
        touched; inspect again.
        """
        kv = await self._bucket()
        record, revision = await self._read(kv, key)
        if record is None and revision == 0:
            return False
        logger.warning(
            "[%s] FORCE RELEASE %s (was: %s) reason=%s",
            self._robot,
            key,
            record.holder if record else "<free>",
            reason,
        )
        try:
            await kv.delete(key, last=revision)
        except (KeyNotFoundError, KeyWrongLastSequenceError):
            logger.warning("[%s] force release of %s aborted: lock changed hands", self._robot, key)
            return False
        self._held.pop(key, None)
        return True


__all__ = [
    "CORRUPT_HOLDER",
    "AlreadyHeldError",
    "ForeignRelease",
    "Grant",
    "InterlockClient",
    "InterlockTimeout",
    "LockId",
    "LockRecord",
]
