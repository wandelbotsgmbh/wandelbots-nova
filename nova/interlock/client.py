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

**Atomicity.**  A robot typically needs several zones at once (in the GEO40 trio
each robot takes its full set in a single ``MAKRO 20`` call).  :meth:`acquire`
takes them in sorted key order and rolls back completely on any conflict, so a
caller never holds a partial set while waiting.  Rollback is free here because
acquisition happens with the robot stationary, before any motion is planned or
started — nothing has to be undone physically.  With no partial holds there is
no hold-and-wait, hence no deadlock among callers that use a single acquire.

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
from nats.js.errors import KeyNotFoundError, KeyWrongLastSequenceError, NotFoundError
from pydantic import ValidationError

from nova.interlock.models import (
    BUCKET_TEMPLATE,
    ForeignRelease,
    Grant,
    InterlockTimeout,
    LockId,
    LockRecord,
)

logger = logging.getLogger(__name__)

_MAX_VALUE_SIZE = 4 * 1024
_MAX_BUCKET_BYTES = _MAX_VALUE_SIZE * 512


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
        self,
        nats_client: nats.NATS,
        *,
        cell: str,
        robot: str,
        create_bucket: bool = True,
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
                raise RuntimeError(
                    f"NATS client is not connected for: {self._nats.connected_url}"
                )
            js = self._nats.jetstream()
            try:
                self._kv = await js.key_value(self._bucket_name)
            except NotFoundError:
                if not self._create_bucket:
                    raise RuntimeError(
                        f"Interlock bucket {self._bucket_name} missing and create_bucket=False"
                    ) from None
                logger.info("Creating interlock bucket %s", self._bucket_name)
                self._kv = await js.create_key_value(
                    KeyValueConfig(
                        bucket=self._bucket_name,
                        max_value_size=_MAX_VALUE_SIZE,
                        max_bytes=_MAX_BUCKET_BYTES,
                        history=1,
                    )
                )
            return self._kv

    async def _read(self, kv: KeyValue, key: str) -> tuple[LockRecord | None, int]:
        """Return the current record and revision, or ``(None, 0)`` if free."""
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
            return None, entry.revision or 0

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

        Raises:
            InterlockTimeout: nothing was acquired and the robot has not moved.
                Abort the program; never proceed.
        """
        if not locks:
            return Grant(holder=self._robot, run_id=self._run_id, revisions={})

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
                    holder=self._robot,
                    run_id=self._run_id,
                    slot=by_key[key].slot,
                    label=label,
                )
                payload = record.model_dump_json().encode()
                try:
                    taken[key] = await kv.create(key, payload)
                except (KeyWrongLastSequenceError, Exception) as exc:  # noqa: B014
                    if isinstance(exc, KeyWrongLastSequenceError) or "wrong last sequence" in str(
                        exc
                    ).lower():
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
                blocked = {}
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
            # Jitter breaks livelock between two robots contending for the same set.
            await asyncio.sleep(poll_interval * (1.0 + random.random()))

    async def _rollback(self, kv: KeyValue, taken: dict[str, int]) -> None:
        for key in taken:
            with contextlib.suppress(Exception):
                await kv.delete(key)

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
        """
        kv = await self._bucket()
        wanted = set(slots) if slots is not None else None

        for key, revision in sorted(grant.revisions.items()):
            record, current_rev = await self._read(kv, key)
            if record is None:
                self._held.pop(key, None)
                continue
            if wanted is not None and record.slot not in wanted:
                continue
            if record.holder != grant.holder or record.run_id != grant.run_id:
                logger.error(
                    "[%s] refusing to release %s: held by %s/%s",
                    self._robot,
                    key,
                    record.holder,
                    record.run_id,
                )
                continue
            try:
                await kv.delete(key, last=current_rev)
            except TypeError:
                # older nats-py: delete() without revision guard
                await kv.delete(key)
            self._held.pop(key, None)
            logger.info("[%s] interlock released %s", self._robot, key)

    async def release_all(self) -> None:
        """Release everything this client currently holds.

        Mirrors the start-of-Folge ``A41..A56 = EIN`` block and is the right thing
        to call from a ``finally:`` when a Folge aborts.
        """
        kv = await self._bucket()
        for key in list(self._held):
            record, revision = await self._read(kv, key)
            if record is not None and record.run_id == self._run_id:
                with contextlib.suppress(Exception):
                    await kv.delete(key)
                logger.info("[%s] interlock released %s (release_all)", self._robot, key)
            self._held.pop(key, None)

    # ------------------------------------------------------------- context mgr

    @contextlib.asynccontextmanager
    async def hold(
        self,
        locks: Sequence[LockId],
        *,
        timeout: float = 120.0,
        label: str = "",
    ) -> AsyncIterator[Grant]:
        """``async with`` wrapper around :meth:`acquire` / :meth:`release`.

        Release is shielded so that a cancellation during the guarded block still
        releases — but note that if the *process* dies the lock stays held, by
        design.
        """
        grant = await self.acquire(locks, timeout=timeout, label=label)
        try:
            yield grant
        finally:
            with contextlib.suppress(asyncio.CancelledError):
                await asyncio.shield(self.release(grant))

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
        """
        kv = await self._bucket()
        record, _ = await self._read(kv, key)
        logger.warning(
            "[%s] FORCE RELEASE %s (was: %s) reason=%s",
            self._robot,
            key,
            record.holder if record else "<free>",
            reason,
        )
        try:
            await kv.delete(key)
        except KeyNotFoundError:
            return False
        self._held.pop(key, None)
        return True


__all__ = ["ForeignRelease", "Grant", "InterlockClient", "InterlockTimeout", "LockId", "LockRecord"]
