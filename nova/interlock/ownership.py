"""One live process per robot id.

The interlock identifies a lock holder by robot id.  Two processes using the same
robot id — a dev PC dispatcher next to the cell app, say — would each believe the
other's locks are their own: the one that reaches home calls
``release_all(include_previous_runs=True)`` and frees locks the other process may
still be standing inside.

:class:`RobotOwnership` closes that gap.  Before a process may recover a previous
run's locks, it claims the robot id:

- The claim is a KV ``create`` of ``owners/<robot>`` in a separate bucket, so it is
  atomic: only one claimant wins.
- While holding the claim, the owner answers liveness pings on a subject derived
  from its session id.
- A later claimant that finds the key taken pings the recorded session.  An answer
  means the owner is alive, and the claim is refused (:class:`OwnershipConflict`).
  Only when nobody answers is the owner considered dead; the claimant then takes
  over with a revision-guarded ``update``, so two claimants racing for the same
  dead owner cannot both win.

There is no heartbeat and no TTL: nothing changes in the cell while the owner is
alive, and liveness is asked for only when someone wants the robot id.

.. warning::
    A live owner that cannot reach NATS for longer than the liveness window looks
    dead.  A successor may then take over, and the old owner learns about it on
    its next :meth:`RobotOwnership.verify` (done by every ``acquire``).  This is a
    coordination aid, not a safety function — see :mod:`nova.interlock`.
"""

from __future__ import annotations

import asyncio
import contextlib
import datetime
import logging
import os
import socket
import uuid

import nats
import nats.errors
import pydantic
from nats.js.api import KeyValueConfig
from nats.js.client import KeyValue
from nats.js.errors import BadRequestError, KeyNotFoundError, NotFoundError
from pydantic import ValidationError

from nova.interlock.client import InterlockClient, _is_wrong_last_sequence
from nova.interlock.models import OWNERS_BUCKET_TEMPLATE, OwnershipConflict, OwnershipLost, _clean

logger = logging.getLogger(__name__)

#: Liveness pings for a session go to this subject.
PING_SUBJECT_TEMPLATE = "nova.interlock.{cell}.owners.{session}"

#: A claim that keeps losing revision races gives up after this many rounds.
_MAX_CLAIM_ROUNDS = 5


class OwnerRecord(pydantic.BaseModel):
    """The value stored under ``owners/<robot>`` while a process owns the robot id."""

    robot: str
    session: str
    """Unique per claim.  Lock records written under this claim carry it as ``owner``."""

    host: str = ""
    pid: int = 0
    claimed_at: datetime.datetime = pydantic.Field(
        default_factory=lambda: datetime.datetime.now(datetime.timezone.utc)
    )


class RobotOwnership:
    """This process's claim on a robot id.  Use as an async context manager.

    Example::

        async with RobotOwnership(ctx.nova.nats, cell="cell", robot="robot-a") as owner:
            locks = owner.client()
            await locks.release_all(include_previous_runs=True)  # robot at home
            ...

    Args:
        nats_client: a connected ``nats.NATS``.
        cell: cell id.
        robot: the robot id to claim.
        create_bucket: create the owners bucket if missing.
        liveness_timeout: how long one liveness ping waits for an answer.
        liveness_attempts: pings sent before an existing owner is considered dead.
            The window must outlast a NATS reconnect of a live owner; with the
            defaults it is about three seconds.
    """

    def __init__(
        self,
        nats_client: nats.NATS,
        *,
        cell: str,
        robot: str,
        create_bucket: bool = True,
        liveness_timeout: float = 1.0,
        liveness_attempts: int = 3,
    ):
        if liveness_attempts < 1:
            raise ValueError("liveness_attempts must be at least 1")
        self._nats = nats_client
        self._cell = cell
        self._robot = robot
        self._session = uuid.uuid4().hex[:12]
        self._bucket_name = OWNERS_BUCKET_TEMPLATE.format(cell=cell)
        self._key = f"owners.{_clean(robot)}"
        self._create_bucket = create_bucket
        self._liveness_timeout = liveness_timeout
        self._liveness_attempts = liveness_attempts
        self._kv: KeyValue | None = None
        self._revision: int | None = None
        self._subscription = None

    @property
    def robot(self) -> str:
        return self._robot

    @property
    def cell(self) -> str:
        return self._cell

    @property
    def session(self) -> str:
        return self._session

    @property
    def claimed(self) -> bool:
        """True between a successful :meth:`claim` and :meth:`release` (or a detected loss)."""
        return self._revision is not None

    def client(self, *, create_bucket: bool = True) -> InterlockClient:
        """An :class:`InterlockClient` for this robot that acts under this claim.

        Call it once per stage when a program uses several clients for one robot;
        they all share the claim.
        """
        return InterlockClient(
            self._nats, cell=self._cell, robot=self._robot, create_bucket=create_bucket, owner=self
        )

    # ------------------------------------------------------------------ claim

    async def claim(self) -> None:
        """Claim the robot id.

        Raises:
            OwnershipConflict: another live process owns the robot id, or its
                record is unreadable.  Stop that process (or fix the record),
                then try again.
        """
        if self.claimed:
            return
        kv = await self._bucket()
        # Answer pings before the key names us, so a racing claimant that reads
        # our record finds us alive.
        await self._start_answering()
        try:
            self._revision = await self._claim(kv)
        except BaseException:
            await self._stop_answering()
            raise
        logger.info("[%s] owns robot id (session %s)", self._robot, self._session)

    async def _claim(self, kv: KeyValue) -> int:
        payload = self._record().model_dump_json().encode()
        for _ in range(_MAX_CLAIM_ROUNDS):
            try:
                return await kv.create(self._key, payload)
            except Exception as exc:
                if not _is_wrong_last_sequence(exc):
                    raise
            existing, revision = await self._read(kv)
            if revision == 0:
                continue  # freed between create and read
            if existing is None:
                raise OwnershipConflict(self._robot, None)
            if existing.session == self._session:
                return revision
            if await self._is_alive(existing.session):
                raise OwnershipConflict(self._robot, existing)
            logger.warning(
                "[%s] taking over robot id from unresponsive session %s (pid %s on %s)",
                self._robot,
                existing.session,
                existing.pid,
                existing.host or "<unknown host>",
            )
            try:
                return await kv.update(self._key, payload, last=revision)
            except Exception as exc:
                if not _is_wrong_last_sequence(exc):
                    raise
                # Another claimant got there first; look again.
        existing, _ = await self._read(kv)
        raise OwnershipConflict(self._robot, existing)

    async def _is_alive(self, session: str) -> bool:
        subject = self._ping_subject(session)
        for attempt in range(self._liveness_attempts):
            try:
                await self._nats.request(subject, b"", timeout=self._liveness_timeout)
                return True
            except nats.errors.NoRespondersError:
                # Fails at once; wait so the attempts still span the window
                # (the owner may be in the middle of a reconnect).
                if attempt + 1 < self._liveness_attempts:
                    await asyncio.sleep(self._liveness_timeout)
            except nats.errors.TimeoutError:
                pass
        return False

    # ----------------------------------------------------------------- verify

    async def verify(self) -> None:
        """Check that the claim still names this session.

        Raises:
            OwnershipLost: the claim was never made, was released, or another
                process took over (this one looked dead to it).  Stop acting
                for the robot.
        """
        if self._revision is None:
            raise OwnershipLost(self._robot, None)
        record, revision = await self._read(await self._bucket())
        if record is None or record.session != self._session or revision != self._revision:
            self._revision = None
            await self._stop_answering()
            raise OwnershipLost(self._robot, record)

    # ---------------------------------------------------------------- release

    async def release(self) -> None:
        """Give up the claim.  Locks held under it stay held.

        A successor finds the key free and claims it without waiting for the
        liveness window.
        """
        revision, self._revision = self._revision, None
        await self._stop_answering()
        if revision is None:
            return
        kv = await self._bucket()
        try:
            await kv.delete(self._key, last=revision)
        except Exception as exc:
            if not (isinstance(exc, KeyNotFoundError) or _is_wrong_last_sequence(exc)):
                raise
            # Someone took over already; the key is not ours to delete.
        logger.info("[%s] released robot id (session %s)", self._robot, self._session)

    async def __aenter__(self) -> RobotOwnership:
        await self.claim()
        return self

    async def __aexit__(self, *exc_info) -> None:
        await self.release()

    # ---------------------------------------------------------------- helpers

    def _record(self) -> OwnerRecord:
        return OwnerRecord(
            robot=self._robot, session=self._session, host=socket.gethostname(), pid=os.getpid()
        )

    def _ping_subject(self, session: str) -> str:
        return PING_SUBJECT_TEMPLATE.format(cell=_clean(self._cell), session=session)

    async def _start_answering(self) -> None:
        if self._subscription is not None:
            return

        async def answer(msg) -> None:
            with contextlib.suppress(Exception):
                await msg.respond(self._session.encode())

        self._subscription = await self._nats.subscribe(
            self._ping_subject(self._session), cb=answer
        )

    async def _stop_answering(self) -> None:
        subscription, self._subscription = self._subscription, None
        if subscription is not None:
            with contextlib.suppress(Exception):
                await subscription.unsubscribe()

    async def _read(self, kv: KeyValue) -> tuple[OwnerRecord | None, int]:
        """The current record and revision; ``(None, 0)`` if free, ``(None, rev)`` if unreadable."""
        try:
            entry = await kv.get(self._key)
        except KeyNotFoundError:
            return None, 0
        if entry.value is None:
            return None, 0
        try:
            return OwnerRecord.model_validate_json(entry.value), entry.revision or 0
        except ValidationError:
            logger.warning("Corrupt owner record at %s", self._key)
            return None, entry.revision or 0

    async def _bucket(self) -> KeyValue:
        if self._kv is not None:
            return self._kv
        js = self._nats.jetstream()
        try:
            self._kv = await js.key_value(self._bucket_name)
        except NotFoundError:
            if not self._create_bucket:
                raise RuntimeError(
                    f"Owners bucket {self._bucket_name} missing and create_bucket=False"
                ) from None
            try:
                self._kv = await js.create_key_value(
                    KeyValueConfig(bucket=self._bucket_name, max_value_size=1024, history=1)
                )
            except BadRequestError:
                self._kv = await js.key_value(self._bucket_name)
        return self._kv
