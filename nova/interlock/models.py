"""Data models for the robot↔robot interlock prototype.

Lock identity mirrors how PLC-based robot interlocks are usually numbered: a
shared zone between two robots is a *slot number* that both partners use
symmetrically — robot A requests and releases slot N, and robot B does the same
with the same N.  ``LockId`` reconstructs a single canonical name from that pair,
so both sides independently compute the same string without any central
registry.
"""

from __future__ import annotations

import datetime
import re

import pydantic

_SAFE = re.compile(r"[^a-zA-Z0-9_=.-]")

#: NATS KV bucket, one per cell.  Mirrors ``nova_cells_{cell}_programs``.
BUCKET_TEMPLATE = "nova_cells_{cell}_interlocks"

#: NATS KV bucket holding one ``owners.<robot>`` key per claimed robot id.
OWNERS_BUCKET_TEMPLATE = "nova_cells_{cell}_interlock_owners"


def _clean(value: str) -> str:
    """NATS KV keys allow ``[-/_=.a-zA-Z0-9]``; we additionally exclude ``/`` and
    map anything unsafe to ``-``.  ``__`` separates the key components."""
    return _SAFE.sub("-", value)


class LockId(pydantic.BaseModel):
    """Canonical identity of one shared zone between exactly two robots.

    Built from the two robot ids and the interlock slot number.  The slot number
    is symmetric — if ``robot-a`` uses slot 9 for ``robot-b`` then ``robot-b``
    uses slot 9 for ``robot-a`` — which is what lets both processes derive the
    same key independently.  A PLC interlock typically flags a pair that does not
    match up as an error; here the model validator normalises the pair to a
    canonical order and rejects self-interlocks, so a mismatched pair simply
    produces a key nobody else contends for.
    """

    model_config = pydantic.ConfigDict(frozen=True)

    robot_a: str
    robot_b: str
    slot: int = pydantic.Field(ge=1, le=16)

    @pydantic.model_validator(mode="after")
    def _order_and_check(self) -> "LockId":
        # Compare what the key is built from: two spellings of one robot that differ
        # only in characters _clean maps to "-" must sort — and collide — the same way.
        a, b = _clean(self.robot_a), _clean(self.robot_b)
        if a == b:
            raise ValueError(f"interlock slot {self.slot}: a robot cannot interlock with itself")
        if a > b:
            # Normalise to a canonical order so both partners derive the same key.
            # Bind both values first — assigning in place would alias.
            lo, hi = self.robot_b, self.robot_a
            object.__setattr__(self, "robot_a", lo)
            object.__setattr__(self, "robot_b", hi)
        return self

    @classmethod
    def of(cls, robot: str, partner: str, slot: int) -> "LockId":
        """Build from one robot's point of view. ``LockId.of("robot-a", "robot-b", 9)``."""
        return cls(robot_a=robot, robot_b=partner, slot=slot)

    @property
    def key(self) -> str:
        """The NATS KV key. Stable, and identical from either partner's side."""
        return f"{_clean(self.robot_a)}__{_clean(self.robot_b)}__v{self.slot}"

    def __str__(self) -> str:
        return self.key


class LockRecord(pydantic.BaseModel):
    """The value stored in the KV bucket while a lock is held."""

    holder: str
    """Robot id of the holder, e.g. ``robot-a``."""

    run_id: str
    """Unique per process run.  Distinguishes a restarted process from its predecessor."""

    slot: int
    """The holder's own slot number — kept for traceability back to the PLC slot."""

    label: str = ""
    """Free-text, e.g. the program step that took the lock. Diagnostics only."""

    owner: str = ""
    """Session of the :class:`~nova.interlock.RobotOwnership` claim the lock was taken
    under; empty for a client without one."""

    acquired_at: datetime.datetime = pydantic.Field(
        default_factory=lambda: datetime.datetime.now(datetime.timezone.utc)
    )

    def age_seconds(self) -> float:
        now = datetime.datetime.now(datetime.timezone.utc)
        return (now - self.acquired_at).total_seconds()


class Grant(pydantic.BaseModel):
    """Handle returned by a successful acquire.  Carries the fencing revisions."""

    model_config = pydantic.ConfigDict(frozen=True)

    holder: str
    run_id: str
    revisions: dict[str, int]
    """KV key -> revision at the moment we took it.  Release is conditional on these."""

    request_id: str | None = None
    """The ``request_id`` passed to ``acquire``.  A retried acquire with the same id
    returns this grant instead of raising :class:`AlreadyHeldError`."""

    @property
    def keys(self) -> list[str]:
        return sorted(self.revisions)


class InterlockError(Exception):
    """Base class for interlock failures."""


class InterlockTimeout(InterlockError):
    """Acquire did not succeed within the timeout.

    The robot has **not** moved and holds nothing: acquisition is all-or-nothing and
    rolls back on failure.  Treat as a program abort, never as permission to proceed.
    """

    def __init__(self, holder: str, blocked_on: dict[str, LockRecord | None], timeout: float):
        self.holder = holder
        self.blocked_on = blocked_on
        self.timeout = timeout
        detail = ", ".join(
            f"{k} held by {v.holder if v else '<unknown>'}"
            + (f" for {v.age_seconds():.1f}s" if v else "")
            for k, v in blocked_on.items()
        )
        super().__init__(f"{holder} waited {timeout:.1f}s for: {detail or '<no detail>'}")


class AlreadyHeldError(InterlockError):
    """``acquire()`` was called while this client already holds locks.

    The deadlock argument of this prototype is *no hold-and-wait*: a robot never
    waits for locks while holding others, because each step takes its **complete**
    slot set in a single all-or-nothing :meth:`InterlockClient.acquire` — exactly
    like a robot program requesting all its zones from the PLC at once.  Acquiring while holding would reintroduce
    hold-and-wait (and with it, representable deadlock), so it is refused.

    Release everything first, or widen the original acquire to the full set.
    """

    def __init__(self, robot: str, held: list[str], requested: list[str]):
        self.robot = robot
        self.held = held
        self.requested = requested
        super().__init__(
            f"{robot} tried to acquire {requested} while already holding {held}; "
            "all-or-nothing interlocking forbids hold-and-wait — release first, "
            "or take the complete set in one acquire()"
        )


class ForeignRelease(InterlockError):
    """A lock from this grant is now held by someone else (or a newer run).

    Raised by :meth:`InterlockClient.release` **after** releasing every lock in
    the grant that was still legitimately ours; the foreign locks listed here
    were left untouched.  This means the caller's view of what it holds has
    diverged from the cell (e.g. an operator ``force_release`` while this robot
    believed it held the zone) — treat it as a program abort and investigate.
    """

    def __init__(self, holder: str, records: dict[str, LockRecord]):
        self.holder = holder
        self.records = records
        detail = ", ".join(f"{k} held by {v.holder}/{v.run_id}" for k, v in records.items())
        super().__init__(f"{holder} attempted to release locks it no longer holds: {detail}")


class GrantLost(InterlockError):
    """A retried ``acquire`` found its grant no longer held as granted.

    Some key of the grant was released or changed hands behind this client's back,
    typically an operator ``force_release``.  Nothing was changed.  Treat as a
    program abort: this robot's view of the cell has diverged.
    """

    def __init__(self, holder: str, records: dict[str, LockRecord | None]):
        self.holder = holder
        self.records = records
        detail = ", ".join(
            f"{k} {'free' if v is None else f'held by {v.holder}/{v.run_id}'}"
            for k, v in records.items()
        )
        super().__init__(f"{holder} retried an acquire but its grant is gone: {detail}")


class OwnershipConflict(InterlockError):
    """Another live process owns this robot id, or its owner record is unreadable.

    Stop the other process (``record`` says where it runs), then claim again.
    """

    def __init__(self, robot: str, record: object | None):
        self.robot = robot
        self.record = record
        if record is None:
            detail = "the owner record is unreadable; delete it after checking no process runs"
        else:
            detail = (
                f"session {getattr(record, 'session', '?')} "
                f"(pid {getattr(record, 'pid', '?')} on {getattr(record, 'host', '?')}) "
                "is alive"
            )
        super().__init__(f"robot id {robot} is owned by another process: {detail}")


class OwnershipLost(InterlockError):
    """This process no longer owns the robot id: it never claimed it, released it,
    or another process took over because this one did not answer liveness pings.

    Stop acting for the robot.
    """

    def __init__(self, robot: str, record: object | None):
        self.robot = robot
        self.record = record
        now = "nobody" if record is None else f"session {getattr(record, 'session', '?')}"
        super().__init__(f"this process does not own robot id {robot} (owner now: {now})")


class OwnershipRequired(InterlockError):
    """``release_all(include_previous_runs=True)`` needs a claimed robot id.

    Recovering a previous run's locks is only safe when no other process acts for
    the same robot.  Create the client with ``RobotOwnership(...).client()`` or pass
    ``owner=`` after claiming.
    """

    def __init__(self, robot: str):
        self.robot = robot
        super().__init__(
            f"{robot}: release_all(include_previous_runs=True) requires a RobotOwnership claim, "
            "so another live process for the same robot cannot lose its locks"
        )
