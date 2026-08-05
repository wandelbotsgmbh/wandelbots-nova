"""Data models for the robot↔robot interlock prototype.

Lock identity mirrors the VASS signal scheme it replaces.  In the source system a
shared zone between two robots is a *slot number* that both partners use
symmetrically: robot A requests slot N with ``A(80+N)`` and releases it with
``A(40+N)``; robot B does the same with the same N.  ``LockId`` reconstructs a
single canonical name from that pair, so both sides independently compute the
same string without any central registry.
"""

from __future__ import annotations

import datetime
import re

import pydantic

_SAFE = re.compile(r"[^a-zA-Z0-9_=.-]")

#: NATS KV bucket, one per cell.  Mirrors ``nova_cells_{cell}_programs``.
BUCKET_TEMPLATE = "nova_cells_{cell}_interlocks"


def _clean(value: str) -> str:
    """NATS KV keys allow ``[-/_=.a-zA-Z0-9]``; ``/`` is reserved here as a separator."""
    return _SAFE.sub("-", value)


class LockId(pydantic.BaseModel):
    """Canonical identity of one shared zone between exactly two robots.

    Built from the two robot ids and the VASS interlock slot number.  The slot
    number is symmetric in the source system — if ``ir340r01`` uses slot 9 for
    ``ir340r02`` then ``ir340r02`` uses slot 9 for ``ir340r01`` — which is what
    lets both processes derive the same key independently.  ``FB207`` enforces
    exactly this reciprocity (``xFehlerNr3``/``PAFE``); :meth:`validate_pair`
    is the client-side equivalent.
    """

    model_config = pydantic.ConfigDict(frozen=True)

    robot_a: str
    robot_b: str
    slot: int = pydantic.Field(ge=1, le=16)

    @pydantic.model_validator(mode="after")
    def _order_and_check(self) -> "LockId":
        if self.robot_a == self.robot_b:
            raise ValueError(f"interlock slot {self.slot}: a robot cannot interlock with itself")
        if self.robot_a > self.robot_b:
            # Normalise to a canonical order so both partners derive the same key.
            # Bind both values first — assigning in place would alias.
            lo, hi = self.robot_b, self.robot_a
            object.__setattr__(self, "robot_a", lo)
            object.__setattr__(self, "robot_b", hi)
        return self

    @classmethod
    def of(cls, robot: str, partner: str, slot: int) -> "LockId":
        """Build from one robot's point of view. ``LockId.of("ir340r01", "ir340r02", 9)``."""
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
    """Robot id of the holder, e.g. ``ir340r01``."""

    run_id: str
    """Unique per process run.  Distinguishes a restarted process from its predecessor."""

    slot: int
    """The holder's own VASS slot number — kept for traceability back to ``A8x``/``A4x``."""

    label: str = ""
    """Free-text, e.g. the Folge step that took the lock. Diagnostics only."""

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


class ForeignRelease(InterlockError):
    """Attempted to release a lock held by someone else (or a newer run)."""
