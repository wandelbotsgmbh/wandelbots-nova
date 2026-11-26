"""Joint velocities for ``Observation.joint_velocities`` (contract M9, ADR 0016).

Measured where the controller sends them, otherwise finite-differenced, which is reported once per
motion group.
"""

from __future__ import annotations

from collections import deque
import logging
import time
from typing import TYPE_CHECKING

from novapolicy.mapping.state_stream import StateStreamVelocities

if TYPE_CHECKING:
    from collections.abc import Sequence

    from nova.cell.motion_group import MotionGroup
    from novapolicy.mapping.state_stream import VelocitySource

logger = logging.getLogger(__name__)

_MIN_DT_S = 1e-4
"""Below this, two positions are the same sample; the previous velocity is kept."""

DIFFERENCE_WINDOW_S = 0.02
"""Least sample time, in s, streamed positions are differenced across (against quantization)."""

_REPEAT_HOLD_S = 0.1
"""A repeated sample keeps the last velocity this long; positions unchanged for longer read as
standstill."""

_FED_STALE_S = 0.2
"""A velocity from streamed packets older than this is not used."""

_FIRST_SAMPLE_GRACE_S = 0.5
"""Seconds after start before a missing measurement is reported."""


class JointVelocities:
    """Measured joint velocities where available, finite-differenced otherwise."""

    def __init__(
        self, motion_groups: list[MotionGroup], *, measured: VelocitySource | None = None
    ) -> None:
        self._motion_groups = motion_groups
        self._measured = measured
        self._owns_measured = measured is None
        self._started_at: float | None = None
        self._sample: dict[str, tuple[float, list[float]]] = {}  # last *changed* positions
        self._velocity: dict[str, list[float]] = {}
        self._packets: dict[str, deque[tuple[float, list[float]]]] = {}
        self._fed: dict[str, tuple[float, list[float]]] = {}  # (monotonic, velocity) from packets
        self._fallback_reported: set[str] = set()

    async def start(self) -> None:
        """Open the measured-velocity streams (one per motion group)."""
        if self._started_at is not None:
            return
        if self._measured is None:
            self._measured = StateStreamVelocities(self._motion_groups)
        await self._measured.start()
        self._started_at = time.monotonic()

    async def stop(self) -> None:
        """Close the streams and forget the finite-difference history."""
        if self._started_at is not None and self._measured is not None:
            await self._measured.stop()
        if self._owns_measured:
            self._measured = None
        self._started_at = None
        self._sample.clear()
        self._velocity.clear()
        self._packets.clear()
        self._fed.clear()

    def feed(self, motion_group_id: str, joints: Sequence[float], sample_time_s: float) -> None:
        """Feed one streamed state packet, stamped with when it was measured."""
        joints = [float(q) for q in joints]
        packets = self._packets.setdefault(motion_group_id, deque(maxlen=64))
        if packets and sample_time_s <= packets[-1][0]:
            return  # a repeated or reordered packet
        packets.append((sample_time_s, joints))
        older = [p for p in packets if sample_time_s - p[0] >= DIFFERENCE_WINDOW_S]
        if not older or len(older[-1][1]) != len(joints):
            return
        then, before = older[-1]
        velocity = [(q - p) / (sample_time_s - then) for q, p in zip(joints, before, strict=True)]
        self._fed[motion_group_id] = (time.monotonic(), velocity)

    def read(
        self, motion_group_id: str, joints: Sequence[float], now: float | None = None
    ) -> list[float]:
        """Velocities (rad/s) of ``motion_group_id``, whose joints are now at ``joints``."""
        now = time.monotonic() if now is None else now
        joints = [float(q) for q in joints]
        source = self._measured if self._started_at is not None else None
        measured = source.velocities(motion_group_id) if source is not None else None
        differenced = self._difference(motion_group_id, joints, now)  # keeps the history going
        if measured is not None and len(measured) == len(joints):
            return [float(v) for v in measured]
        self._report_fallback(motion_group_id, measured, len(joints), now)
        fed = self._fed.get(motion_group_id)
        if (
            fed is not None
            and time.monotonic() - fed[0] < _FED_STALE_S
            and len(fed[1]) == len(joints)
        ):
            return list(fed[1])
        return differenced

    def _difference(self, motion_group_id: str, joints: list[float], now: float) -> list[float]:
        """Positions differenced against the last sample that differed from them.

        A repeated sample keeps the last velocity for up to ``_REPEAT_HOLD_S``.
        """
        sample = self._sample.get(motion_group_id)
        if sample is None or len(sample[1]) != len(joints):
            self._sample[motion_group_id] = (now, joints)
            self._velocity[motion_group_id] = [0.0] * len(joints)
        elif joints != sample[1] and now - sample[0] >= _MIN_DT_S:
            then, before = sample
            self._velocity[motion_group_id] = [
                (q - p) / (now - then) for q, p in zip(joints, before, strict=True)
            ]
            self._sample[motion_group_id] = (now, joints)
        elif joints == sample[1] and now - sample[0] > _REPEAT_HOLD_S:
            self._velocity[motion_group_id] = [0.0] * len(joints)
            self._sample[motion_group_id] = (now, joints)
        return list(self._velocity[motion_group_id])

    def _report_fallback(
        self, motion_group_id: str, measured: Sequence[float] | None, width: int, now: float
    ) -> None:
        if motion_group_id in self._fallback_reported:
            return
        started_at = self._started_at
        if started_at is not None and measured is None and now - started_at < _FIRST_SAMPLE_GRACE_S:
            return  # the first measurement may still be on its way
        self._fallback_reported.add(motion_group_id)
        why = (
            "the stream is not started"
            if started_at is None
            else "the controller sends none"
            if measured is None
            else f"it sends {len(measured)} values for {width} joints"
        )
        logger.warning(
            "No measured joint velocities for %s (%s); finite-differencing joint positions, "
            "which is noisier than a measurement",
            motion_group_id,
            why,
        )
