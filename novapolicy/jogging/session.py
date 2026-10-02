"""Jogging state tracking — blocking pauses and standstill detection.

``JoggingStateTracker`` monitors the NOVA jogging state stream and raises
``MotionError`` after a confirmed blocking pause (joint limit, collision,
singularity).

``StandstillDetector`` answers the separate question of whether the robot has
actually stopped moving. The server has no state for "the commanded waypoints
ran out": since NOVA 26.6 ``PAUSED_BY_USER`` is reported only in response to a
Pause/Stop *request*, and this SDK never sends one, so a drained queue leaves
the stream reporting ``RUNNING`` forever. Standstill therefore has to be
measured from the joint positions themselves.
"""

from __future__ import annotations

import logging
import time

from novapolicy.types import MotionError

logger = logging.getLogger(__name__)

_BLOCKING_PAUSES = frozenset({
    "PAUSED_NEAR_JOINT_LIMIT",
    "PAUSED_NEAR_COLLISION",
    "PAUSED_NEAR_SINGULARITY",
})


# How far a joint may drift from its reference before the robot counts as moving
# [rad] (~0.06 deg). Above encoder noise at standstill, far below any commanded
# motion. The reference only moves when this is exceeded, so noise stays bounded
# under it forever while a slow crawl keeps accumulating until it trips.
_STANDSTILL_EPS_RAD = 1e-3

# How long every joint must sit inside that band before the robot is settled
# [server ms]. Long enough to outlast the tail of a braking ramp, short enough
# not to dominate a sequential policy's cycle time.
_STANDSTILL_HOLD_MS = 60


class StandstillDetector:
    """Decides whether the joints have stopped moving, from state samples alone.

    Fed ``(server_ms, joints)`` from the state stream. A *reference* posture is
    held and only replaced once some joint leaves the
    ``_STANDSTILL_EPS_RAD`` band around it; the moment that happens is recorded
    as the last motion. Keeping the reference fixed while inside the band is
    what makes a slow crawl detectable: the deviation accumulates against an
    anchor instead of against the previous sample, where a creep of less than
    one epsilon per sample would read as standstill indefinitely.

    The hold is measured in *server* milliseconds. State packets arrive in
    bursts on some deployments — stretches of near-silence followed by a dozen
    at once — so counting samples, or measuring on the client clock, would call
    a burst-delivered standstill either far too early or far too late.
    """

    def __init__(self, *, hold_ms: float = _STANDSTILL_HOLD_MS) -> None:
        self._hold_ms = hold_ms
        self._reference: list[float] | None = None
        self._last_motion_ms: int | None = None
        self._last_sample_ms: int | None = None

    def update(self, server_ms: int, joints: list[float]) -> None:
        """Record one state sample, taken at ``server_ms``."""
        self._last_sample_ms = server_ms
        reference = self._reference
        if reference is None or len(reference) != len(joints):
            self._reference = list(joints)
            self._last_motion_ms = server_ms
            return
        if any(
            abs(now - ref) > _STANDSTILL_EPS_RAD for now, ref in zip(joints, reference, strict=True)
        ):
            self._reference = list(joints)
            self._last_motion_ms = server_ms

    @property
    def standstill_ms(self) -> float:
        """How long the joints have held their reference posture [server ms].

        ``0.0`` before the first sample, and whenever motion was just seen.
        """
        if self._last_sample_ms is None or self._last_motion_ms is None:
            return 0.0
        return max(0.0, float(self._last_sample_ms - self._last_motion_ms))

    @property
    def is_settled(self) -> bool:
        """Whether the joints have been still for the full hold window."""
        if self._last_sample_ms is None:
            # No state yet: unknown, and "unknown" must not read as stopped.
            return False
        return self.standstill_ms >= self._hold_ms


class JoggingStateTracker:
    """Tracks NOVA jogging pause state and raises on confirmed standstill."""

    def __init__(self, motion_group_id: str, *, confirm_ticks: int = 10) -> None:
        self.motion_group_id = motion_group_id
        self._confirm_ticks = confirm_ticks
        self._paused_reason: str | None = None
        self._paused_detail: str = ""
        self._paused_count: int = 0
        self._last_kind: str | None = None
        self._t0 = time.monotonic()

    @property
    def last_kind(self) -> str | None:
        """Most recent jogging-state kind (e.g. ``RUNNING``), or ``None``.

        ``None`` means the state stream has not reported an execution state yet.
        """
        return self._last_kind

    def update_from_state(self, state: object) -> None:
        """Extract jogging pause reason from MotionGroupState.execute.details."""
        jog_state = self._extract_jogging_state(state)

        if jog_state is None:
            self._paused_reason = None
            self._paused_detail = ""
            return

        kind: str = getattr(jog_state, "kind", "RUNNING")
        if kind != self._last_kind:
            logger.debug(
                "%s jogging state -> %s (+%.0fms)",
                self.motion_group_id,
                kind,
                (time.monotonic() - self._t0) * 1000,
            )
            self._last_kind = kind
        if kind == "RUNNING":
            self._paused_reason = None
            self._paused_detail = ""
        else:
            self._paused_reason = kind
            joint_indices = getattr(jog_state, "joint_indices", None)
            description = getattr(jog_state, "description", None)
            if joint_indices is not None:
                self._paused_detail = f"joints: {joint_indices}"
            elif isinstance(description, str):
                self._paused_detail = description
            else:
                self._paused_detail = ""

    @staticmethod
    def _extract_jogging_state(state: object) -> object | None:
        """Navigate MotionGroupState.execute.details.state safely."""
        execute = getattr(state, "execute", None)
        if execute is None:
            return None
        details = getattr(execute, "details", None)
        if details is None:
            return None
        return getattr(details, "state", None)

    def check(self) -> None:
        """Raise MotionError after confirmed blocking pause."""
        if self._paused_reason is None or self._paused_reason not in _BLOCKING_PAUSES:
            self._paused_count = 0
            return

        self._paused_count += 1
        if self._paused_count >= self._confirm_ticks:
            reason = self._paused_reason.replace("PAUSED_NEAR_", "").lower()
            detail = f" ({self._paused_detail})" if self._paused_detail else ""
            raise MotionError(
                self.motion_group_id,
                f"Jogging paused: {reason}{detail}",
            )
