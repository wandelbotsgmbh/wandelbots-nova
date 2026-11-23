"""Jogging state tracking: blocking pauses (``JoggingStateTracker``) and standstill from the
joint positions (``StandstillDetector``, docs/executor.md#detecting-standstill)."""

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


def jogging_details(state: object) -> object | None:
    """``state.execute.details`` of a ``MotionGroupState``, or ``None`` when it has none."""
    execute = getattr(state, "execute", None)
    return None if execute is None else getattr(execute, "details", None)


# How far a joint may drift from its reference before the robot counts as moving [rad].
_STANDSTILL_EPS_RAD = 1e-3

# How long every joint must sit inside that band before the robot is settled [server ms]:
# past a braking ramp's tail, short against a sequential policy's cycle.
_STANDSTILL_HOLD_MS = 60


class StandstillDetector:
    """Decides whether the joints have stopped moving, from ``(server_ms, joints)`` samples.

    A reference posture is replaced only when a joint leaves the ``_STANDSTILL_EPS_RAD`` band
    around it; the hold is measured in server milliseconds.
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
        """How long the joints have held their reference posture [server ms]; 0 after motion."""
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
        """Most recent jogging-state kind (e.g. ``RUNNING``), or ``None`` before one."""
        return self._last_kind

    def update_from_state(self, state: object) -> None:
        """Extract jogging pause reason from MotionGroupState.execute.details."""
        jog_state = getattr(jogging_details(state), "state", None)

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
