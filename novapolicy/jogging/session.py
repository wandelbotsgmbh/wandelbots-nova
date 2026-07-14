"""Jogging state tracking — detects blocking braking conditions.

Monitors the NOVA action-chunk-streaming state stream and raises ``MotionError``
after a confirmed blocking brake (joint limit, collision, singularity, or
workspace boundary).
"""

from __future__ import annotations

import logging
import time

from novapolicy.types import MotionError

logger = logging.getLogger(__name__)

_BLOCKING_BRAKES = frozenset(
    {
        "BRAKING_NEAR_JOINT_LIMIT",
        "BRAKING_NEAR_COLLISION",
        "BRAKING_NEAR_SINGULARITY",
        "BRAKING_NEAR_WORKSPACE_BOUNDARY",
    }
)


class JoggingStateTracker:
    """Tracks NOVA action-chunk state and raises on confirmed braking."""

    def __init__(self, motion_group_id: str, *, confirm_ticks: int = 10) -> None:
        self.motion_group_id = motion_group_id
        self._confirm_ticks = confirm_ticks
        self._braking_reason: str | None = None
        self._braking_detail: str = ""
        self._braking_count: int = 0
        self._last_kind: str | None = None
        self._t0 = time.monotonic()

    @property
    def last_kind(self) -> str | None:
        """Most recent jogging-state kind (e.g. ``RUNNING``), or ``None``.

        ``None`` means the state stream has not reported an execution state yet.
        """
        return self._last_kind

    def update_from_state(self, state: object) -> None:
        """Extract braking reason from MotionGroupState.execute.details.state."""
        jog_state = self._extract_jogging_state(state)

        if jog_state is None:
            self._braking_reason = None
            self._braking_detail = ""
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
            self._braking_reason = None
            self._braking_detail = ""
        else:
            self._braking_reason = kind
            if hasattr(jog_state, "joint_indices"):
                self._braking_detail = f"joints: {jog_state.joint_indices}"
            elif hasattr(jog_state, "description"):
                self._braking_detail = jog_state.description
            else:
                self._braking_detail = ""

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
        """Raise MotionError after confirmed blocking brake."""
        if self._braking_reason is None or self._braking_reason not in _BLOCKING_BRAKES:
            self._braking_count = 0
            return

        self._braking_count += 1
        if self._braking_count >= self._confirm_ticks:
            reason = self._braking_reason.replace("BRAKING_NEAR_", "").lower()
            detail = f" ({self._braking_detail})" if self._braking_detail else ""
            raise MotionError(
                self.motion_group_id,
                f"Jogging braking: {reason}{detail}",
            )
