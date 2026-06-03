"""Jogging state tracking — detects blocking pause conditions.

Monitors the NOVA jogging state stream and raises ``MotionError``
after a confirmed blocking pause (joint limit, collision, singularity).
"""

from __future__ import annotations

import logging

from policy.types import MotionError

logger = logging.getLogger(__name__)

_BLOCKING_PAUSES = frozenset({
    "PAUSED_NEAR_JOINT_LIMIT",
    "PAUSED_NEAR_COLLISION",
    "PAUSED_NEAR_SINGULARITY",
})


class JoggingStateTracker:
    """Tracks NOVA jogging pause state and raises on confirmed standstill."""

    def __init__(self, motion_group_id: str, *, confirm_ticks: int = 10) -> None:
        self.motion_group_id = motion_group_id
        self._confirm_ticks = confirm_ticks
        self._paused_reason: str | None = None
        self._paused_detail: str = ""
        self._paused_count: int = 0

    def update_from_state(self, state: object) -> None:
        """Extract jogging pause reason from MotionGroupState.execute.details."""
        jog_state = self._extract_jogging_state(state)

        if jog_state is None:
            self._paused_reason = None
            self._paused_detail = ""
            return

        kind: str = getattr(jog_state, "kind", "RUNNING")
        if kind == "RUNNING":
            self._paused_reason = None
            self._paused_detail = ""
        else:
            self._paused_reason = kind
            if hasattr(jog_state, "joint_indices"):
                self._paused_detail = f"joints: {jog_state.joint_indices}"
            elif hasattr(jog_state, "description"):
                self._paused_detail = jog_state.description
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
