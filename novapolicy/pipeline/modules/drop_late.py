"""``DropLate``: drop the waypoints due too soon to be sent (contract T3, ADR 0004)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext, TimedWaypoint
    from novapolicy.transport.waypoints import PendingChunk

MIN_LEAD_MS = 20.0
"""Smallest lead, in ms, a waypoint may still be sent with (contract T3)."""


@dataclass(frozen=True, slots=True)
class DropLate:
    """Filter: drop the waypoints due within ``min_lead_ms`` of now; the rest keep their times.

    Contract T3, ADR 0004. A chunk stamped at the send passes whole; when every waypoint is late,
    nothing is sent.
    """

    min_lead_ms: float = MIN_LEAD_MS

    def keep(
        self, waypoints: list[TimedWaypoint], chunk: PendingChunk, context: ModuleContext
    ) -> list[TimedWaypoint]:
        if not chunk.lead and chunk.first_timestamp_ms is None:
            return waypoints
        earliest_ms = context.now_ms + self.min_lead_ms
        return [(t, v) for t, v in waypoints if t >= earliest_ms]
