"""Observers of a synchronized execution session's state streams."""

import asyncio
from collections.abc import AsyncIterable, Mapping
from datetime import datetime
from typing import Protocol

from nova import api
from nova.exceptions import ErrorDuringMovement

# Location spread at which a drift monitor aborts a session by default.
DEFAULT_MAX_DRIFT = 0.2

# Bound on samples kept per group while waiting for the other groups to report
# the same timestamp.
_MAX_PENDING_DRIFT_SAMPLES = 100


class SyncDriftError(ErrorDuringMovement):
    """Raised when synchronized motion groups drift apart beyond the allowed spread."""

    def __init__(self, locations: dict[str, float], max_drift: float):
        self.locations = locations
        self.max_drift = max_drift
        location_details = ", ".join(
            f"{name}={location:.4f}" for name, location in locations.items()
        )
        super().__init__(
            f"Sync drift exceeded threshold: {location_details}, max_drift={max_drift:.4f}"
        )


class SessionMonitor(Protocol):
    """Observer of one execution session's per-group state streams.

    Runs inside the session's TaskGroup for the whole session; raising aborts
    the session, cancelling every sibling task — which closes the
    ``executeTrajectory`` sockets, the most reliable stop channel for
    physically coupled groups.
    """

    async def run(
        self, labeled_streams: Mapping[str, AsyncIterable[api.models.MotionGroupState]]
    ) -> None:
        pass


def _extract_location(state: api.models.MotionGroupState) -> float | None:
    if state.execute is None or not isinstance(state.execute.details, api.models.TrajectoryDetails):
        return None
    return state.execute.details.location.root


class SyncDriftMonitor:
    """Abort the session when the location spread across groups grows too large.

    Consumes every group's state stream, indexes locations by timestamp
    (rounded to 1 ms), and raises :class:`SyncDriftError` when all groups have
    a sample at the same timestamp whose spread exceeds ``max_drift``.
    Comparing at matching timestamps assumes the groups' state streams share a
    clock — true on one controller, not guaranteed across controllers.

    Holds no per-run state, so one instance can serve an executor's lifetime.
    """

    def __init__(self, max_drift: float = DEFAULT_MAX_DRIFT):
        self._max_drift = max_drift

    async def run(
        self, labeled_streams: Mapping[str, AsyncIterable[api.models.MotionGroupState]]
    ) -> None:
        pending_locations: dict[str, dict[datetime, float]] = {name: {} for name in labeled_streams}
        try:
            async with asyncio.TaskGroup() as task_group:
                for name, stream in labeled_streams.items():
                    task_group.create_task(self._track(name, stream, pending_locations))
        except BaseExceptionGroup as error_group:
            if len(error_group.exceptions) == 1 and isinstance(
                error_group.exceptions[0], Exception
            ):
                raise error_group.exceptions[0] from error_group
            raise

    async def _track(
        self,
        name: str,
        states: AsyncIterable[api.models.MotionGroupState],
        pending_locations: dict[str, dict[datetime, float]],
    ) -> None:
        async for state in states:
            location = _extract_location(state)
            if location is None:
                continue
            timestamp = state.timestamp.replace(
                microsecond=(state.timestamp.microsecond // 1000) * 1000
            )
            pending = pending_locations[name]
            pending[timestamp] = location
            if len(pending) > _MAX_PENDING_DRIFT_SAMPLES:
                for stale in sorted(pending)[: len(pending) - _MAX_PENDING_DRIFT_SAMPLES]:
                    del pending[stale]
            self._compare_at(timestamp, pending_locations)

    def _compare_at(
        self, timestamp: datetime, pending_locations: dict[str, dict[datetime, float]]
    ) -> None:
        if not all(timestamp in pending for pending in pending_locations.values()):
            return
        locations = {name: pending[timestamp] for name, pending in pending_locations.items()}
        drift = max(locations.values()) - min(locations.values())
        if drift > self._max_drift:
            raise SyncDriftError(locations, self._max_drift)
        for pending in pending_locations.values():
            for stale in [t for t in pending if t <= timestamp]:
                del pending[stale]
