"""Measured joint velocities, read from the raw state stream as JSON (ADR 0016).

The typed ``MotionGroupState`` drops ``joint_velocity``; do not move this onto the typed stream.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import TYPE_CHECKING, Protocol, runtime_checkable

import websockets
from websockets.exceptions import WebSocketException

from novapolicy._sdk import get_cell, get_controller_id, get_ws_target

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup

logger = logging.getLogger(__name__)

_STREAM_PATH = (
    "/cells/{cell}/controllers/{controller}"
    "/motion-groups/{motion_group}/state-stream?response_rate={rate}"
)
"""Relative to the gateway's configured base, which already carries ``/api/v2``."""

DEFAULT_RESPONSE_RATE_MS = 20
"""Requested stream period in ms; at or below the controller cycle it streams every cycle."""

DEFAULT_STALE_AFTER_S = 0.2
"""Seconds a sample stays usable; past it the caller falls back to finite differences."""


_RETRY_MIN_S = 1.0
_RETRY_MAX_S = 10.0
"""Reconnect backoff of a lost measured-velocity stream."""


@runtime_checkable
class VelocitySource(Protocol):
    """What a policy client needs from a source of measured velocities."""

    async def start(self) -> None:
        """Begin reading measured velocities."""

    async def stop(self) -> None:
        """Stop reading and forget the latest values."""

    def velocities(self, motion_group_id: str) -> tuple[float, ...] | None:
        """Latest measured velocities, or ``None`` when absent or stale."""


class StateStreamVelocities:
    """One read-only raw state-stream reader per motion group, latest value wins."""

    def __init__(
        self,
        motion_groups: list[MotionGroup],
        *,
        response_rate_ms: int = DEFAULT_RESPONSE_RATE_MS,
        stale_after_s: float = DEFAULT_STALE_AFTER_S,
    ) -> None:
        self._motion_groups = motion_groups
        self._response_rate_ms = response_rate_ms
        self._stale_after_s = stale_after_s
        self._latest: dict[str, tuple[float, tuple[float, ...]]] = {}
        self._tasks: list[asyncio.Task[None]] = []

    async def start(self) -> None:
        if self._tasks:
            return
        for mg in self._motion_groups:
            self._tasks.append(asyncio.create_task(self._read(mg)))
        logger.info(
            "Measured-velocity streams open on %s at %dms",
            [mg.id for mg in self._motion_groups],
            self._response_rate_ms,
        )

    async def stop(self) -> None:
        tasks, self._tasks = self._tasks, []
        for task in tasks:
            task.cancel()
        # Each reader ends in its cancellation; nothing a reader raised matters now.
        await asyncio.gather(*tasks, return_exceptions=True)
        self._latest.clear()

    def velocities(self, motion_group_id: str) -> tuple[float, ...] | None:
        entry = self._latest.get(motion_group_id)
        if entry is None:
            return None
        moment, values = entry
        if time.monotonic() - moment > self._stale_after_s:
            return None
        return values

    @property
    def sampled(self) -> dict[str, int]:
        """Motion groups that have produced at least one sample, and their width."""
        return {mg_id: len(values) for mg_id, (_, values) in self._latest.items()}

    async def _read(self, mg: MotionGroup) -> None:
        path = _STREAM_PATH.format(
            cell=get_cell(mg),
            controller=get_controller_id(mg),
            motion_group=mg.id,
            rate=self._response_rate_ms,
        )
        url, headers = get_ws_target(mg, path)
        retry_s = _RETRY_MIN_S
        while True:
            try:
                await self._read_until_closed(mg, url, headers)
                retry_s = _RETRY_MIN_S  # it was connected; the next drop starts over
                reason = "stream closed"
            except asyncio.CancelledError:
                raise
            except (OSError, RuntimeError, ValueError, WebSocketException) as exc:
                reason = str(exc) or type(exc).__name__
            # Until the stream is back the caller falls back to finite differences.
            logger.warning(
                "Measured-velocity stream lost for %s (%s); reconnecting in %.0f s",
                mg.id,
                reason,
                retry_s,
            )
            await asyncio.sleep(retry_s)
            retry_s = min(retry_s * 2, _RETRY_MAX_S)

    async def _read_until_closed(self, mg: MotionGroup, url: str, headers: dict[str, str]) -> None:
        async with websockets.connect(
            url, open_timeout=10, additional_headers=headers, max_queue=None
        ) as socket:
            async for raw in socket:
                velocities = _parse_velocities(raw)
                if velocities is not None:
                    self._latest[mg.id] = (time.monotonic(), velocities)


def _parse_velocities(raw: str | bytes) -> tuple[float, ...] | None:
    """``result.joint_velocity`` from one stream frame, if it carries one."""
    try:
        result = json.loads(raw).get("result") or {}
    except (TypeError, ValueError):
        return None
    values = result.get("joint_velocity")
    if not isinstance(values, list) or not values:
        return None
    try:
        return tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
