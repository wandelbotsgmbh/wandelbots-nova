"""Measured joint velocities, read from the raw state stream.

A policy trained on measured joint velocities needs them at inference, and they
cannot be had through the typed SDK path: ``MotionGroupState`` has no
``joint_velocity`` field and pydantic drops unknown ones, so the field the
controller does put on the wire is discarded before any caller sees it. The
SDK's ``RobotState`` carries only pose, TCP and joints for the same reason.

Finite-differencing joint positions at the request cadence is the obvious
substitute and a poor one. Measured against the controller's own values on a
7-dof arm at a 20ms cadence, the difference was twice the true magnitude, three
times as noisy sample to sample, and wrong by up to 4.8 rad/s on individual
samples. A residual policy fed that produces correspondingly large and erratic
residuals, so this reads the stream as raw JSON instead and keeps the finite
difference only as a fallback for when no measurement is available.

Do not "simplify" this onto the typed stream. That is the bug it exists for.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import time
from typing import TYPE_CHECKING, Protocol, runtime_checkable

import websockets

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
"""Requested stream period. At or below the controller cycle the server streams
every cycle anyway, which measured ~8ms on a live instance at this value."""

DEFAULT_STALE_AFTER_S = 0.2
"""How long a sample stays usable. Past this the stream is treated as absent and
the caller falls back, rather than feeding the policy a stale velocity."""


@runtime_checkable
class VelocitySource(Protocol):
    """What a policy client needs from a source of measured velocities."""

    async def start(self) -> None: ...

    async def stop(self) -> None: ...

    def velocities(self, motion_group_id: str) -> tuple[float, ...] | None:
        """Latest measured velocities, or ``None`` when absent or stale."""
        ...


class StateStreamVelocities:
    """One raw state-stream reader per motion group, latest value wins.

    Read-only: it opens its own stream and commands nothing, so it can run
    alongside a jogging session's own state stream and anything else.
    """

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
        for task in tasks:
            with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                await task
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
        try:  # ruff: ignore[too-many-nested-blocks]
            async with websockets.connect(
                url, open_timeout=10, additional_headers=headers, max_queue=None
            ) as socket:
                async for raw in socket:
                    velocities = _parse_velocities(raw)
                    if velocities is not None:
                        self._latest[mg.id] = (time.monotonic(), velocities)
        except asyncio.CancelledError:
            raise
        except (OSError, RuntimeError, ValueError) as exc:
            # A missing measurement is survivable: the caller falls back to
            # finite differencing, which is worse but not fatal.
            logger.warning("Measured-velocity stream lost for %s: %s", mg.id, exc)


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
