"""E-stop and session failure detection."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from typing import TYPE_CHECKING

from pydantic import ValidationError

from nova import api
from novapolicy._sdk import get_api_gateway, get_cell, get_controller_id
from novapolicy.types import EmergencyStopError

if TYPE_CHECKING:
    from collections.abc import Mapping
    from typing import Any

    from nova.cell.motion_group import MotionGroup
    from nova.core.gateway import ApiGateway
    from novapolicy.jogging.waypoint_session import WaypointJoggingSession

logger = logging.getLogger(__name__)

# Safety states in which the robot is still operational. Anything else (e-stop,
# protective stop, fault, violation, ...) is treated as a safety stop. Mirrors
# nova's ProgramRunner estop check (single source of truth: SafetyStateType).
_OPERATIONAL_SAFETY_STATES = frozenset({
    api.models.SafetyStateType.SAFETY_STATE_NORMAL,
    api.models.SafetyStateType.SAFETY_STATE_REDUCED,
})
_RECONNECT_DELAY_S = 0.05
_BLIND_LIMIT_S = 2.0
"""How long the monitor may go without a controller state before it stops the run: an e-stop
it cannot see must not leave the robot moving on."""


# ---------------------------------------------------------------------------
# Shared failure checks (used by both jogger and executor)
# ---------------------------------------------------------------------------


def check_sessions(sessions: Mapping[Any, WaypointJoggingSession]) -> None:
    """Check all sessions for failures. Re-raises the original exception.

    Stop conditions are *not* failures — they end the run normally; see
    :func:`triggered_stop_condition`.

    Raises:
        MotionError: Joint limit, self-collision, or singularity.
        RuntimeError: Jogging connection lost or unknown failure.
    """
    for session in sessions.values():
        if not session.has_failed:
            continue
        exc = session.failure_exception
        if exc is not None:
            raise exc
        raise RuntimeError(session.failure_reason or "unknown session failure")


def triggered_stop_condition(sessions: Mapping[Any, WaypointJoggingSession]) -> str | None:
    """Return the name of the first stop condition that fired, else ``None``.

    A fired stop condition ends the run normally (no exception); the caller
    turns this into ``ExecutionResult`` with the name in its reason.
    """
    for session in sessions.values():
        name = session.stop_condition_triggered
        if name is not None:
            return name
    return None


def check_estop(monitor: EstopMonitor | None) -> None:
    """Raise if the e-stop monitor detected a safety stop.

    Raises:
        EmergencyStopError: Controller entered non-operational safety state.
    """
    if monitor is not None and monitor.error is not None:
        raise monitor.error


# ---------------------------------------------------------------------------
# E-stop monitor
# ---------------------------------------------------------------------------


class EstopMonitor:
    """Streams controller state and detects safety stops.

    Runs one background WebSocket per unique controller. Sets ``error``
    when any controller enters a non-operational safety state.
    """

    def __init__(self, motion_groups: list[MotionGroup]) -> None:
        self._motion_groups = motion_groups
        self._task: asyncio.Task[None] | None = None
        self.error: EmergencyStopError | None = None
        self._skipping: set[str] = set()  # controllers whose stream dropped
        self._last_state: dict[str, float] = {}  # monotonic time of each controller's last state

    async def start(self) -> None:
        self._task = asyncio.create_task(self._run(), name="estop-monitor")

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None

    async def _run(self) -> None:
        seen: set[str] = set()
        tasks: list[asyncio.Task[None]] = []
        for mg in self._motion_groups:
            ctrl_id = get_controller_id(mg)
            if ctrl_id not in seen:
                seen.add(ctrl_id)
                tasks.append(
                    asyncio.create_task(
                        self._watch(ctrl_id, get_api_gateway(mg)),
                        name=f"estop-watch-{ctrl_id}",
                    )
                )
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _watch(self, controller_id: str, api_client: ApiGateway) -> None:
        # A state the SDK model cannot parse (an enum value it does not know yet, e.g. mode
        # MODE_UNDEFINED while a client takes control), a dropped connection or a stream that
        # simply ends all close the generator. Reopen it: the rest of the run must stay
        # monitored. If no state arrives for _BLIND_LIMIT_S, stop the run instead.
        self._last_state[controller_id] = time.monotonic()
        while True:
            try:
                await self._watch_stream(controller_id, api_client)
                if self.error is not None:
                    return
                reason = "the state stream ended"
            except ValidationError as e:
                reason = f"unparseable controller state ({str(e).splitlines()[0]})"
            except (OSError, RuntimeError) as e:
                reason = f"connection error ({e})"
            if controller_id not in self._skipping:  # once per streak, not per retry
                self._skipping.add(controller_id)
                logger.warning("E-stop monitor for %s: %s, reconnecting", controller_id, reason)
            blind_s = time.monotonic() - self._last_state[controller_id]
            if blind_s > _BLIND_LIMIT_S:
                logger.error(
                    "E-stop monitor for %s has read no controller state for %.1f s; stopping",
                    controller_id,
                    blind_s,
                )
                self.error = EmergencyStopError(controller_id, "E-STOP MONITOR BLIND")
                return
            await asyncio.sleep(_RECONNECT_DELAY_S)

    async def _watch_stream(self, controller_id: str, api_client: ApiGateway) -> None:
        cell = get_cell(self._motion_groups[0])
        stream = None
        try:
            stream = api_client.controller_api.stream_robot_controller_state(
                cell=cell,
                controller=controller_id,
                response_rate=100,
            )
            async for state in stream:
                self._last_state[controller_id] = time.monotonic()
                if controller_id in self._skipping:
                    self._skipping.discard(controller_id)
                    logger.info(
                        "E-stop monitor for %s reads controller states again", controller_id
                    )
                safety = getattr(state, "safety_state", None)
                if safety is None:
                    continue
                if safety not in _OPERATIONAL_SAFETY_STATES:
                    label = getattr(safety, "value", str(safety))
                    logger.error("E-stop detected on %s: %s", controller_id, label)
                    self.error = EmergencyStopError(controller_id, label)
                    return
        finally:
            if stream is not None:
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await stream.aclose()
