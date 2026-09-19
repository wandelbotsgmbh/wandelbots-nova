"""E-stop and session failure detection."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING

from pydantic import ValidationError

from nova import api
from novapolicy._sdk import get_api_gateway, get_cell, get_controller_id
from novapolicy.types import EmergencyStopError

if TYPE_CHECKING:
    from collections.abc import Mapping
    from typing import Any

    from nova.cell.motion_group import MotionGroup
    from novapolicy.jogging.waypoint_session import WaypointJoggingSession

logger = logging.getLogger(__name__)

# Safety states in which the robot is still operational. Anything else (e-stop,
# protective stop, fault, violation, ...) is treated as a safety stop. Mirrors
# nova's ProgramRunner estop check (single source of truth: SafetyStateType).
_OPERATIONAL_SAFETY_STATES = frozenset(
    {
        api.models.SafetyStateType.SAFETY_STATE_NORMAL,
        api.models.SafetyStateType.SAFETY_STATE_REDUCED,
    }
)


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

    async def _watch(self, controller_id: str, api_client: object) -> None:
        cell = get_cell(self._motion_groups[0])
        stream = None
        try:
            stream = api_client.controller_api.stream_robot_controller_state(
                cell=cell,
                controller=controller_id,
                response_rate=100,
            )
            async for state in stream:
                safety = getattr(state, "safety_state", None)
                if safety is None:
                    continue
                if safety not in _OPERATIONAL_SAFETY_STATES:
                    label = getattr(safety, "value", str(safety))
                    logger.error("E-stop detected on %s: %s", controller_id, label)
                    self.error = EmergencyStopError(controller_id, label)
                    return
        except asyncio.CancelledError:
            raise
        except (OSError, RuntimeError) as e:
            logger.warning("E-stop monitor for %s stopped: %s", controller_id, e)
        except ValidationError as e:
            # Unknown/unparseable state kinds (e.g. KIND_UNKNOWN on waypoint
            # jogging feature branches). Non-fatal, but make it visible: the
            # monitor stops watching this controller after this point.
            logger.warning(
                "E-stop monitor for %s stopped (unparseable controller state): %s",
                controller_id,
                e,
            )
        finally:
            if stream is not None:
                with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                    await stream.aclose()
