"""The session lifecycle the executor and the joggers share: start, check, stop."""

from __future__ import annotations

import asyncio
import contextlib
from typing import TYPE_CHECKING

from nova.actions import jnt
from novapolicy.estop import check_estop, check_sessions, triggered_stop_condition
from novapolicy.types import EmergencyStopError, MotionError

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping
    from typing import Any

    from nova.cell.motion_group import MotionGroup
    from novapolicy.estop import EstopMonitor
    from novapolicy.jogging.waypoint_session import WaypointJoggingSession
    from novapolicy.rerun import PolicyRerunLogger

_STOP_ERRORS = (asyncio.CancelledError, MotionError, EmergencyStopError, OSError, RuntimeError)
"""What stopping a session may raise without failing the teardown: the robot is stopping anyway."""


async def move_to_start_joint_position(start: Mapping[MotionGroup, list[float]] | None) -> None:
    """PTP every motion group to its start joints, in parallel.

    The cell's collision setups stay in force: the start move is planned like any other move.
    """

    async def ptp(mg: MotionGroup, joints: list[float]) -> None:
        tcp = await mg.active_tcp_name() or (await mg.tcp_names())[0]
        setup = await mg.get_setup(tcp)
        target = tuple(joints)
        trajectory = await mg.plan([jnt(target)], tcp, motion_group_setup=setup)
        await mg.execute(trajectory, tcp, actions=[jnt(target)])

    if start:
        await asyncio.gather(*(ptp(mg, joints) for mg, joints in start.items()))


class SessionLifecycle:
    """Starts a run's sessions with an e-stop monitor and, when a viewer is active, Rerun;
    checks them for faults and stop conditions; stops them again."""

    def __init__(
        self, sessions: Mapping[str, WaypointJoggingSession], motion_groups: list[MotionGroup]
    ) -> None:
        self._sessions = sessions
        self._motion_groups = motion_groups
        self._estop: EstopMonitor | None = None
        self.rerun: PolicyRerunLogger | None = None

    async def start_sessions(self) -> None:
        for session in self._sessions.values():
            await session.start()

    async def start_monitoring(
        self,
        estop: EstopMonitor,
        *,
        camera_names: list[str] | None = None,
        tcp: str = "",
        image_reader: Callable[[], dict[str, Any]] | None = None,
    ) -> PolicyRerunLogger | None:
        """Start ``estop`` and Rerun; returns the Rerun logger, ``None`` without a viewer."""
        self._estop = estop
        await estop.start()
        from novapolicy.rerun import _is_rerun_active  # ruff: ignore[import-outside-top-level]

        if not _is_rerun_active():
            return None
        from novapolicy.rerun import PolicyRerunLogger  # ruff: ignore[import-outside-top-level]

        self.rerun = PolicyRerunLogger(
            self._motion_groups,
            camera_names=camera_names,
            use_tcp_offset_for_joint_actions=True,
            tcp=tcp,
        )
        await self.rerun.initialize()
        self.rerun.start_streaming(self._sessions, image_reader=image_reader)
        return self.rerun

    async def wait_ready(self) -> None:
        """Wait until the server acknowledged every session, so motion starts on the first
        waypoint."""
        for session in self._sessions.values():
            await session.wait_ready()

    @property
    def stop_condition(self) -> str | None:
        """Name of the first stop condition that fired, or ``None``."""
        return triggered_stop_condition(self._sessions)

    def check(self) -> str | None:
        """Raise a session fault or an e-stop; else return :attr:`stop_condition`."""
        check_sessions(self._sessions)
        check_estop(self._estop)
        return self.stop_condition

    async def close(self, *, drain: bool) -> None:
        """Stop Rerun, the e-stop monitor and every session; with ``drain``, let the accepted
        waypoints run out first (a fault or e-stop wants the robot stopped now)."""
        if drain:
            with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                _ = await asyncio.gather(
                    *(session.drain() for session in self._sessions.values()),
                    return_exceptions=True,
                )
        try:
            if self.rerun is not None:
                await self.rerun.stop_streaming()
                self.rerun = None
            if self._estop is not None:
                await self._estop.stop()
                self._estop = None
        finally:
            # In parallel: each stop waits for its server to settle.
            results = await asyncio.gather(
                *(session.stop() for session in self._sessions.values()), return_exceptions=True
            )
        for result in results:
            if isinstance(result, BaseException) and not isinstance(result, _STOP_ERRORS):
                raise result
