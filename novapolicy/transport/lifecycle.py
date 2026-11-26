"""The session lifecycle the executor and the joggers share: build, start, check, stop."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from nova.actions import jnt
from novapolicy.transport.estop import EstopMonitor
from novapolicy.transport.session import ActionChunkSession
from novapolicy.types import EmergencyStopError, MotionError

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping, Sequence
    from typing import Any

    from nova.cell.motion_group import MotionGroup
    from novapolicy.rerun import PolicyRerunLogger
    from novapolicy.types import ActionChunkConfig, JoggingMode, StopCondition

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
    """Builds a run's sessions, one per motion group; starts them with an e-stop monitor and,
    when a viewer is active, Rerun; checks them for faults and stop conditions; stops them."""

    def __init__(
        self,
        *,
        config: ActionChunkConfig,
        stop_conditions: list[StopCondition] | None = None,
        pipeline: Sequence[object] | None = None,
    ) -> None:
        self._config = config
        self._stop_conditions = stop_conditions
        self._pipeline = pipeline
        self.sessions: dict[str, ActionChunkSession] = {}
        """The sessions by motion group ID, filled by :meth:`build`."""
        self._motion_groups: list[MotionGroup] = []
        self._estop: EstopMonitor | None = None
        self.rerun: PolicyRerunLogger | None = None

    def build(
        self, groups: Mapping[MotionGroup, tuple[JoggingMode, str]]
    ) -> dict[str, ActionChunkSession]:
        """A fresh session per motion group, with its jogging mode and TCP, in place of the
        last run's; returns :attr:`sessions`."""
        self._motion_groups = list(groups)
        self.sessions.clear()
        for mg, (mode, tcp) in groups.items():
            self.sessions[mg.id] = ActionChunkSession(
                motion_group=mg,
                config=self._config,
                tcp=tcp,
                mode=mode,
                stop_conditions=self._stop_conditions,
                pipeline=self._pipeline,
            )
        return self.sessions

    async def start_sessions(self) -> None:
        for session in self.sessions.values():
            await session.start()

    async def start_monitoring(
        self,
        *,
        camera_names: list[str] | None = None,
        tcp: str = "",
        image_reader: Callable[[], dict[str, Any]] | None = None,
    ) -> PolicyRerunLogger | None:
        """Start an e-stop monitor and Rerun; returns the Rerun logger, ``None`` without a
        viewer."""
        self._estop = EstopMonitor(self._motion_groups)
        await self._estop.start()
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
        self.rerun.start_streaming(self.sessions, image_reader=image_reader)
        return self.rerun

    async def wait_ready(self) -> None:
        """Wait until the server acknowledged every session, so motion starts on the first
        waypoint."""
        for session in self.sessions.values():
            await session.wait_ready()

    @property
    def stop_condition(self) -> str | None:
        """Name of the first stop condition that fired, or ``None``; a normal stop."""
        fired = (session.stop_condition_triggered for session in self.sessions.values())
        return next((name for name in fired if name is not None), None)

    def check(self) -> str | None:
        """Raise an e-stop (``EmergencyStopError``) or else the first session failure
        (``MotionError``: joint limit, self-collision, singularity; ``RuntimeError``: connection
        lost or unknown); else return :attr:`stop_condition`. The e-stop comes first: it also
        ends the sessions' streams, and it is the cause to report."""
        if self._estop is not None and self._estop.error is not None:
            raise self._estop.error
        for session in self.sessions.values():
            if session.has_failed:
                raise session.failure_exception or RuntimeError(
                    session.failure_reason or "unknown session failure"
                )
        return self.stop_condition

    async def drain(self) -> None:
        """Let every session's accepted waypoints run out (a session that cannot returns early);
        a failure meanwhile is left for :meth:`check`."""
        _ = await asyncio.gather(
            *(session.drain() for session in self.sessions.values()), return_exceptions=True
        )

    async def close(self, *, drain: bool) -> None:
        """Stop Rerun, the e-stop monitor and every session; with ``drain``, let the accepted
        waypoints run out first (a fault or e-stop wants the robot stopped now). A cancel that
        arrives while draining still stops everything, then reaches the caller."""
        cancelled = False
        if drain:
            try:
                _ = await asyncio.gather(
                    *(session.drain() for session in self.sessions.values()),
                    return_exceptions=True,
                )
            except asyncio.CancelledError:
                cancelled = True
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
                *(session.stop() for session in self.sessions.values()), return_exceptions=True
            )
        for result in results:
            if isinstance(result, BaseException) and not isinstance(result, _STOP_ERRORS):
                raise result
        if cancelled:
            raise asyncio.CancelledError
