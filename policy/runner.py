"""PolicyRunner — orchestrates PID-controlled jogging for policy execution."""

from __future__ import annotations

import asyncio
import logging
import signal
from typing import TYPE_CHECKING

from policy.pid_jogging_session import PidJoggingSession
from policy.types import PolicyRunnerConfig

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from policy.types import ActionChunk, SafetyGuard

logger = logging.getLogger(__name__)

# Background IO tasks — stored to prevent GC
_io_tasks: set[asyncio.Task[None]] = set()


class PolicyRunner:
    """Orchestrates PID-controlled jogging for one or more motion groups.

    Feeds action chunks (joint targets + IO) from a policy into the robot
    via velocity-controlled jogging.

    Usage:
        runner = PolicyRunner(motion_groups=[mg])
        async with runner:
            await runner.send(action_chunk)
            obs = await runner.observe()
    """

    def __init__(
        self,
        motion_groups: list[MotionGroup],
        config: PolicyRunnerConfig | None = None,
        *,
        safety_guards: list[SafetyGuard] | None = None,
    ) -> None:
        """Initialize the PolicyRunner.

        Args:
            motion_groups: Motion groups to control.
            config: PID configuration. Defaults to training-time values.
            safety_guards: Optional safety callbacks. All must return True each tick.
        """
        self._config = config or PolicyRunnerConfig()
        self._safety_guards = safety_guards or []
        self._sessions: dict[str, PidJoggingSession] = {}

        for mg in motion_groups:
            session = PidJoggingSession(
                motion_group=mg, config=self._config, safety_guards=self._safety_guards
            )
            self._sessions[mg.id] = session

        self._original_sigterm: signal.Handlers | None = None
        self._original_sigint: signal.Handlers | None = None

    @property
    def motion_group_ids(self) -> list[str]:
        """IDs of all managed motion groups."""
        return list(self._sessions.keys())

    def set_io_values_ref(self, motion_group_id: str, io_values: dict[str, object]) -> None:
        """Attach a shared IO values dict to a session (for guards to read)."""
        session = self._sessions.get(motion_group_id)
        if session is not None:
            session._io_values = io_values

    async def send(self, chunk: ActionChunk) -> None:
        """Send an action chunk to the motion groups.

        Updates the active target for each group and fires IO commands.

        Args:
            chunk: The action chunk containing joints and optional IO.

        Raises:
            GuardStopError: If a safety guard triggers during the next tick.
        """
        for group_id, steps in chunk.joints.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in chunk: %s", group_id)
                continue
            session.update_chunk(steps=steps, dt_ms=chunk.dt_ms)

        # Fire IO commands (non-blocking — don't wait for HTTP response)
        if chunk.ios:
            for group_id, ios in chunk.ios.items():
                session = self._sessions.get(group_id)
                if session is None:
                    continue
                task = asyncio.create_task(session.write_ios(ios))
                _io_tasks.add(task)
                task.add_done_callback(_io_tasks.discard)

    async def observe(self) -> dict[str, RobotState]:
        """Get current state for all motion groups.

        Returns:
            Dict of motion group ID → RobotState (pose + joints + tcp).
        """
        result: dict[str, RobotState] = {}
        for group_id, session in self._sessions.items():
            state = session.current_state
            if state is not None:
                result[group_id] = state
        return result

    async def stop(self) -> None:
        """Stop all jogging sessions (sends zero velocity)."""
        for session in self._sessions.values():
            await session.stop()

    # -------------------------------------------------------------------------
    # Async context manager
    # -------------------------------------------------------------------------

    async def __aenter__(self) -> PolicyRunner:
        """Start all jogging sessions and register signal handlers."""
        self._register_signal_handlers()
        for session in self._sessions.values():
            await session.start()
        return self

    async def __aexit__(
        self, exc_type: type[BaseException] | None, exc_val: BaseException | None, exc_tb: object
    ) -> bool:
        """Stop all sessions and deregister signal handlers."""
        await self.stop()
        self._deregister_signal_handlers()
        return False

    # -------------------------------------------------------------------------
    # Signal handling for graceful shutdown
    # -------------------------------------------------------------------------

    def _register_signal_handlers(self) -> None:
        """Register SIGTERM/SIGINT handlers to ensure zero velocity on kill."""
        try:
            loop = asyncio.get_running_loop()
            self._original_sigterm = signal.getsignal(signal.SIGTERM)
            self._original_sigint = signal.getsignal(signal.SIGINT)
            loop.add_signal_handler(signal.SIGTERM, self._emergency_stop)
            loop.add_signal_handler(signal.SIGINT, self._emergency_stop)
        except (NotImplementedError, RuntimeError):
            # Signal handlers not available (e.g. Windows, non-main thread)
            logger.debug("Signal handlers not available, skipping registration")

    def _deregister_signal_handlers(self) -> None:
        """Restore original signal handlers."""
        try:
            loop = asyncio.get_running_loop()
            loop.remove_signal_handler(signal.SIGTERM)
            loop.remove_signal_handler(signal.SIGINT)
        except (NotImplementedError, RuntimeError):
            pass

    def _emergency_stop(self) -> None:
        """Emergency stop: cancel all sessions immediately."""
        logger.warning("Emergency stop triggered (signal received)")
        for session in self._sessions.values():
            session._running = False
            session._steps = []
            session._pid.reset()
