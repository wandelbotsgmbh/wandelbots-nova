"""PolicyRunner — orchestrates PID-controlled jogging for policy execution."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING

from policy._sdk import get_api_gateway
from policy.pid_jogging_session import PidJoggingSession
from policy.types import PolicyRunnerConfig

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from policy.types import ActionChunk, SafetyGuard

logger = logging.getLogger(__name__)


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
        tcp: str = "",
        safety_guards: list[SafetyGuard] | None = None,
    ) -> None:
        """Initialize the PolicyRunner.

        Args:
            motion_groups: Motion groups to control.
            config: PID configuration. Defaults to training-time values.
            tcp: TCP name for jogging and state streaming. Empty = robot's default.
            safety_guards: Optional safety callbacks. All must return True each tick.
        """
        self._config = config or PolicyRunnerConfig()
        self._safety_guards = safety_guards or []
        self._sessions: dict[str, PidJoggingSession] = {}

        self._io_tasks: set[asyncio.Task[None]] = set()

        for mg in motion_groups:
            session = PidJoggingSession(
                motion_group=mg, config=self._config, tcp=tcp, safety_guards=self._safety_guards
            )
            self._sessions[mg.id] = session

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
                self._io_tasks.add(task)
                task.add_done_callback(self._io_tasks.discard)

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

    def check_health(self) -> None:
        """Check all sessions for failures. Raises on first error found.

        Raises:
            GuardStopError: A safety guard triggered.
            MotionError: Joint limit, self-collision, or singularity.
            RuntimeError: Jogging connection lost or unknown failure.
        """
        from policy.estop import check_sessions  # noqa: PLC0415

        check_sessions(self._sessions)

    # -------------------------------------------------------------------------
    # Async context manager
    # -------------------------------------------------------------------------

    async def __aenter__(self) -> PolicyRunner:
        """Start all jogging sessions."""
        for session in self._sessions.values():
            await session.start()
        return self

    async def __aexit__(
        self, exc_type: type[BaseException] | None, exc_val: BaseException | None, exc_tb: object
    ) -> bool:
        """Stop all sessions and clean up resources."""
        await self.stop()

        # Wait for pending IO tasks to finish
        if self._io_tasks:
            await asyncio.gather(*self._io_tasks, return_exceptions=True)
            self._io_tasks.clear()

        # Close the SDK's aiohttp sessions used by jogging/state-stream WebSockets
        closed: set[int] = set()
        for session in self._sessions.values():
            api_client = get_api_gateway(session._motion_group)
            client_id = id(api_client)
            if client_id not in closed:
                closed.add(client_id)
                with contextlib.suppress(Exception):
                    await api_client.close()
        return False
