"""PolicyExecutor — runs one policy episode via PID-controlled jogging."""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Coroutine
import contextlib
from dataclasses import dataclass
from enum import StrEnum
import logging
import time
from typing import TYPE_CHECKING, Any

from policy._sdk import get_controller_id
from policy.estop import EstopMonitor, check_estop, check_sessions
from policy.io import IOStreamCache
from policy.pidjogging import PidJoggingSession
from policy.types import ActionChunk, EmergencyStopError, GuardStopError, MotionError, PidConfig

if TYPE_CHECKING:
    from policy.cameras import CameraSource
    from policy.policy_client import PolicyClient
    from policy.schema import PolicySchema
    from policy.types import SafetyGuard

# Type for bare async policy functions
_PolicyFn = Callable[[dict[str, Any]], Coroutine[Any, Any, dict[str, float] | ActionChunk]]

logger = logging.getLogger(__name__)


class Phase(StrEnum):
    """Executor lifecycle phase."""

    IDLE = "IDLE"
    CONNECTING = "CONNECTING"
    EXECUTING = "EXECUTING"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"


@dataclass
class ExecutorStatus:
    """Current executor state, queryable at any time."""

    phase: Phase = Phase.IDLE
    step: int = 0
    message: str = ""


@dataclass
class ExecutionResult:
    """Result of a policy execution run."""

    reason: str
    """Why execution ended: 'timeout' | 'stopped'"""

    steps: int = 0
    duration_s: float = 0.0
    last_state: dict[str, Any] | None = None
    """Last observed robot state (per motion group). Useful to know where the robot stopped."""


class PolicyExecutor:
    """Runs one policy episode: observe → query policy → send actions → repeat.

    The policy is a pure function: obs → actions. It never signals "done".
    Execution runs until timeout_s expires or stop() is called externally.
    """

    def __init__(
        self,
        schema: PolicySchema,
        policy: _PolicyFn | PolicyClient,
        *,
        config: PidConfig | None = None,
        safety_guards: list[SafetyGuard] | None = None,
        timeout_s: float = 0,
        inference_hz: float = 30,
        camera_max_age_s: float = 30.0,
    ) -> None:
        self._schema = schema
        self._motion_groups = schema.get_motion_groups()

        # Accept bare async function as policy (no wrapper needed)
        if callable(policy) and not hasattr(policy, "get_actions"):
            from policy.policy_client import CallbackPolicyClient  # noqa: PLC0415

            self._policy: PolicyClient = CallbackPolicyClient(policy)
        else:
            self._policy = policy  # type: ignore[assignment]

        self._config = config or PidConfig()
        self._safety_guards = safety_guards or []
        self._timeout_s = timeout_s
        self._inference_hz = inference_hz
        self._camera_max_age_s = camera_max_age_s

        self._sessions: dict[str, PidJoggingSession] = {}
        self._camera_sources: dict[str, CameraSource] = {}
        self._stop_event = asyncio.Event()
        self._last_obs: dict[str, Any] | None = None
        self._estop_monitor: EstopMonitor | None = None
        self._io_caches: list[IOStreamCache] = []
        self._io_tasks: set[asyncio.Task[None]] = set()

        self.status = ExecutorStatus()
        self.result: ExecutionResult | None = None

    @property
    def phase(self) -> Phase:
        return self.status.phase

    @property
    def mg_ids(self) -> list[str]:
        return [mg.id for mg in self._motion_groups]

    @property
    def last_observation(self) -> dict[str, Any] | None:
        """The most recent observation (robot state + camera images).

        Available during and after execution. Returns None before the first
        observation is collected.
        """
        return self._last_obs

    async def run(self) -> ExecutionResult:
        """Run execution, blocking until timeout or stop.

        Returns:
            ExecutionResult on normal termination (timeout, stopped).

        Raises:
            GuardStopError: A safety guard triggered.
            MotionError: Joint limit or self-collision detected.
            EmergencyStopError: E-stop or protective stop.
            RuntimeError: Connection lost or other error.
        """
        self._stop_event.clear()
        self.result = None
        self.status = ExecutorStatus(phase=Phase.CONNECTING, message="Connecting...")
        try:
            await self._run()
        except (GuardStopError, MotionError, EmergencyStopError):
            self.status = ExecutorStatus(phase=Phase.ERROR, step=self.status.step, message=str(self.result) if self.result else "")
            raise
        except Exception:
            self.status = ExecutorStatus(phase=Phase.ERROR, step=self.status.step, message="Unexpected error")
            raise
        finally:
            await self._cleanup()

        if self.result is None:
            self.result = ExecutionResult(reason="stopped", steps=self.status.step)
        return self.result

    def stop(self) -> None:
        """Signal the executor to stop. Non-blocking — run() will return shortly after."""
        self._stop_event.set()

    async def _cleanup(self) -> None:
        """Stop all sessions and close policy connection."""
        for session in self._sessions.values():
            with contextlib.suppress(GuardStopError, MotionError, EmergencyStopError, OSError, RuntimeError):
                await session.stop()

        # Wait for pending IO tasks
        if self._io_tasks:
            await asyncio.gather(*self._io_tasks, return_exceptions=True)
            self._io_tasks.clear()

        self._sessions.clear()

        with contextlib.suppress(OSError, RuntimeError):
            await self._policy.close()

        if self.status.phase not in (Phase.ERROR, Phase.COMPLETED):
            self.status = ExecutorStatus(phase=Phase.IDLE)

        if self.result is not None:
            logger.info(
                "PolicyExecutor stopped: reason=%s steps=%d duration=%.1fs",
                self.result.reason,
                self.result.steps,
                self.result.duration_s,
            )

    # -------------------------------------------------------------------------
    # Execution lifecycle
    # -------------------------------------------------------------------------

    async def _run(self) -> None:
        """Main execution: create sessions, loop observe→act, clean up.

        Exceptions (GuardStopError, MotionError, EmergencyStopError) propagate
        to run().
        """
        # Create and start sessions
        tcp_groups = self._schema.tcp_action_groups()
        for mg in self._motion_groups:
            if mg.id in tcp_groups:
                tcp_name = tcp_groups[mg.id] or self._schema.tcp
                tcp_config = self._make_tcp_config()
                self._sessions[mg.id] = PidJoggingSession(
                    motion_group=mg,
                    config=tcp_config,
                    tcp=tcp_name,
                    safety_guards=self._safety_guards,
                    mode="cartesian",
                )
            else:
                self._sessions[mg.id] = PidJoggingSession(
                    motion_group=mg,
                    config=self._config,
                    tcp=self._schema.tcp,
                    safety_guards=self._safety_guards,
                )

        image_sources = self._schema.image_sources
        if image_sources:
            logger.info("Connecting cameras...")
            await self._connect_cameras(image_sources)
            logger.info("All cameras ready")

        for session in self._sessions.values():
            await session.start()

        try:
            await self._start_io_streams()
            await self._policy.connect(self.mg_ids)
            self._estop_monitor = EstopMonitor(self._motion_groups)
            await self._estop_monitor.start()
            self.result = await self._execute()
            self.status.phase = Phase.COMPLETED
        finally:
            if self._estop_monitor is not None:
                await self._estop_monitor.stop()
                self._estop_monitor = None
            if self._camera_sources:
                await self._disconnect_cameras()
            await self._stop_io_streams()

    # -------------------------------------------------------------------------
    # Observe → act loop
    # -------------------------------------------------------------------------

    async def _execute(self) -> ExecutionResult:
        """Run the observe-act loop until termination.

        Raises GuardStopError, MotionError, EmergencyStopError directly.
        """
        step = 0
        start_time = time.monotonic()
        interval = 1.0 / self._inference_hz
        last_obs: dict[str, Any] | None = None

        self.status.phase = Phase.EXECUTING
        self.status.message = "Running policy..."

        while not self._stop_event.is_set():
            if self._timeout_s > 0 and (time.monotonic() - start_time) >= self._timeout_s:
                return _result("timeout", step, start_time, last_obs)

            # Observe
            robot_states = self._observe()
            if not robot_states:
                await asyncio.sleep(interval)
                continue
            images = self._read_cameras() if self._camera_sources else None
            self._last_obs = robot_states
            last_obs = robot_states

            # Query policy → send to robot
            action = await self._policy.get_actions(
                robot_states, self._schema, images, self._all_io_values or None,
            )
            action = self._apply_relative_mode(action, robot_states)
            self._send(action)
            step += 1
            self.status.step = step

            # Check failures — raises directly on error
            check_sessions(self._sessions)
            check_estop(self._estop_monitor)

            await asyncio.sleep(interval)

        return _result("stopped", step, start_time, last_obs)

    # -------------------------------------------------------------------------
    # Session operations (inlined from former PolicyRunner)
    # -------------------------------------------------------------------------

    def _observe(self) -> dict[str, Any]:
        """Get current state for all motion groups."""
        result: dict[str, Any] = {}
        for group_id, session in self._sessions.items():
            state = session.current_state
            if state is not None:
                result[group_id] = state
        return result

    def _send(self, chunk: ActionChunk) -> None:
        """Send an action chunk to the motion groups."""
        for group_id, steps in chunk.joints.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in chunk: %s", group_id)
                continue
            session.update_chunk(steps=steps, dt_ms=chunk.dt_ms)

        for group_id, raw_tcp_steps in chunk.tcp.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in TCP chunk: %s", group_id)
                continue
            session.update_chunk(steps=raw_tcp_steps, dt_ms=chunk.dt_ms)

        if chunk.ios:
            for group_id, ios in chunk.ios.items():
                session = self._sessions.get(group_id)
                if session is None:
                    continue
                task = asyncio.create_task(session.write_ios(ios))
                self._io_tasks.add(task)
                task.add_done_callback(self._io_tasks.discard)

    def _make_tcp_config(self) -> PidConfig:
        """Build a PidConfig with velocity limits suitable for Cartesian mode.

        Uses Nova's default TCP velocity limits (mm/s for translation,
        rad/s for orientation) combined with the user's PID gains.
        """
        from nova.types.motion_settings import DEFAULT_TCP_VELOCITY_LIMIT  # noqa: PLC0415

        tcp_vel = DEFAULT_TCP_VELOCITY_LIMIT  # 50 mm/s
        orient_vel = 1.5  # rad/s

        # If user provided explicit per-axis limits, use those
        base = self._config or PidConfig()
        if isinstance(base.velocity_limit, list) and len(base.velocity_limit) >= 6:  # noqa: PLR2004
            return base

        return PidConfig(
            velocity_limit=[tcp_vel, tcp_vel, tcp_vel, orient_vel, orient_vel, orient_vel],
            tolerance=base.tolerance,
            p_gain=base.p_gain,
            i_gain=base.i_gain,
            d_gain=base.d_gain,
        )

    def _apply_relative_mode(
        self, chunk: ActionChunk, states: dict[str, Any],
    ) -> ActionChunk:
        """Convert relative action targets to absolute.

        For ``mode='relative'``, each step in the chunk is an offset from the
        robot's state at inference time.
        """
        relative_mgs = self._schema.relative_motion_groups()
        if not relative_mgs:
            return chunk

        new_joints = dict(chunk.joints)
        new_tcp = dict(chunk.tcp)

        for mg_id in relative_mgs:
            state = states.get(mg_id)
            if state is None:
                continue

            # Relative joint actions
            if mg_id in new_joints:
                current = list(state.joints)
                new_joints[mg_id] = [
                    [c + d for c, d in zip(current, step, strict=True)]
                    for step in new_joints[mg_id]
                ]

            # Relative TCP actions
            if mg_id in new_tcp and hasattr(state, "pose") and state.pose is not None:
                current_tcp = (
                    list(state.pose.position) + list(state.pose.orientation)
                )
                new_tcp[mg_id] = [
                    [c + d for c, d in zip(current_tcp, step, strict=True)]
                    for step in new_tcp[mg_id]
                ]

        return ActionChunk(
            joints=new_joints, tcp=new_tcp, ios=chunk.ios, dt_ms=chunk.dt_ms,
        )

    # -------------------------------------------------------------------------
    # IO stream management
    # -------------------------------------------------------------------------

    async def _start_io_streams(self) -> None:
        """Open IO WebSocket streams and wire caches to sessions for guards."""
        io_by_ctrl = self._schema.io_keys_by_controller()
        if not io_by_ctrl:
            return

        started_ctrls: set[str] = set()
        for mg in self._motion_groups:
            ctrl_id = get_controller_id(mg)
            if ctrl_id in started_ctrls:
                continue
            io_keys = io_by_ctrl.get(ctrl_id)
            if not io_keys:
                continue
            started_ctrls.add(ctrl_id)
            cache = IOStreamCache(mg, io_keys)
            self._io_caches.append(cache)
            await cache.start()

        for cache in self._io_caches:
            session = self._sessions.get(cache.motion_group.id)
            if session is not None:
                session.set_io_values_ref(cache.values)

    async def _stop_io_streams(self) -> None:
        """Close all IO streams."""
        for cache in self._io_caches:
            await cache.stop()
        self._io_caches.clear()

    @property
    def _all_io_values(self) -> dict[str, object]:
        """Merged IO values from all caches."""
        merged: dict[str, object] = {}
        for cache in self._io_caches:
            merged.update(cache.values)
        return merged

    # -------------------------------------------------------------------------
    # Camera management
    # -------------------------------------------------------------------------

    async def _connect_cameras(self, sources: dict[str, CameraSource]) -> None:
        """Connect all camera sources from the schema."""
        tasks = []
        for key, source in sources.items():
            self._camera_sources[key] = source
            tasks.append(source.connect())
        if tasks:
            await asyncio.gather(*tasks)

    async def _disconnect_cameras(self) -> None:
        """Disconnect all camera sources."""
        tasks = [source.disconnect() for source in self._camera_sources.values()]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self._camera_sources.clear()

    def _read_cameras(self) -> dict[str, Any]:
        """Read one frame from each camera source."""
        return {
            key: source.read(max_age_s=self._camera_max_age_s)
            for key, source in self._camera_sources.items()
        }


def _result(
    reason: str, step: int, start_time: float, last_obs: dict[str, Any] | None = None,
) -> ExecutionResult:
    return ExecutionResult(
        reason=reason, steps=step, duration_s=time.monotonic() - start_time, last_state=last_obs,
    )
