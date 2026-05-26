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
from policy.jogging.session import JoggingSession
from policy.jogging.waypoint_session import WaypointJoggingSession, is_waypoint_jogging_available
from policy.types import (
    ActionChunk,
    EmergencyStopError,
    GuardState,
    GuardStopError,
    MotionConfig,
    MotionError,
    WaypointConfig,
)

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from policy.cameras import CameraSource
    from policy.policy_client import PolicyClient
    from policy.rerun import PolicyRerunLogger
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
        safety_guards: list[SafetyGuard] | None = None,
        timeout_s: float = 0,
        camera_max_age_s: float = 30.0,
        motion: MotionConfig | WaypointConfig | None = None,
        policy_rate_hz: float = 0,
    ) -> None:
        """Create a policy executor.

        Args:
            schema: Observation/action schema defining robot topology.
            policy: Async callable or PolicyClient that maps observations to actions.
            safety_guards: Optional list of guard functions checked each jogging tick.
            timeout_s: Maximum execution duration in seconds. 0 = no timeout.
            camera_max_age_s: Maximum allowed age of a camera frame before raising.
            motion: Motion configuration. MotionConfig (client-side velocity profile)
                or WaypointConfig (server-side waypoint jogging, experimental).
                Defaults to WaypointConfig() if available, else MotionConfig().
            policy_rate_hz: Fixed rate (Hz) at which the policy is called.
                When set (>0), the executor calls the policy at this rate and
                each new chunk replaces the previous one mid-execution.
                When 0 (default), the executor waits for the current chunk to
                finish before calling the policy again (sequential mode).
        """
        self._schema = schema
        self._motion_groups = schema.get_motion_groups()

        # Accept bare async function as policy (no wrapper needed)
        if callable(policy) and not hasattr(policy, "get_actions"):
            from policy.policy_client import CallbackPolicyClient  # noqa: PLC0415

            self._policy: PolicyClient = CallbackPolicyClient(policy)
        else:
            self._policy = policy  # type: ignore[assignment]

        self._motion: MotionConfig | WaypointConfig = motion or self._default_motion_config()
        self._safety_guards = safety_guards or []
        self._timeout_s = timeout_s
        self._camera_max_age_s = camera_max_age_s
        # Use policy_rate_hz from WaypointConfig if not explicitly overridden
        if policy_rate_hz == 0 and isinstance(self._motion, WaypointConfig):
            self._policy_rate_hz = self._motion.policy_rate_hz
        else:
            self._policy_rate_hz = policy_rate_hz

        self._sessions: dict[str, JoggingSession] = {}
        self._camera_sources: dict[str, CameraSource] = {}
        self._stop_event = asyncio.Event()
        self._last_obs: dict[str, Any] | None = None
        self._estop_monitor: EstopMonitor | None = None
        self._io_caches: list[IOStreamCache] = []
        self._io_tasks: set[asyncio.Task[None]] = set()
        self._rerun: PolicyRerunLogger | None = None

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
        for mg in self._motion_groups:
            self._sessions[mg.id] = self._create_session(mg)

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
            await self._policy.validate_schema(self._schema)
            self._estop_monitor = EstopMonitor(self._motion_groups)
            await self._estop_monitor.start()
            await self._init_rerun()
            if self._rerun is not None:
                self._rerun.start_streaming(self._sessions)
            self.result = await self._execute()
            self._log_completion()
            self.status.phase = Phase.COMPLETED
        finally:
            if self._rerun is not None:
                await self._rerun.stop_streaming()
            if self._estop_monitor is not None:
                await self._estop_monitor.stop()
                self._estop_monitor = None
            if self._camera_sources:
                await self._disconnect_cameras()
            await self._stop_io_streams()

    # -------------------------------------------------------------------------
    # Observe → act loop
    # -------------------------------------------------------------------------

    async def _execute(self) -> ExecutionResult:  # noqa: C901
        """Run the observe-act loop until termination.

        Raises GuardStopError, MotionError, EmergencyStopError directly.
        """
        step = 0
        start_time = time.monotonic()
        last_obs: dict[str, Any] | None = None

        self.status.phase = Phase.EXECUTING
        self.status.message = "Running policy..."

        # Fixed-rate mode: call policy at policy_rate_hz instead of
        # waiting for chunk_done between iterations.
        fixed_rate_period = (
            1.0 / self._policy_rate_hz if self._policy_rate_hz > 0 else 0.0
        )

        while not self._stop_event.is_set():
            if self._timeout_s > 0 and (time.monotonic() - start_time) >= self._timeout_s:
                return _result("timeout", step, start_time, last_obs)

            tick_start = time.monotonic()

            # Observe
            robot_states = self._observe()
            if not robot_states:
                await asyncio.sleep(0.01)  # retry shortly
                continue
            images = self._read_cameras() if self._camera_sources else None
            self._last_obs = robot_states
            last_obs = robot_states

            # Rerun: log observation
            if self._rerun is not None:
                self._rerun.log_observation(robot_states, step)
                if images:
                    self._rerun.log_images(images)

            # Query policy → send to robot
            action = await self._policy.get_actions(
                robot_states, self._schema, images, self._all_io_values or None,
            )
            action = self._apply_relative_mode(action, robot_states)
            self._check_guards_pre_send(action, robot_states)

            # Rerun: log full action chunk (includes discarded tail for visualization)
            if self._rerun is not None:
                self._rerun.log_action_chunk(action, step, n_action_steps=self._n_action_steps)

            self._send(self._trim_chunk(action))
            step += 1
            self.status.step = step

            # Check failures — raises directly on error
            check_sessions(self._sessions)
            check_estop(self._estop_monitor)

            if fixed_rate_period > 0:
                # Waypoint mode: sleep for the remainder of the period.
                # Each new chunk replaces the previous one mid-execution.
                elapsed = time.monotonic() - tick_start
                sleep_time = fixed_rate_period - elapsed
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
            else:
                # Velocity-profile mode: wait until chunk is fully executed.
                while not all(s.chunk_done for s in self._sessions.values()):
                    if self._stop_event.is_set():
                        break
                    if self._timeout_s > 0 and (time.monotonic() - start_time) >= self._timeout_s:
                        break
                    check_sessions(self._sessions)
                    check_estop(self._estop_monitor)
                    await asyncio.sleep(0.01)

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

    def _check_guards_pre_send(self, chunk: ActionChunk, robot_states: dict[str, RobotState]) -> None:
        """Run safety guards with the intended action before sending.

        This gives guards visibility into what the policy intends to do
        (target positions + IO writes) so they can reject before execution.
        """
        if not self._safety_guards:
            return

        for group_id in {*chunk.joints, *chunk.tcp}:
            state = robot_states.get(group_id)
            if state is None:
                continue

            target_joints = chunk.joints.get(group_id) or chunk.tcp.get(group_id)
            target_ios = chunk.ios.get(group_id) if chunk.ios else None

            ctx = GuardState(
                state=state,
                prev_state=None,
                dt=0.0,
                motion_group_id=group_id,
                io_values=self._all_io_values,
                target_joints=target_joints,
                target_ios=target_ios,
            )
            for guard in self._safety_guards:
                if not guard(ctx):
                    guard_name = getattr(guard, "__name__", repr(guard))
                    raise GuardStopError(group_id, guard_name)

    def _send(self, chunk: ActionChunk) -> None:
        """Send an action chunk to the motion groups."""
        for group_id, steps in chunk.joints.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in chunk: %s", group_id)
                continue
            # Log chunk magnitude for debugging
            if len(steps) > 1 and logger.isEnabledFor(logging.DEBUG):
                span = sum(
                    abs(steps[-1][j] - steps[0][j]) for j in range(len(steps[0]))
                )
                per_step = span / (len(steps) - 1) if len(steps) > 1 else 0
                ff_vel = per_step / (chunk.dt_ms / 1000.0) if chunk.dt_ms > 0 else 0
                logger.debug(
                    "Chunk %s: %d steps, span=%.5f rad, per_step=%.6f rad, "
                    "ff_vel=%.4f rad/s, dt_ms=%.1f",
                    group_id, len(steps), span, per_step, ff_vel, chunk.dt_ms,
                )
            session.update_chunk(steps=steps, dt_ms=chunk.dt_ms, start_time_ms=chunk.start_time_ms)

        for group_id, raw_tcp_steps in chunk.tcp.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in TCP chunk: %s", group_id)
                continue
            session.update_chunk(steps=raw_tcp_steps, dt_ms=chunk.dt_ms, start_time_ms=chunk.start_time_ms)

        if chunk.ios:
            for group_id, ios in chunk.ios.items():
                session = self._sessions.get(group_id)
                if session is None:
                    continue
                task = asyncio.create_task(session.write_ios(ios))
                self._io_tasks.add(task)
                task.add_done_callback(self._io_tasks.discard)

    def _create_session(self, mg: MotionGroup) -> JoggingSession:
        """Create a jogging session for a motion group."""
        tcp_groups = self._schema.tcp_action_groups()
        mode = "cartesian" if mg.id in tcp_groups else "joint"
        tcp = tcp_groups.get(mg.id) or self._schema.tcp if mode == "cartesian" else self._schema.tcp

        # WaypointConfig → use native waypoint jogging
        if isinstance(self._motion, WaypointConfig):
            _compat_config = MotionConfig(state_rate_ms=self._motion.state_rate_ms)
            return WaypointJoggingSession(  # type: ignore[return-value]
                motion_group=mg,
                config=_compat_config,
                tcp=tcp,
                safety_guards=self._safety_guards,
                mode=mode,
            )

        # MotionConfig → client-side velocity profile
        config = self._motion
        if mode == "cartesian" and not isinstance(config.velocity_limit, list):
            tcp_vel = 50.0  # mm/s
            orient_vel = 1.5  # rad/s
            config = MotionConfig(
                velocity_limit=[tcp_vel, tcp_vel, tcp_vel, orient_vel, orient_vel, orient_vel],
                ramp_steps=config.ramp_steps,
                p_gain=config.p_gain,
                state_rate_ms=config.state_rate_ms,
                n_action_steps=config.n_action_steps,
            )

        return JoggingSession(
            motion_group=mg,
            config=config,
            tcp=tcp,
            safety_guards=self._safety_guards,
            mode=mode,
        )

    @property
    def _n_action_steps(self) -> int:
        """Number of action steps to execute from each chunk (0 = all)."""
        return self._motion.n_action_steps

    @staticmethod
    def _default_motion_config() -> MotionConfig | WaypointConfig:
        """Pick the best available motion mode.

        Uses WaypointConfig (native server-side) if the NOVA SDK supports it,
        otherwise falls back to MotionConfig (client-side velocity profile).
        """
        if is_waypoint_jogging_available():
            return WaypointConfig()
        return MotionConfig()

    def _trim_chunk(self, chunk: ActionChunk) -> ActionChunk:
        """Trim action chunk to n_action_steps if configured.

        Returns the original chunk unmodified if n_action_steps is 0 (execute all)
        or if the chunk is already shorter than n_action_steps.
        """
        n = self._n_action_steps
        if n <= 0:
            return chunk

        trimmed_joints = {
            mg_id: steps[:n] for mg_id, steps in chunk.joints.items()
        } if chunk.joints else {}

        trimmed_tcp = {
            mg_id: steps[:n] for mg_id, steps in chunk.tcp.items()
        } if chunk.tcp else {}

        return ActionChunk(
            joints=trimmed_joints, tcp=trimmed_tcp, ios=chunk.ios, dt_ms=chunk.dt_ms,
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

            # Relative joint actions: each step is a delta from the previous,
            # so step[i] target = current + sum(deltas[0..i])
            if mg_id in new_joints:
                running = list(state.joints)
                abs_steps = []
                for step in new_joints[mg_id]:
                    running = [r + d for r, d in zip(running, step, strict=True)]
                    abs_steps.append(list(running))
                new_joints[mg_id] = abs_steps

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

    # -------------------------------------------------------------------------
    # Rerun visualization (lazy, zero-cost when viewer not active)
    # -------------------------------------------------------------------------

    async def _init_rerun(self) -> None:
        """Initialize Rerun logger if a viewer is active."""
        from policy.rerun import _is_rerun_active  # noqa: PLC0415

        if not _is_rerun_active():
            return

        from policy.rerun import PolicyRerunLogger  # noqa: PLC0415

        camera_names = list(self._camera_sources.keys()) if self._camera_sources else []
        self._rerun = PolicyRerunLogger(self._motion_groups, camera_names=camera_names)
        await self._rerun.initialize()

    def _log_completion(self) -> None:
        """Log execution result to Rerun."""
        if self._rerun is not None and self.result is not None:
            self._rerun.log_completion(
                self.result.reason, self.result.steps, self.result.duration_s,
            )

def _result(
    reason: str, step: int, start_time: float, last_obs: dict[str, Any] | None = None,
) -> ExecutionResult:
    return ExecutionResult(
        reason=reason, steps=step, duration_s=time.monotonic() - start_time, last_state=last_obs,
    )
