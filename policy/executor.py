"""PolicyExecutor — runs one policy episode via waypoint jogging."""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Coroutine
import contextlib
from dataclasses import dataclass
from enum import StrEnum
import logging
import time
from typing import TYPE_CHECKING, Any

from policy.cameras.manager import CameraManager
from policy.chunking import (
    apply_relative_mode,
    chunk_duration_s,
    placement,
    trim_chunk,
)
from policy.estop import EstopMonitor, check_estop, check_sessions, triggered_stop_condition
from policy.io import IOStreamManager
from policy.jogging.waypoint_session import WaypointJoggingSession
from policy.types import (
    ActionChunk,
    EmergencyStopError,
    MotionError,
    StopContext,
    WaypointConfig,
)

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from policy.policy_client import PolicyClient
    from policy.rerun import PolicyRerunLogger
    from policy.schema import PolicySchema
    from policy.types import StopCondition

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
    """Why execution ended: 'timeout' | 'stopped' | 'stop condition: <name>'"""

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
        stop_conditions: list[StopCondition] | None = None,
        timeout_s: float = 0,
        camera_max_age_s: float = 30.0,
        motion: WaypointConfig | None = None,
        policy_rate_hz: float = -1,
        n_action_steps: int = 0,
    ) -> None:
        """Create a policy executor.

        Args:
            schema: Observation/action schema defining robot topology.
            policy: Async callable or PolicyClient that maps observations to actions.
            stop_conditions: Optional checks run each tick; one returning ``True``
                stops the run normally (its name appears in ``result.reason``).
            timeout_s: Maximum execution duration in seconds. 0 = no timeout.
            camera_max_age_s: Maximum allowed age of a camera frame before raising.
            motion: Waypoint jogging configuration. Defaults to WaypointConfig().
            policy_rate_hz: Controls timing between policy calls.
                -1 (default): Wait for each chunk to finish executing before
                    calling the policy again. Use for sequential policies that
                    do not support RTC.
                0: Call the policy as fast as possible (no sleep between calls).
                    Each new chunk immediately replaces the previous one.
                >0: Call the policy at this fixed rate (Hz). Each new chunk
                    replaces the previous one mid-execution. Use for RTC
                    policies (e.g. 20 Hz with GR00T RTC enabled).
            n_action_steps: Number of steps from each action chunk to execute.
                0 (default): Execute all steps returned by the policy.
                >0: Trim to first N steps (receding horizon). Later steps
                have higher prediction uncertainty and are discarded.
                The policy still predicts the full action_horizon (e.g. 16)
                which is used internally for RTC warm-starting.
        """
        self._schema = schema
        self._motion_groups = schema.get_motion_groups()

        # Accept bare async function as policy (no wrapper needed)
        if callable(policy) and not hasattr(policy, "get_actions"):
            from policy.policy_client import CallbackPolicyClient  # noqa: PLC0415

            self._policy: PolicyClient = CallbackPolicyClient(policy)
        else:
            self._policy = policy

        self._motion: WaypointConfig = motion or WaypointConfig()
        self._stop_conditions = stop_conditions or []
        self._timeout_s = timeout_s
        self._policy_rate_hz = policy_rate_hz
        self._n_action_steps_cfg = n_action_steps

        # RTC produces overlapping chunks whose seam backdate is only honored in
        # overlapping placement (policy_rate_hz >= 0). In wait-for-chunk mode
        # (-1) the backdate would be silently dropped and seams would not
        # connect — reject the contradictory combination up front.
        if self._policy_rate_hz < 0 and getattr(self._policy, "rtc", None) is not None:
            msg = (
                f"RTC is enabled on the policy but policy_rate_hz={self._policy_rate_hz} "
                "(wait-for-chunk). RTC requires overlapping placement: set "
                "policy_rate_hz to 0 (ASAP) or a fixed rate (>0 Hz)."
            )
            raise ValueError(msg)

        self._sessions: dict[str, WaypointJoggingSession] = {}
        self._cameras = CameraManager(camera_max_age_s)
        self._stop_event = asyncio.Event()
        self._last_obs: dict[str, Any] | None = None
        self._estop_monitor: EstopMonitor | None = None
        self._io_streams = IOStreamManager(self._motion_groups, schema.io_keys_by_controller())
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

        Public lifecycle shell: resets state, drives the :class:`Phase` state
        machine (mapping errors to ``Phase.ERROR``), and guarantees
        :meth:`_cleanup` runs. The actual orchestration lives in
        :meth:`_run_episode`.

        Returns:
            ExecutionResult on normal termination (timeout, stopped, or a
            triggered stop condition — its name is in ``result.reason``).

        Raises:
            MotionError: Joint limit or self-collision detected.
            EmergencyStopError: E-stop or protective stop.
            RuntimeError: Connection lost or other error.
        """
        self._stop_event.clear()
        self.result = None
        self.status = ExecutorStatus(phase=Phase.CONNECTING, message="Connecting...")
        try:
            await self._run_episode()
        except (MotionError, EmergencyStopError):
            self.status = ExecutorStatus(
                phase=Phase.ERROR,
                step=self.status.step,
                message=str(self.result) if self.result else "",
            )
            raise
        except Exception:
            self.status = ExecutorStatus(
                phase=Phase.ERROR, step=self.status.step, message="Unexpected error"
            )
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
        """Outer teardown — always runs, even if ``_run_episode`` setup throws.

        Owns the resources allocated *before* and *around* ``_run_episode``:
        the jogging sessions, pending IO write tasks, and the policy
        connection. Also performs the final status reset and result logging.
        """
        for session in self._sessions.values():
            with contextlib.suppress(MotionError, EmergencyStopError, OSError, RuntimeError):
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

    async def _run_episode(self) -> None:
        """Orchestrate one episode: create sessions, loop observe→act, tear down.

        The ``finally`` here owns only the resources started *inside* this
        method (rerun logger, e-stop monitor, cameras, IO streams). Sessions
        and the policy connection are released by :meth:`_cleanup`, which
        ``run`` guarantees runs even if this method throws during setup.

        Exceptions (MotionError, EmergencyStopError) propagate
        to :meth:`run`.
        """
        # Create and start sessions
        for mg in self._motion_groups:
            self._sessions[mg.id] = self._create_session(mg)

        image_sources = self._schema.image_sources
        if image_sources:
            logger.info("Connecting cameras...")
            await self._cameras.connect(image_sources)
            logger.info("All cameras ready")

        for session in self._sessions.values():
            await session.start()

        try:
            await self._io_streams.start()
            self._io_streams.wire_to_sessions(self._sessions)
            await self._policy.connect(self.mg_ids)
            await self._policy.validate_schema(self._schema)
            self._estop_monitor = EstopMonitor(self._motion_groups)
            await self._estop_monitor.start()
            await self._init_rerun()
            if self._rerun is not None:
                self._rerun.start_streaming(self._sessions)
            # Wait for sessions to be ready (server acknowledged init)
            for session in self._sessions.values():
                await session.wait_ready()
            self.result = await self._execute()
            self._log_completion()
            self.status.phase = Phase.COMPLETED
        finally:
            # Release only what this method started; sessions + policy are
            # handled by _cleanup() in run()'s finally.
            if self._rerun is not None:
                await self._rerun.stop_streaming()
            if self._estop_monitor is not None:
                await self._estop_monitor.stop()
                self._estop_monitor = None
            if self._cameras.active:
                await self._cameras.disconnect()
            await self._io_streams.stop()

    # -------------------------------------------------------------------------
    # Observe → act loop
    # -------------------------------------------------------------------------

    async def _execute(self) -> ExecutionResult:
        """Run the observe-act loop until termination.

        Raises MotionError, EmergencyStopError directly.
        """
        step = 0
        start_time = time.monotonic()
        last_obs: dict[str, Any] | None = None

        self.status.phase = Phase.EXECUTING
        self.status.message = "Running policy..."

        # Determine timing mode
        # -1 = wait for chunk, 0 = as fast as possible, >0 = fixed rate
        fixed_rate_period = 1.0 / self._policy_rate_hz if self._policy_rate_hz > 0 else 0.0

        while not self._stop_event.is_set():
            # Always yield once per iteration so stop()/other tasks make progress
            # even when the policy and sleeps below complete synchronously
            # (e.g. dt_ms=0 with policy_rate_hz<=0, or a local in-process policy).
            await asyncio.sleep(0)

            if self._timeout_s > 0 and (time.monotonic() - start_time) >= self._timeout_s:
                return _result("timeout", step, start_time, last_obs)

            tick_start = time.monotonic()

            # Observe
            robot_states = self._observe()
            if not robot_states:
                await asyncio.sleep(0.01)  # retry shortly
                continue
            images = self._cameras.read() if self._cameras.active else None
            self._last_obs = robot_states
            last_obs = robot_states

            # Rerun: log observation
            if self._rerun is not None:
                self._rerun.log_observation(robot_states, step)
                if images:
                    self._rerun.log_images(images)

            # Query policy → send to robot
            action = await self._policy.get_actions(
                robot_states,
                self._schema,
                images,
                self._all_io_values or None,
            )
            action = self._apply_relative_mode(action, robot_states)
            stopped_by = self._check_stop_conditions_pre_send(action, robot_states)
            if stopped_by is not None:
                return _result(f"stop condition: {stopped_by}", step, start_time, last_obs)

            # Rerun: log full action chunk (includes discarded tail for visualization)
            if self._rerun is not None:
                self._rerun.log_action_chunk(action, step, n_action_steps=self._n_action_steps)

            trimmed = trim_chunk(action, self._n_action_steps)
            self._send(trimmed)
            step += 1
            self.status.step = step

            # Check failures — raises directly on error
            check_sessions(self._sessions)
            check_estop(self._estop_monitor)

            # A stop condition firing on a session tick ends the run normally.
            stopped_by = triggered_stop_condition(self._sessions)
            if stopped_by is not None:
                return _result(f"stop condition: {stopped_by}", step, start_time, last_obs)

            await self._wait_after_send(trimmed, tick_start, start_time, fixed_rate_period)

        return _result("stopped", step, start_time, last_obs)

    async def _wait_after_send(
        self,
        trimmed: ActionChunk,
        tick_start: float,
        start_time: float,
        fixed_rate_period: float,
    ) -> None:
        """Pace the loop after a chunk is sent, per the configured timing mode."""
        if self._policy_rate_hz < 0:
            # Wait for the full chunk duration before next inference.
            chunk_s = chunk_duration_s(trimmed)
            if chunk_s > 0:
                await self._sleep_interruptible(chunk_s, start_time)
        elif self._policy_rate_hz > 0:
            # Fixed-rate: sleep for the remainder of the period.
            # Each new chunk replaces the previous one mid-execution.
            elapsed = time.monotonic() - tick_start
            sleep_time = fixed_rate_period - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
        # else: policy_rate_hz == 0 → no sleep, call as fast as possible

    async def _sleep_interruptible(self, duration_s: float, exec_start: float) -> None:
        """Sleep for duration_s but wake early on stop or timeout."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            if self._stop_event.is_set():
                return
            if self._timeout_s > 0 and (time.monotonic() - exec_start) >= self._timeout_s:
                return
            check_sessions(self._sessions)
            check_estop(self._estop_monitor)
            remaining = end - time.monotonic()
            await asyncio.sleep(min(0.05, max(0, remaining)))

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

    def _check_stop_conditions_pre_send(
        self, chunk: ActionChunk, robot_states: dict[str, RobotState]
    ) -> str | None:
        """Evaluate stop conditions against the intended action before sending.

        Gives conditions visibility into what the policy intends to do (target
        positions + IO writes) so they can stop before execution. Returns the
        name of the first condition that fired (``True``), or ``None``.
        """
        if not self._stop_conditions:
            return None

        for group_id in {*chunk.joints, *chunk.tcp}:
            state = robot_states.get(group_id)
            if state is None:
                continue

            target_joints = chunk.joints.get(group_id) or chunk.tcp.get(group_id)
            target_ios = chunk.ios.get(group_id) if chunk.ios else None

            ctx = StopContext(
                state=state,
                prev_state=None,
                dt=0.0,
                motion_group_id=group_id,
                io_values=self._all_io_values,
                target_joints=target_joints,
                target_ios=target_ios,
            )
            for condition in self._stop_conditions:
                if condition(ctx):
                    return getattr(condition, "__name__", repr(condition))
        return None

    def _send(self, chunk: ActionChunk) -> None:
        """Send an action chunk to the motion groups.

        Placement (relative vs. absolute, with optional RTC seam backdate) is
        decided per session by :func:`policy.chunking.placement`; the "now"
        component is resolved at yield time inside the session. For overlapping
        mode to keep the robot moving, ``(horizon - seam_backdate_steps) *
        dt_ms`` must exceed the inference latency (increase dt_ms or cut latency
        otherwise).
        """
        backdate_ms = int(chunk.seam_backdate_steps * chunk.dt_ms) if chunk.dt_ms > 0 else 0
        place = placement(chunk, policy_rate_hz=self._policy_rate_hz, backdate_ms=backdate_ms)
        for group_id, steps in chunk.joints.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in chunk: %s", group_id)
                continue
            session.update_chunk(
                steps=steps,
                dt_ms=chunk.dt_ms,
                first_timestamp_ms=place.first_timestamp_ms,
                overlapping=place.overlapping,
                backdate_ms=place.backdate_ms,
            )

        for group_id, raw_tcp_steps in chunk.tcp.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in TCP chunk: %s", group_id)
                continue
            session.update_chunk(
                steps=raw_tcp_steps,
                dt_ms=chunk.dt_ms,
                first_timestamp_ms=place.first_timestamp_ms,
                overlapping=place.overlapping,
                backdate_ms=place.backdate_ms,
            )

        if chunk.ios:
            for group_id, ios in chunk.ios.items():
                session = self._sessions.get(group_id)
                if session is None:
                    continue
                task = asyncio.create_task(session.write_ios(ios))
                self._io_tasks.add(task)
                task.add_done_callback(self._io_tasks.discard)

    def _create_session(self, mg: MotionGroup) -> WaypointJoggingSession:
        """Create a waypoint jogging session for a motion group."""
        tcp_groups = self._schema.tcp_action_groups()
        mode = "cartesian" if mg.id in tcp_groups else "joint"
        tcp = self._schema.tcp
        if mode == "cartesian":
            tcp = tcp_groups.get(mg.id) or self._schema.tcp

        return WaypointJoggingSession(
            motion_group=mg,
            config=self._motion,
            tcp=tcp,
            stop_conditions=self._stop_conditions,
            mode=mode,
        )

    @property
    def _n_action_steps(self) -> int:
        """Number of action steps to execute from each chunk (0 = all)."""
        return self._n_action_steps_cfg

    def _apply_relative_mode(self, chunk: ActionChunk, states: dict[str, Any]) -> ActionChunk:
        """Convert relative (delta) action targets to absolute (see chunking)."""
        return apply_relative_mode(chunk, states, self._schema.relative_motion_groups())

    @property
    def _all_io_values(self) -> dict[str, object]:
        """Merged IO values across all controller streams."""
        return self._io_streams.all_values

    # -------------------------------------------------------------------------
    # Rerun visualization (lazy, zero-cost when viewer not active)
    # -------------------------------------------------------------------------

    async def _init_rerun(self) -> None:
        """Initialize Rerun logger if a viewer is active."""
        from policy.rerun import _is_rerun_active  # noqa: PLC0415

        if not _is_rerun_active():
            return

        from policy.rerun import PolicyRerunLogger  # noqa: PLC0415

        self._rerun = PolicyRerunLogger(self._motion_groups, camera_names=self._cameras.names)
        await self._rerun.initialize()

    def _log_completion(self) -> None:
        """Log execution result to Rerun."""
        if self._rerun is not None and self.result is not None:
            self._rerun.log_completion(
                self.result.reason,
                self.result.steps,
                self.result.duration_s,
            )


def _result(
    reason: str,
    step: int,
    start_time: float,
    last_obs: dict[str, Any] | None = None,
) -> ExecutionResult:
    return ExecutionResult(
        reason=reason,
        steps=step,
        duration_s=time.monotonic() - start_time,
        last_state=last_obs,
    )
