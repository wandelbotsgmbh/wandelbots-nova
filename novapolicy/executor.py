"""PolicyExecutor — runs one policy episode via waypoint jogging."""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass
from enum import StrEnum
import logging
import time
from typing import TYPE_CHECKING, Any

from nova import api
from nova.actions import jnt
from novapolicy.cameras.manager import CameraManager
from novapolicy.chunking import (
    ConnectedActionChunk,
    apply_relative_mode,
    connect_action_chunk,
    interpolate_action_chunk_ramps,
    placement,
    trim_chunk,
)
from novapolicy.debug import ExecutionTrajectoryTrace
from novapolicy.estop import EstopMonitor, check_estop, check_sessions, triggered_stop_condition
from novapolicy.io import IOStreamManager
from novapolicy.jogging.waypoint_session import WaypointJoggingSession
from novapolicy.policy_client import PolicyClient
from novapolicy.types import (
    ActionChunk,
    EmergencyStopError,
    MotionError,
    StopContext,
    WaypointConfig,
)

if TYPE_CHECKING:
    from pathlib import Path

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.rerun import PolicyRerunLogger
    from novapolicy.schema import PolicySchema
    from novapolicy.types import StopCondition

logger = logging.getLogger(__name__)

_MIN_RAMP_INTERPOLATION_STEPS = 2
_ASYNC_SEAM_STEPS = 4
_ASYNC_REPLACEMENT_LEAD_STEPS = 1
_DEFAULT_WAYPOINT_CONFIG = WaypointConfig()


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
class _PolicyTimeline:
    """Immutable action anchor in the raw NOVA controller timeline."""

    action_timestep: int
    policy_dt_ms: float
    server_timestamp_ms: float


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
        policy: PolicyClient,
        *,
        stop_conditions: list[StopCondition] | None = None,
        timeout_s: float = 0,
        camera_max_age_s: float = 30.0,
        motion: WaypointConfig = _DEFAULT_WAYPOINT_CONFIG,
        policy_rate_hz: float = -1,
        n_action_steps: int = 0,
        interpolate_chunk_ramps: bool = False,
        ramp_interpolation_steps: int = 3,
        start_joint_position: dict[MotionGroup, list[float]] | None = None,
        trajectory_trace_path: str | Path | None = None,
    ) -> None:
        """Create a policy executor.

        Args:
            schema: Observation/action schema defining robot topology.
            policy: PolicyClient that maps observations to actions.
            stop_conditions: Optional checks run each tick; one returning ``True``
                stops the run normally (its name appears in ``result.reason``).
            timeout_s: Maximum execution duration in seconds. 0 = no timeout.
            camera_max_age_s: Maximum allowed age of a camera frame before raising.
            motion: Waypoint jogging configuration. Defaults to WaypointConfig().
            policy_rate_hz: Controls timing between policy calls.
                -1 (default): Bridge from the observed state when needed, wait
                    for the exact chunk deadline and NOVA standstill, then call
                    the policy again. Use for settled sequential inference.
                0: Call the policy as fast as possible (no sleep between calls).
                    Each new chunk immediately replaces the previous one.
                >0: Call the policy at this fixed rate (Hz). Each new chunk
                    replaces the previous one mid-execution. Use for continuous
                    asynchronous inference or model-side RTC.
            n_action_steps: Number of steps from each action chunk to execute.
                0 (default): Execute all steps returned by the policy.
                >0: Trim to first N steps (receding horizon). Later steps
                have higher prediction uncertainty and are discarded.
                The policy still predicts the full action_horizon (e.g. 16)
                which remains available to asynchronous policy clients.
            interpolate_chunk_ramps: Subdivide the first and final motion
                intervals with acceleration and braking interpolation. Intended
                for settled execution, where every submitted request ends at standstill.
            ramp_interpolation_steps: Number of same-``dt_ms`` intervals replacing
                each endpoint interval when interpolation is enabled. Must be >= 2.
            start_joint_position: Optional mapping of motion group to joint pose
                to PTP-move to before starting waypoint jogging.
            trajectory_trace_path: Optional JSON output containing every policy
                chunk, exact waypoint request, and controller-timestamped state
                sample for offline trajectory reconstruction.
        """
        self._schema = schema
        self._motion_groups = schema.get_motion_groups()

        if not isinstance(policy, PolicyClient):
            msg = "policy must be a PolicyClient; wrap callbacks with CallbackPolicyClient"
            raise TypeError(msg)
        self._policy = policy

        self._motion = motion
        self._start_joint_position = start_joint_position
        self._trajectory_trace = (
            ExecutionTrajectoryTrace(trajectory_trace_path)
            if trajectory_trace_path is not None
            else None
        )
        if self._trajectory_trace is not None:
            self._trajectory_trace.enable_policy_client(self._policy)
        self._stop_conditions = stop_conditions or []
        self._timeout_s = timeout_s
        self._policy_rate_hz = policy_rate_hz
        self._n_action_steps_cfg = n_action_steps
        self._interpolate_chunk_ramps = interpolate_chunk_ramps
        self._ramp_interpolation_steps = ramp_interpolation_steps

        # Endpoint ramps add time to a request that deliberately ends at
        # standstill. They do not apply to continuously replaced chunks.
        if self._interpolate_chunk_ramps and self._policy_rate_hz >= 0:
            msg = "interpolate_chunk_ramps requires settled wait-for-chunk mode"
            raise ValueError(msg)
        if self._ramp_interpolation_steps < _MIN_RAMP_INTERPOLATION_STEPS:
            msg = "ramp_interpolation_steps must be at least 2"
            raise ValueError(msg)

        if self._policy_rate_hz < 0 and self._policy.rtc is not None:
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
        self._policy_timelines: dict[str, _PolicyTimeline] = {}
        self._rerun: PolicyRerunLogger | None = None
        self._logged_first_images = False

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
        self._policy_timelines.clear()
        if self._trajectory_trace is not None:
            self._trajectory_trace.clear()
        self.result = None
        self.status = ExecutorStatus(phase=Phase.CONNECTING, message="Connecting...")
        try:
            await self._run_episode()
        except (MotionError, EmergencyStopError) as exc:
            self.status = ExecutorStatus(
                phase=Phase.ERROR,
                step=self.status.step,
                message=str(exc),
            )
            raise
        except Exception as exc:
            self.status = ExecutorStatus(
                phase=Phase.ERROR,
                step=self.status.step,
                message=f"{type(exc).__name__}: {exc}",
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

        self._write_trajectory_trace()

        # Wait for pending IO tasks
        if self._io_tasks:
            await asyncio.gather(*self._io_tasks, return_exceptions=True)
            self._io_tasks.clear()

        self._sessions.clear()

        with contextlib.suppress(OSError, RuntimeError):
            await self._policy.close()

        if self.status.phase not in {Phase.ERROR, Phase.COMPLETED}:
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

    async def _move_to_start_joint_position(self) -> None:
        """PTP move configured motion groups before starting waypoint jogging."""

        async def ptp(mg: MotionGroup, joints: list[float]) -> None:
            tcp = await mg.active_tcp_name() or (await mg.tcp_names())[0]
            setup = await mg.get_setup(tcp)
            setup.collision_setups = api.models.CollisionSetups({})
            target = tuple(joints)
            trajectory = await mg.plan([jnt(target)], tcp, motion_group_setup=setup)
            await mg.execute(trajectory, tcp, actions=[jnt(target)])

        start_positions = self._start_joint_position
        if start_positions is None:
            return
        await asyncio.gather(*(ptp(mg, joints) for mg, joints in start_positions.items()))

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

        if self._start_joint_position:
            await self._move_to_start_joint_position()

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
                image_reader = self._cameras.read_latest_frames if self._cameras.active else None
                self._rerun.start_streaming(self._sessions, image_reader=image_reader)
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
        start_time: float | None = None
        last_obs: dict[str, Any] | None = None

        self.status.message = "Preparing policy..."

        # Determine timing mode
        # -1 = wait for chunk, 0 = as fast as possible, >0 = fixed rate
        fixed_rate_period = 1.0 / self._policy_rate_hz if self._policy_rate_hz > 0 else 0.0

        while not self._stop_event.is_set():
            # Always yield once per iteration so stop()/other tasks make progress
            # even when the policy and sleeps below complete synchronously
            # (e.g. dt_ms=0 with policy_rate_hz<=0, or a local in-process policy).
            await asyncio.sleep(0)

            termination = self._termination_result(step, start_time, last_obs)
            if termination is not None:
                return termination

            tick_start = time.monotonic()

            # Observe
            robot_states = self._observe()
            if not robot_states:
                await asyncio.sleep(0.01)  # retry shortly
                continue
            observation_time = time.monotonic()
            images = self._cameras.read() if self._cameras.active else None
            self._log_first_image_shapes(images)
            self._last_obs = robot_states
            last_obs = robot_states

            self._log_policy_observation(robot_states, images, step)

            if start_time is None:
                await self._prepare_policy(robot_states, images)
                start_time = time.monotonic()
                tick_start = start_time
                self.status.phase = Phase.EXECUTING
                self.status.message = "Running policy..."

            # Query policy → send to robot
            action = await self._get_policy_actions(robot_states, images, step=step + 1)
            self._log_observation_to_first_waypoint(
                action,
                robot_states,
                step=step + 1,
                observation_time=observation_time,
            )
            stopped_by = self._check_stop_conditions_pre_send(action, robot_states)
            if stopped_by is not None:
                return _result(f"stop condition: {stopped_by}", step, start_time, last_obs)

            trimmed = trim_chunk(action, self._n_action_steps)
            connected = self._connected_motion(trimmed, robot_states)
            if connected is not None:
                boundary_termination = await self._send_connected_policy_chunk(
                    connected,
                    action,
                    step=step,
                    start_time=start_time,
                    last_obs=last_obs,
                )
                if boundary_termination is not None:
                    return boundary_termination
            else:
                sent_chunk = self._interpolate_motion(trimmed)
                await self._schema.run_computed_actions(action)
                self._log_policy_action(action, step)
                self._send(sent_chunk)
            step += 1
            self.status.step = step

            # Check failures — raises directly on error
            check_sessions(self._sessions)
            check_estop(self._estop_monitor)

            # A stop condition firing on a session tick ends the run normally.
            stopped_by = triggered_stop_condition(self._sessions)
            if stopped_by is not None:
                return _result(f"stop condition: {stopped_by}", step, start_time, last_obs)

            await self._wait_after_send(tick_start, start_time, fixed_rate_period)

        return _result("stopped", step, start_time or time.monotonic(), last_obs)

    async def _get_policy_actions(
        self,
        robot_states: dict[str, RobotState],
        images: dict[str, Any] | None,
        *,
        step: int,
    ) -> ActionChunk:
        """Synchronize a queue, request its action, and retain exact trace data."""
        self._synchronize_policy_action_timestep()
        action = await self._policy.get_actions(
            robot_states,
            self._schema,
            images,
            self._all_io_values or None,
        )
        action = self._apply_relative_mode(action, robot_states)
        self._record_policy_chunk(action, robot_states, step=step)
        return action

    def _synchronize_policy_action_timestep(self) -> None:
        """Advance a queue policy to the first safely replaceable NOVA action.

        Queue consumption advances to the immediate successor of the action
        currently due. The async policy client prepends its preceding published
        action, yielding a retained predecessor/current/two-successor seam even
        if inference consumes one full policy interval. Queue progress is read from the
        latest acknowledged raw NOVA controller timestamp, not extrapolated
        from local policy-loop ticks or a measured transport-delay constant.
        """
        if not self._policy_timelines:
            return

        replaceable_timesteps: list[int] = []
        for group_id, timeline in self._policy_timelines.items():
            session = self._sessions.get(group_id)
            if session is None or timeline.policy_dt_ms <= 0:
                continue
            elapsed_ms = max(
                0.0,
                session.last_server_timestamp_ms - timeline.server_timestamp_ms,
            )
            currently_due_timestep = timeline.action_timestep + int(
                elapsed_ms // timeline.policy_dt_ms
            )
            replaceable_timesteps.append(currently_due_timestep + _ASYNC_REPLACEMENT_LEAD_STEPS)
        if replaceable_timesteps:
            self._policy.synchronize_action_timestep(max(replaceable_timesteps))

    def _connected_motion(
        self,
        chunk: ActionChunk,
        robot_states: dict[str, RobotState],
    ) -> ConnectedActionChunk | None:
        continuous_bridge = self._policy.requires_first_waypoint_bridge
        needs_continuous_bridge = continuous_bridge and (
            chunk.action_timestep < 0 or not self._policy_timelines
        )
        if self._policy_rate_hz >= 0 and not needs_continuous_bridge:
            return None
        connected = connect_action_chunk(
            chunk,
            robot_states,
            always_anchor=continuous_bridge,
        )
        if connected is None:
            return None
        return self._interpolate_connected_motion(connected)

    def _interpolate_motion(self, chunk: ActionChunk) -> ActionChunk:
        if not self._interpolate_chunk_ramps:
            return chunk
        return interpolate_action_chunk_ramps(
            chunk,
            interpolation_steps=self._ramp_interpolation_steps,
        ).motion

    def _interpolate_connected_motion(
        self,
        connected: ConnectedActionChunk,
    ) -> ConnectedActionChunk:
        if not self._interpolate_chunk_ramps:
            return connected
        interpolated = interpolate_action_chunk_ramps(
            connected.motion,
            interpolation_steps=self._ramp_interpolation_steps,
        )
        policy_start_steps = {
            group_id: interpolated.original_step_indices[group_id][policy_start_step]
            for group_id, policy_start_step in connected.policy_start_steps.items()
        }
        return ConnectedActionChunk(
            motion=interpolated.motion,
            bridge=connected.bridge,
            policy_start_steps=policy_start_steps,
        )

    async def _send_connected_policy_chunk(
        self,
        connected: ConnectedActionChunk,
        action: ActionChunk,
        *,
        step: int,
        start_time: float,
        last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        """Send continuous bridge+policy motion and defer side effects to its boundary."""
        bridge_steps = {
            group_id: len(steps)
            for group_id, steps in (*connected.bridge.joints.items(), *connected.bridge.tcp.items())
        }
        logger.info(
            "Policy chunk=%d uses continuous bridge steps=%s policy_start_steps=%s",
            step + 1,
            bridge_steps,
            connected.policy_start_steps,
        )
        if self._rerun is not None:
            self._rerun.log_bridge_chunk(connected.bridge, step)
        self._log_policy_action(action, step)
        target_chunks = self._send(connected.motion)
        termination = await self._run_actions_at_policy_boundary(
            connected,
            action,
            target_chunks,
            step=step,
            start_time=start_time,
            last_obs=last_obs,
        )
        if termination is None and self._policy_rate_hz >= 0 and action.action_timestep >= 0:
            for group_id, policy_start_step in connected.policy_start_steps.items():
                session = self._sessions.get(group_id)
                if session is None:
                    continue
                timestamps = session.scheduled_waypoint_timestamps
                if len(timestamps) <= policy_start_step:
                    continue
                # Policy waypoint zero already has an exact timestamp on NOVA's
                # jogger timer. Preserve it as the immutable queue origin.
                self._policy_timelines[group_id] = _PolicyTimeline(
                    action_timestep=action.action_timestep,
                    policy_dt_ms=action.dt_ms,
                    server_timestamp_ms=float(timestamps[policy_start_step]),
                )
            logger.info("Policy action timeline initialized: %s", self._policy_timelines)
        return termination

    async def _run_actions_at_policy_boundary(
        self,
        connected: ConnectedActionChunk,
        action: ActionChunk,
        target_chunks: dict[str, int],
        *,
        step: int,
        start_time: float,
        last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        """Wait on NOVA timestamps, then fire IO and computed policy actions."""
        pending_ios = dict(action.ios or {})
        while True:
            termination = self._termination_result(step, start_time, last_obs)
            if termination is not None:
                return termination
            if self._stop_event.is_set():
                return _result("stopped", step, start_time, last_obs)
            check_sessions(self._sessions)
            check_estop(self._estop_monitor)

            ready_groups = self._policy_boundary_ready_groups(connected, target_chunks)
            for group_id in list(pending_ios):
                if group_id in connected.policy_start_steps and group_id not in ready_groups:
                    continue
                session = self._sessions.get(group_id)
                if session is not None:
                    await session.write_ios(pending_ios[group_id])
                pending_ios.pop(group_id)

            if ready_groups == set(connected.policy_start_steps):
                await self._schema.run_computed_actions(action)
                return None
            await asyncio.sleep(0.001)

    def _policy_boundary_ready_groups(
        self,
        connected: ConnectedActionChunk,
        target_chunks: dict[str, int],
    ) -> set[str]:
        """Return motion groups whose NOVA clock reached policy waypoint zero."""
        ready: set[str] = set()
        for group_id, policy_start_step in connected.policy_start_steps.items():
            session = self._sessions.get(group_id)
            target_chunk = target_chunks.get(group_id)
            if session is None or target_chunk is None:
                continue
            timestamps = session.scheduled_waypoint_timestamps
            if (
                session.scheduled_chunk_count >= target_chunk
                and len(timestamps) > policy_start_step
                and session.last_server_timestamp_ms >= timestamps[policy_start_step]
            ):
                ready.add(group_id)
        return ready

    def _log_policy_action(self, action: ActionChunk, step: int) -> None:
        """Log a full policy prediction including its retained queue seam."""
        if self._rerun is None:
            return
        if action.action_timestep > 0 and self._policy.requires_first_waypoint_bridge:
            seam = ActionChunk(
                joints={
                    group_id: steps[:_ASYNC_SEAM_STEPS]
                    for group_id, steps in action.joints.items()
                    if len(steps) >= _ASYNC_SEAM_STEPS
                },
                tcp={
                    group_id: steps[:_ASYNC_SEAM_STEPS]
                    for group_id, steps in action.tcp.items()
                    if len(steps) >= _ASYNC_SEAM_STEPS
                },
                dt_ms=action.dt_ms,
                action_timestep=action.action_timestep,
            )
            if seam.joints or seam.tcp:
                self._rerun.log_bridge_chunk(seam, step)
        self._rerun.log_action_chunk(action, step, n_action_steps=self._n_action_steps)

    def _termination_result(
        self,
        step: int,
        start_time: float | None,
        last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        if (
            start_time is not None
            and self._timeout_s > 0
            and (time.monotonic() - start_time) >= self._timeout_s
        ):
            return _result("timeout", step, start_time, last_obs)
        stopped_by = triggered_stop_condition(self._sessions)
        if stopped_by is not None:
            return _result(
                f"stop condition: {stopped_by}",
                step,
                start_time or time.monotonic(),
                last_obs,
            )
        return None

    async def _prepare_policy(
        self,
        robot_states: dict[str, RobotState],
        images: dict[str, Any] | None,
    ) -> None:
        """Run policy setup before the execution timeout starts."""
        self._log_first_image_shapes(images)
        await self._policy.prepare(
            robot_states,
            self._schema,
            images,
            self._all_io_values or None,
        )

    def _log_policy_observation(
        self,
        robot_states: dict[str, RobotState],
        images: dict[str, Any] | None,
        step: int,
    ) -> None:
        """Log one policy observation to Rerun when visualization is active."""
        if self._rerun is None:
            return
        self._rerun.log_observation(robot_states, step)
        if images:
            self._rerun.log_images(images)

    def _log_first_image_shapes(self, images: dict[str, Any] | None) -> None:
        """Log camera keys/shapes once, when first frames enter the policy loop."""
        if self._logged_first_images or not images:
            return
        summary = {key: getattr(image, "shape", None) for key, image in images.items()}
        logger.info("First policy camera frames: %s", summary)
        self._logged_first_images = True

    async def _wait_after_send(
        self,
        tick_start: float,
        start_time: float,
        fixed_rate_period: float,
    ) -> None:
        """Pace the loop after a chunk is sent, per the configured timing mode."""
        if self._policy_rate_hz < 0:
            await self._wait_until_sessions_settled(start_time)
        elif self._policy_rate_hz > 0:
            # Fixed-rate: sleep for the remainder of the period.
            # Each new chunk replaces the previous one mid-execution.
            elapsed = time.monotonic() - tick_start
            sleep_time = fixed_rate_period - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    async def _wait_until_sessions_settled(self, exec_start: float) -> None:
        """Wait until each submitted NOVA chunk reaches its exact final timestamp."""
        target_chunks = {
            group_id: session.queued_chunk_count for group_id, session in self._sessions.items()
        }
        while True:
            if self._stop_event.is_set():
                return
            if self._timeout_s > 0 and (time.monotonic() - exec_start) >= self._timeout_s:
                return
            check_sessions(self._sessions)
            check_estop(self._estop_monitor)
            if triggered_stop_condition(self._sessions) is not None:
                return

            if all(
                session.scheduled_chunk_count >= target_chunks[group_id]
                and session.last_server_timestamp_ms >= session.scheduled_until_server_ms
                and session.jogging_state == "PAUSED_BY_USER"
                for group_id, session in self._sessions.items()
            ):
                return
            await asyncio.sleep(0.01)

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

    def _log_observation_to_first_waypoint(
        self,
        chunk: ActionChunk,
        robot_states: dict[str, RobotState],
        *,
        step: int,
        observation_time: float,
    ) -> None:
        """Log the exact observation, live pre-send state, and first joint target."""
        inference_ms = (time.monotonic() - observation_time) * 1000.0
        for group_id, steps in chunk.joints.items():
            observed_state = robot_states.get(group_id)
            session = self._sessions.get(group_id)
            live_state = session.current_state if session is not None else None
            if observed_state is None or live_state is None or not steps:
                continue

            observed = list(observed_state.joints)
            live = list(live_state.joints)
            first = steps[0]
            if len(observed) != len(live) or len(observed) != len(first):
                continue

            observed_to_first_deg = [
                abs(target - state) * 57.2958 for state, target in zip(observed, first, strict=True)
            ]
            observed_to_live_deg = [
                abs(current - state) * 57.2958
                for state, current in zip(observed, live, strict=True)
            ]
            logger.info(
                "%s policy chunk=%d observation-to-first max=%.2fdeg per_joint=%s "
                "observation-to-presend max=%.3fdeg inference=%.1fms "
                "observation=%s presend=%s first=%s",
                group_id,
                step,
                max(observed_to_first_deg, default=0.0),
                _format_values(observed_to_first_deg, digits=2),
                max(observed_to_live_deg, default=0.0),
                inference_ms,
                _format_values(observed),
                _format_values(live),
                _format_values(first),
            )

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

    def _send(self, chunk: ActionChunk) -> dict[str, int]:
        """Send an action chunk to the motion groups.

        Placement (relative vs. absolute, with optional overlap backdate) is
        decided per session by :func:`novapolicy.chunking.placement`; the "now"
        component is resolved at yield time inside the session. For overlapping
        mode to keep the robot moving, ``(horizon - seam_backdate_steps) *
        dt_ms`` must exceed the inference latency (increase dt_ms or cut latency
        otherwise).
        """
        place = placement(chunk, policy_rate_hz=self._policy_rate_hz)
        server_dt_ms = (
            chunk.dt_ms
            if self._policy_rate_hz >= 0 and self._policy.requires_first_waypoint_bridge
            else None
        )
        target_chunks: dict[str, int] = {}
        for group_id, steps in chunk.joints.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in chunk: %s", group_id)
                continue
            server_timestamp_ms = self._policy_server_timestamp_ms(chunk, group_id)
            session.update_chunk(
                steps=steps,
                dt_ms=chunk.dt_ms,
                first_timestamp_ms=(
                    server_timestamp_ms
                    if server_timestamp_ms is not None
                    else place.first_timestamp_ms
                ),
                timestamp_offset_steps=(
                    0 if server_timestamp_ms is not None else place.timestamp_offset_steps
                ),
                server_dt_ms=server_dt_ms,
                action_timestep=chunk.action_timestep,
            )
            target_chunks[group_id] = session.queued_chunk_count

        for group_id, raw_tcp_steps in chunk.tcp.items():
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in TCP chunk: %s", group_id)
                continue
            server_timestamp_ms = self._policy_server_timestamp_ms(chunk, group_id)
            session.update_chunk(
                steps=raw_tcp_steps,
                dt_ms=chunk.dt_ms,
                first_timestamp_ms=(
                    server_timestamp_ms
                    if server_timestamp_ms is not None
                    else place.first_timestamp_ms
                ),
                timestamp_offset_steps=(
                    0 if server_timestamp_ms is not None else place.timestamp_offset_steps
                ),
                server_dt_ms=server_dt_ms,
                action_timestep=chunk.action_timestep,
            )
            target_chunks[group_id] = session.queued_chunk_count

        if chunk.ios:
            for group_id, ios in chunk.ios.items():
                session = self._sessions.get(group_id)
                if session is None:
                    continue
                task = asyncio.create_task(session.write_ios(ios))
                self._io_tasks.add(task)
                task.add_done_callback(self._io_tasks.discard)
        return target_chunks

    def _policy_server_timestamp_ms(
        self,
        chunk: ActionChunk,
        group_id: str,
    ) -> int | None:
        """Map a policy action timestep onto its immutable NOVA timeline."""
        timeline = self._policy_timelines.get(group_id)
        if chunk.action_timestep < 0 or timeline is None:
            return None
        elapsed_steps = chunk.action_timestep - timeline.action_timestep
        return int(timeline.server_timestamp_ms + elapsed_steps * timeline.policy_dt_ms)

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
            trajectory_trace=(
                self._trajectory_trace.create_session_trace(mg.id, mode)
                if self._trajectory_trace is not None
                else None
            ),
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
        from novapolicy.rerun import _is_rerun_active  # noqa: PLC0415

        if not _is_rerun_active():
            return

        from novapolicy.rerun import PolicyRerunLogger  # noqa: PLC0415

        self._rerun = PolicyRerunLogger(
            self._motion_groups,
            camera_names=self._cameras.names,
            use_tcp_offset_for_joint_actions=True,
        )
        await self._rerun.initialize()

    def _record_policy_chunk(
        self,
        action: ActionChunk,
        robot_states: dict[str, RobotState],
        *,
        step: int,
    ) -> None:
        """Collect policy output and controller samples when tracing is enabled."""
        if self._trajectory_trace is None:
            return
        self._trajectory_trace.record_policy_chunk(
            action,
            robot_states,
            {
                group_id: session.last_server_timestamp_ms
                for group_id, session in self._sessions.items()
            },
            step=step,
        )

    def _write_trajectory_trace(self) -> None:
        """Write optional diagnostics after control loops have stopped."""
        if self._trajectory_trace is None:
            return
        result = self.result
        self._trajectory_trace.write(
            reason=result.reason if result is not None else None,
            steps=result.steps if result is not None else None,
            duration_s=result.duration_s if result is not None else None,
            policy=self._policy,
        )
        logger.info("Wrote trajectory trace to %s", self._trajectory_trace.path)

    def _log_completion(self) -> None:
        """Log execution result to Rerun."""
        if self._rerun is not None and self.result is not None:
            self._rerun.log_completion(
                self.result.reason,
                self.result.steps,
                self.result.duration_s,
            )


def _format_values(values: list[float], *, digits: int = 4) -> str:
    return "[" + ",".join(f"{value:.{digits}f}" for value in values) + "]"


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
