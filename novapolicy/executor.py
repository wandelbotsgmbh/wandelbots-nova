"""PolicyExecutor — runs one policy episode via waypoint jogging."""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass
from enum import StrEnum
import gc
import logging
import time
from typing import TYPE_CHECKING, Any

from nova.actions import jnt
from novapolicy.cameras.manager import CameraManager
from novapolicy.chunking import (
    apply_relative_mode,
    is_single_step,
    placement,
    trim_chunk,
)
from novapolicy.estop import (
    EstopMonitor,
    check_estop,
    check_sessions,
    triggered_stop_condition,
)
from novapolicy.io import IOStreamManager
from novapolicy.jogging.live_buffer import LiveBuffer
from novapolicy.jogging.modules import TimestepTimeline, default_pipeline
from novapolicy.jogging.pipeline import split_pipeline
from novapolicy.jogging.waypoint_session import WaypointJoggingSession
from novapolicy.policy_client import PolicyClient
from novapolicy.types import (
    ActionChunk,
    ContinuousExecution,
    EmergencyStopError,
    ExecutionMode,
    MotionError,
    SequentialExecution,
    StopContext,
    WaypointConfig,
)

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Sequence

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.jogging.clock import ServerMs
    from novapolicy.rerun import PolicyRerunLogger
    from novapolicy.schema import PolicySchema
    from novapolicy.types import StopCondition

logger = logging.getLogger(__name__)

_ASYNC_REPLACEMENT_LEAD_STEPS = 1
_DEFAULT_WAYPOINT_CONFIG = WaypointConfig()
_DEFAULT_EXECUTION = SequentialExecution()

# A camera feeding a control loop should be fresh in frame terms, not in tens of
# seconds. One second is loose enough for a slow stream and tight enough that a
# frozen feed surfaces before it has driven many chunks of motion.
_DEFAULT_CAMERA_MAX_AGE_S = 1.0

# Outer liveness guard on one inference. Generous by design: it exists to catch a
# hung service, not to police latency. Transports may have tighter deadlines of
# their own (LeRobot's gRPC calls use 15s).
_DEFAULT_INFERENCE_TIMEOUT_S = 30.0


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


def _check_execution_mode(
    execution: ExecutionMode, policy: PolicyClient, pipeline: Sequence[object]
) -> None:
    """Reject a setup the executor would run wrongly, before anything moves.

    Sequential execution waits for standstill and observes again, so nothing in it replaces a
    plan while it runs: a timestep queue (``QueuedPolicyClient``) or a
    live-target buffer would be fed timings that never come. In continuous
    execution a timestep queue needs its ``TimestepTimeline``, or every chunk restarts at "now".
    """
    queue = any(isinstance(m, TimestepTimeline) for m in policy.default_modules)
    if not isinstance(execution, SequentialExecution):
        if queue and not any(isinstance(m, TimestepTimeline) for m in pipeline):
            raise ValueError(
                f"{type(policy).__name__} numbers its steps: put its default_modules (a "
                "TimestepTimeline) in the pipeline, or use default_pipeline(execution=..., "
                "policy=...)"
            )
        return
    if queue:
        raise ValueError(
            f"{type(policy).__name__} replaces its plan step by step and requires "
            "ContinuousExecution. For SequentialExecution drop the queue: the policy without "
            "QueuedPolicyClient"
        )
    if any(isinstance(module, LiveBuffer) for module in pipeline):
        raise ValueError("LiveBuffer requires ContinuousExecution")


def resolve_actions(
    schema: PolicySchema, chunk: ActionChunk, states: dict[str, RobotState]
) -> ActionChunk:
    """A policy's raw chunk as the executor sends it: units and rotation representation
    converted back (the schema's operators and ``orientation_type``), relative targets
    composed with ``states`` in each action's coordinate system, and TCP targets expressed
    in the world. Use it for dry runs (after ``await schema.load_coordinate_systems()``)."""
    chunk = schema.apply_inverse_ops(chunk)  # NOVA units, rotation vectors, policy frames
    chunk = apply_relative_mode(
        chunk,
        schema.policy_frame_states(states),
        schema.relative_motion_groups(),
        schema.relative_tcp_deltas(),
    )
    return schema.tcp_targets_to_world(chunk)


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
        camera_max_age_s: float = _DEFAULT_CAMERA_MAX_AGE_S,
        inference_timeout_s: float = _DEFAULT_INFERENCE_TIMEOUT_S,
        motion: WaypointConfig = _DEFAULT_WAYPOINT_CONFIG,
        execution: ExecutionMode = _DEFAULT_EXECUTION,
        n_action_steps: int | None = None,
        start_joint_position: dict[MotionGroup, list[float]] | None = None,
        pipeline: Sequence[object] | None = None,
    ) -> None:
        """Create a policy executor.

        Args:
            schema: Observation/action schema defining robot topology.
            policy: PolicyClient that maps observations to actions.
            stop_conditions: Optional checks run each tick; one returning ``True``
                stops the run normally (its name appears in ``result.reason``).
            timeout_s: Maximum execution duration in seconds. 0 = no timeout.

            camera_max_age_s: Default freshness bound for camera frames, in
                seconds. A channel can override it with
                ``Observation.image(..., max_age_s=...)``. Frames older than the
                bound are stale and end the run with ``RuntimeError``. Applies to the policy's
                frames only — visualization uses a looser bound of its own.
            inference_timeout_s: Deadline for one ``get_actions`` call, in
                seconds. 0 = no deadline. This is a liveness guard against a
                hung policy service, applied uniformly whatever the transport's
                own timeout is; it is not a real-time guarantee.
            motion: Waypoint jogging configuration. Defaults to WaypointConfig().
            execution: Explicit execution strategy. ``SequentialExecution``
                completes and settles each chunk before the next inference;
                ``ContinuousExecution`` replaces the active lookahead either as
                fast as possible or at its configured fixed rate.
            n_action_steps: Number of steps from each action chunk to execute.
                None (default): Use the policy's own horizon when it declares
                one (a LeRobot client reports its checkpoint's value), else all
                steps.
                0: Execute all steps returned by the policy, ignoring any
                horizon the policy declares.
                >0: Trim to first N steps (receding horizon). Later steps
                have higher prediction uncertainty and are discarded.
                The policy still predicts the full action_horizon (e.g. 16)
                which remains available to asynchronous policy clients.
            start_joint_position: Optional mapping of motion group to joint pose
                to PTP-move to before starting waypoint jogging.
            pipeline: The modules between the policy's chunk and NOVA, in order, exactly as
                given; ``None`` runs :func:`~novapolicy.jogging.modules.default_pipeline`. To
                add one, extend that list with the same execution and policy, e.g.
                ``[*default_pipeline(execution=execution, policy=policy), EaseIn(duration_s=1.0)]``.
                Each motion group runs its own copy. See :mod:`novapolicy.jogging.pipeline`.
        """
        if pipeline is None:
            pipeline = default_pipeline(execution=execution, policy=policy)
        split_pipeline(pipeline)  # refuse a misordered pipeline before anything moves
        self._pipeline = tuple(pipeline)
        self._schema = schema
        self._motion_groups = schema.get_motion_groups()

        if not isinstance(policy, PolicyClient):
            msg = "policy must be a PolicyClient; wrap callbacks with CallbackPolicyClient"
            raise TypeError(msg)
        self._policy = policy

        self._motion = motion
        self._start_joint_position = start_joint_position
        self._stop_conditions = stop_conditions or []
        self._timeout_s = timeout_s
        if inference_timeout_s < 0:
            raise ValueError("inference_timeout_s must not be negative")
        self._inference_timeout_s = inference_timeout_s
        if not isinstance(execution, ExecutionMode):
            raise TypeError("execution must be SequentialExecution or ContinuousExecution")
        self._execution = execution
        self._n_action_steps_cfg = n_action_steps

        _check_execution_mode(execution, self._policy, self._pipeline)

        self._sessions: dict[str, WaypointJoggingSession] = {}
        self._cameras = CameraManager(camera_max_age_s)
        self._stop_event = asyncio.Event()
        self._last_obs: dict[str, Any] | None = None
        self._estop_monitor: EstopMonitor | None = None
        self._io_streams = IOStreamManager(self._motion_groups, schema.io_keys_by_controller())
        self._io_tasks: set[asyncio.Task[None]] = set()
        self._observed_at_ms: dict[str, ServerMs | None] = {}
        self._observed_clock_ms: dict[str, ServerMs] = {}
        self._gc_frozen = False
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

        How the run ended decides what happens to motion already accepted by the
        controller. A run that ended normally — a stop condition, the execution
        timeout or an external :meth:`stop` — owes the caller the waypoints it accepted, so they
        are drained first. A run that ended in an error or an e-stop drops them:
        ``session.stop`` cancels immediately, which is the point.
        """
        if self.status.phase is not Phase.ERROR:
            for session in self._sessions.values():
                with contextlib.suppress(MotionError, EmergencyStopError, OSError, RuntimeError):
                    await session.drain()

        # In parallel: each stop waits for its server to settle.
        results = await asyncio.gather(
            *(session.stop() for session in self._sessions.values()), return_exceptions=True
        )
        for result in results:
            if isinstance(result, BaseException) and not isinstance(
                result, (MotionError, EmergencyStopError, OSError, RuntimeError)
            ):
                raise result

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
            # The cell's collision setups stay in force: the start move is planned like any
            # other move, not around them.
            setup = await mg.get_setup(tcp)
            target = tuple(joints)
            trajectory = await mg.plan([jnt(target)], tcp, motion_group_setup=setup)
            await mg.execute(trajectory, tcp, actions=[jnt(target)])

        start_positions = self._start_joint_position
        if start_positions is None:
            return
        await asyncio.gather(*(ptp(mg, joints) for mg, joints in start_positions.items()))

    async def _run_episode(self) -> None:  # ruff: ignore[complex-structure]
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
            self._sessions[mg.id].set_state_observer(
                lambda state, ts_ms, _generated, group_id=mg.id: self._schema.observe_state(
                    group_id, state.joints, ts_ms / 1000.0
                )
            )

        if self._start_joint_position:
            await self._move_to_start_joint_position()

        image_sources = self._schema.image_sources
        if image_sources:
            logger.info("Connecting cameras...")
            await self._cameras.connect(image_sources, self._schema.image_max_age_s)
            logger.info("All cameras ready")

        for session in self._sessions.values():
            await session.start()

        try:
            await self._io_streams.start()
            self._io_streams.wire_to_sessions(self._sessions)
            await self._schema.load_coordinate_systems()
            await self._schema.start_sources()
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
            if self._rerun is not None:
                self._rerun.log_completion(
                    self.result.reason, self.result.steps, self.result.duration_s
                )
            self.status.phase = Phase.COMPLETED
        finally:
            # Release only what this method started; sessions + policy are
            # handled by _cleanup() in run()'s finally.
            if self._gc_frozen:
                gc.unfreeze()
                self._gc_frozen = False
            if self._rerun is not None:
                await self._rerun.stop_streaming()
            if self._estop_monitor is not None:
                await self._estop_monitor.stop()
                self._estop_monitor = None
            if self._cameras.active:
                await self._cameras.disconnect()
            await self._schema.stop_sources()
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

        fixed_rate_period = (
            1.0 / self._execution.rate_hz
            if isinstance(self._execution, ContinuousExecution)
            and self._execution.rate_hz is not None
            else 0.0
        )
        tick_due: float | None = None  # when the current tick was due on the fixed-rate grid

        while not self._stop_event.is_set():
            # Always yield once per iteration so stop()/other tasks make progress
            # even when the policy and sleeps below complete synchronously
            # (e.g. dt_ms=0 or a local in-process policy).
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
            action, start_time = await self._observe_and_infer(robot_states, step, start_time)
            if step == 0:
                tick_start = start_time
            last_obs = robot_states
            termination = await self._send_policy_action(
                action,
                robot_states,
                step=step,
                start_time=start_time,
                last_obs=last_obs,
            )
            if termination is not None:
                return termination
            step += 1
            self.status.step = step

            # Check failures — raises directly on error
            check_sessions(self._sessions)
            check_estop(self._estop_monitor)

            # A stop condition firing on a session tick ends the run normally.
            stopped_by = triggered_stop_condition(self._sessions)
            if stopped_by is not None:
                return _result(f"stop condition: {stopped_by}", step, start_time, last_obs)

            tick_due = tick_start if tick_due is None else tick_due
            tick_due = await self._wait_after_send(tick_due, start_time, fixed_rate_period)

        return _result("stopped", step, start_time or time.monotonic(), last_obs)

    def _synchronize_policy_action_timestep(self) -> None:
        """Tell a timestep queue which of its steps it may still replace: the one after the step
        the sessions are executing now (read from the controller's clock)."""
        executing = max((s.executing_action_timestep for s in self._sessions.values()), default=-1)
        if executing >= 0:
            self._policy.synchronize_action_timestep(executing + _ASYNC_REPLACEMENT_LEAD_STEPS)

    def _bridges_in_queue(self) -> dict[str, list[list[float]]]:
        """Each session's bridge in the chunk just queued: its steps up to the policy's first."""
        bridges: dict[str, list[list[float]]] = {}
        for group_id, session in self._sessions.items():
            queued = session.queued_chunk
            if queued is not None and queued.start_step > 0:
                bridges[group_id] = queued.steps[: queued.start_step + 1]
        return bridges

    async def _send_policy_action(
        self,
        action: ActionChunk,
        robot_states: dict[str, RobotState],
        *,
        step: int,
        start_time: float,
        last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        """Trim, place and send one chunk. Returns a result when the run ends here."""
        trimmed = trim_chunk(action, self._n_action_steps)
        stopped_by = self._check_stop_conditions_pre_send(trimmed, robot_states)
        if stopped_by is not None:
            return _result(f"stop condition: {stopped_by}", step, start_time, last_obs)

        if self._rerun is not None:
            self._rerun.log_target_tracking(trimmed, robot_states, step)
            self._rerun.log_action_chunk(action, step, n_action_steps=self._n_action_steps)
        self._send(trimmed, action)
        # The sessions' Bridge module, if any, put a bridge in front of the policy's steps.
        bridges = self._bridges_in_queue()
        if bridges and self._rerun is not None:
            self._rerun.log_bridge_chunk(
                ActionChunk(
                    joints={g: b for g, b in bridges.items() if g in trimmed.joints},
                    tcp={g: b for g, b in bridges.items() if g in trimmed.tcp},
                    dt_ms=trimmed.dt_ms,
                ),
                step,
            )
        return None

    async def _observe_and_infer(
        self,
        robot_states: dict[str, RobotState],
        step: int,
        start_time: float | None,
    ) -> tuple[ActionChunk, float]:
        """Read the cameras, prepare on the first tick, and query the policy.

        Returns ``(action, start_time)``; ``start_time`` is set on the
        first tick, after policy preparation, so model loading is excluded from
        the execution timeout.

        Raises:
            RuntimeError: A camera frame or the inference missed its deadline.
        """
        images = None
        if self._cameras.active:
            try:
                images = self._cameras.read()
            except RuntimeError as exc:
                msg = f"Stale policy input (camera: {exc})"
                raise RuntimeError(msg) from exc
        if images and not self._logged_first_images:
            shapes = {key: getattr(image, "shape", None) for key, image in images.items()}
            logger.info("First policy camera frames: %s", shapes)
            self._logged_first_images = True
        self._last_obs = robot_states
        if self._rerun is not None:
            self._rerun.log_observation(robot_states, step)
            if images:
                self._rerun.log_images(images)

        if start_time is None:
            await self._policy.prepare(
                robot_states, self._schema, images, self._io_streams.all_values or None
            )
            self._freeze_setup_objects()
            start_time = time.monotonic()
            self.status.phase = Phase.EXECUTING
            self.status.message = "Running policy..."

        return await self._infer(robot_states, images), start_time

    def _freeze_setup_objects(self) -> None:
        """Keep full garbage collections from pausing the control loop.

        Everything allocated up to here (SDK clients, sessions, a loaded model) lives for the
        whole run, and traversing it stalled the loop for over 100 ms per full collection.
        ``gc.freeze()`` takes it out of collection; the collector stays on, and ``run()``
        unfreezes at the end (which also releases anything the host froze).
        """
        gc.collect()
        gc.freeze()
        self._gc_frozen = True

    async def _infer(
        self,
        robot_states: dict[str, RobotState],
        images: dict[str, Any] | None,
    ) -> ActionChunk:
        """Synchronize a queue, request its action under the executor's inference deadline, and
        apply relative targets."""
        try:
            self._synchronize_policy_action_timestep()
            async with asyncio.timeout(self._inference_timeout_s or None):
                action = await self._policy.get_actions(
                    robot_states, self._schema, images, self._io_streams.all_values or None
                )
            # Units first: a relative target is a delta in the policy's units, and
            # adding a degree-space delta to a radian state is exactly the error the
            # operators exist to prevent.
            if action.resolved:
                return action
            return resolve_actions(self._schema, action, robot_states)
        except TimeoutError as exc:
            msg = f"Stale policy input (inference exceeded {self._inference_timeout_s:.0f}s)"
            raise RuntimeError(msg) from exc

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

    async def _wait_after_send(
        self,
        tick_due: float,
        start_time: float,
        fixed_rate_period: float,
    ) -> float:
        """Pace the loop after a chunk is sent, per the configured timing mode.

        Fixed-rate execution sleeps until the next tick on an absolute grid (as ROS
        ``Rate.sleep``), so a slow tick or a late wake-up is made up by the next tick
        instead of lowering the rate. More than a period behind, the grid restarts
        from now rather than firing catch-up ticks. Returns when the next tick is due.
        """
        if isinstance(self._execution, SequentialExecution):
            await self._wait_until_sessions_settled(start_time)
            return time.monotonic()
        if fixed_rate_period <= 0:
            return time.monotonic()
        next_due = tick_due + fixed_rate_period
        delay = next_due - time.monotonic()
        if delay > 0:
            await asyncio.sleep(delay)
        elif delay < -fixed_rate_period:
            return time.monotonic()
        return next_due

    async def _wait_until_sessions_settled(self, exec_start: float) -> None:
        """Wait until every submitted chunk has run out *and* the robot has stopped.

        Both halves are needed: past the last timestamp the robot is still braking, and before
        the first it is legitimately still. Standstill is measured from the joint positions
        (:attr:`WaypointJoggingSession.is_at_standstill`), since NOVA reports no "waypoints ran
        out" state; ``PAUSED_BY_USER`` is accepted when it appears.
        """
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
                and (session.is_at_standstill or session.jogging_state == "PAUSED_BY_USER")
                for group_id, session in self._sessions.items()
            ):
                return
            await asyncio.sleep(0.01)

    # -------------------------------------------------------------------------
    # Session operations
    # -------------------------------------------------------------------------

    def _observe(self) -> dict[str, Any]:
        """Get current state for all motion groups."""
        result: dict[str, Any] = {}
        for group_id, session in self._sessions.items():
            state = session.current_state
            if state is not None:
                result[group_id] = state
                # When the observation was taken, on the server's clock: a continuous chunk is
                # due a fixed time after it. A one-step target goes by the state's own timestamp
                # (its playback clock smooths that); a multi-step chunk by the clock estimate,
                # since the state stream's jitter and stalls would shift whole chunks against
                # each other and the follower turns every shifted seam into a speed spike.
                self._observed_clock_ms[group_id] = session.estimated_server_timestamp_ms
                measured_ms = session.current_state_server_ms
                self._observed_at_ms[group_id] = (
                    measured_ms
                    if measured_ms is not None
                    else session.estimated_server_timestamp_ms
                )
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

        group_ids = {*chunk.joints, *chunk.tcp}
        if chunk.ios:
            group_ids.update(chunk.ios)

        for group_id in group_ids:
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
                io_values=self._io_streams.all_values,
                target_joints=target_joints,
                target_ios=target_ios,
            )
            for condition in self._stop_conditions:
                if condition(ctx):
                    return getattr(condition, "__name__", repr(condition))
        return None

    def _send(self, chunk: ActionChunk, effects: ActionChunk) -> None:
        """Hand an action chunk to each motion group's session, placed by
        :func:`novapolicy.chunking.placement` (the session's modules may still move it).

        Contract: the IO and computed actions of ``effects`` (the policy's chunk) run when the
        policy's first step is due; a chunk module that puts steps in front keeps them there. A
        group with IO but no motion, and computed actions of a chunk without motion, run at once.
        """
        place = placement(chunk, execution=self._execution)
        # A one-step chunk placed ahead is replaced by the next step before it is due: padding
        # it with a hold would make the controller brake towards every step.
        hold = not (
            place.lead
            and is_single_step(chunk)
            and isinstance(self._execution, ContinuousExecution)
        )
        moving = [*chunk.joints, *chunk.tcp]
        for group_id, steps in [*chunk.joints.items(), *chunk.tcp.items()]:
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in chunk: %s", group_id)
                continue
            session.update_chunk(
                steps=steps,
                dt_ms=chunk.dt_ms,
                first_timestamp_ms=place.first_timestamp_ms,
                action_timestep=chunk.action_timestep,
                extend_buffer=hold,
                lead=place.lead,
                observed_at_ms=self._observed_ms(group_id, chunk) if place.lead else None,
                when_due=self._effects_when_due(effects, group_id, moving),
                retained_prefix_steps=chunk.retained_prefix_steps,
            )
        self._schedule_ios(effects, skip=set(moving))
        if not moving and self._schema.has_computed_actions:
            task = asyncio.create_task(self._schema.run_computed_actions(effects))
            self._io_tasks.add(task)
            task.add_done_callback(self._io_tasks.discard)

    def _observed_ms(self, group_id: str, chunk: ActionChunk) -> ServerMs | None:
        """When the observation behind ``chunk`` was taken, on the server's clock; ``None`` in
        sequential execution, where the send is the reference (the robot was at rest)."""
        if isinstance(self._execution, SequentialExecution):
            return None
        if is_single_step(chunk):
            return self._observed_at_ms.get(group_id)
        return self._observed_clock_ms.get(group_id)

    def _effects_when_due(
        self, effects: ActionChunk, group_id: str, moving: list[str]
    ) -> tuple[int, Callable[[], Awaitable[None]]] | None:
        """The session runs this group's IO, and with the first moving group the chunk's
        computed actions, when the policy's first step is due.

        Interim: NOVA's action chunk API is to carry timestamped IO itself. Then the IO goes out
        with the waypoints, on the controller's clock, and only computed actions remain here.
        """
        ios = (effects.ios or {}).get(group_id)
        computed = group_id == moving[0] and self._schema.has_computed_actions
        session = self._sessions[group_id]

        async def run() -> None:
            if ios:
                await session.write_ios(ios)
            if computed:
                await self._schema.run_computed_actions(effects)

        return (0, run) if ios or computed else None

    def _schedule_ios(self, chunk: ActionChunk, *, skip: set[str] | None = None) -> None:
        """Write the chunk's IO at once, except for the groups in ``skip``."""
        for group_id, ios in (chunk.ios or {}).items():
            session = self._sessions.get(group_id)
            if session is None or group_id in (skip or set()):
                continue
            task = asyncio.create_task(session.write_ios(ios))
            self._io_tasks.add(task)
            task.add_done_callback(self._io_tasks.discard)

    def _create_session(self, mg: MotionGroup) -> WaypointJoggingSession:
        """Create a waypoint jogging session for a motion group."""
        tcp_groups = self._schema.tcp_action_groups()
        return WaypointJoggingSession(
            motion_group=mg,
            config=self._motion,
            tcp=tcp_groups.get(mg.id) or self._schema.tcp,
            stop_conditions=self._stop_conditions,
            mode="cartesian" if mg.id in tcp_groups else "joint",
            pipeline=self._pipeline,
        )

    @property
    def _n_action_steps(self) -> int:
        """Number of action steps to execute from each chunk (0 = all).

        An explicit executor argument always wins, including an explicit ``0``
        that keeps the full horizon for an asynchronous queue. Only when none
        was given does the policy's own declared horizon apply.
        """
        if self._n_action_steps_cfg is not None:
            return self._n_action_steps_cfg
        return self._policy.n_action_steps or 0

    # -------------------------------------------------------------------------
    # Rerun visualization (lazy, zero-cost when viewer not active)
    # -------------------------------------------------------------------------

    async def _init_rerun(self) -> None:
        """Initialize Rerun logger if a viewer is active."""
        from novapolicy.rerun import _is_rerun_active  # ruff: ignore[import-outside-top-level]

        if not _is_rerun_active():
            return

        from novapolicy.rerun import PolicyRerunLogger  # ruff: ignore[import-outside-top-level]

        self._rerun = PolicyRerunLogger(
            self._motion_groups,
            camera_names=self._cameras.names,
            use_tcp_offset_for_joint_actions=True,
            tcp=self._schema.tcp,
        )
        await self._rerun.initialize()


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
