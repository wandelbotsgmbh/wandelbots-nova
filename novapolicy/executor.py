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
from novapolicy.jogging.clock import ServerMs
from novapolicy.jogging.live_buffer import LiveBuffer
from novapolicy.jogging.modules import Bridge, default_pipeline
from novapolicy.jogging.pipeline import split_pipeline
from novapolicy.jogging.waypoint_session import WaypointJoggingSession
from novapolicy.policy_client import PolicyClient
from novapolicy.types import (
    ActionChunk,
    ContinuousExecution,
    EmergencyStopError,
    ExecutionMode,
    MotionError,
    OnStale,
    SequentialExecution,
    StopContext,
    WaypointConfig,
)

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable, Mapping, Sequence

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.rerun import PolicyRerunLogger
    from novapolicy.schema import PolicySchema
    from novapolicy.types import StopCondition

logger = logging.getLogger(__name__)

_ASYNC_SEAM_STEPS = 4
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

# How long OnStale.HOLD may retry before escalating.
_DEFAULT_HOLD_BUDGET_S = 2.0

# Pause between HOLD retries, so a stale channel cannot spin the loop hot.
_HOLD_RETRY_PERIOD_S = 0.05


class _StaleSignal(Exception):  # ruff: ignore[error-suffix-on-exception-name] - control flow
    """Base for the two stale-input outcomes that do not raise to the caller."""

    def __init__(self, what: str) -> None:
        super().__init__(what)
        self.what = what


class _StaleHold(_StaleSignal):
    """Retry the tick after a pause."""


class _StaleStop(_StaleSignal):
    """End the run normally, running out the waypoints already accepted."""


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


def _check_execution_mode(
    execution: ExecutionMode, policy: PolicyClient, pipeline: Sequence[object]
) -> None:
    """Reject a setup the executor would run wrongly, before anything moves.

    Sequential execution waits for standstill and observes again, so nothing in it replaces a
    plan while it runs: a timestep queue (``QueuedPolicyClient``, LeRobot's async
    queue) or a live-target buffer would be fed timings that never come. In continuous
    execution a timestep queue anchors its timeline on its first bridged chunk, so its pipeline
    needs a ``Bridge``.
    """
    if not isinstance(execution, SequentialExecution):
        if policy.requires_first_waypoint_bridge and not any(
            isinstance(module, Bridge) for module in pipeline
        ):
            raise ValueError(
                f"{type(policy).__name__} continues its plan from its first bridged chunk: put "
                "Bridge(always_anchor=True) in the pipeline, or use "
                "default_pipeline(execution=..., policy=...)"
            )
        return
    if policy.requires_first_waypoint_bridge:
        raise ValueError(
            f"{type(policy).__name__} replaces its plan step by step and requires "
            "ContinuousExecution. For SequentialExecution drop the queue: the policy without "
            "QueuedPolicyClient, or LeRobotPolicyClient(use_async_queue=False)"
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
        on_stale: OnStale = OnStale.ABORT,
        hold_budget_s: float = _DEFAULT_HOLD_BUDGET_S,
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
                bound are stale; see ``on_stale``. Applies to the policy's
                frames only — visualization uses a looser bound of its own.
            inference_timeout_s: Deadline for one ``get_actions`` call, in
                seconds. 0 = no deadline. This is a liveness guard against a
                hung policy service, applied uniformly whatever the transport's
                own timeout is; it is not a real-time guarantee.
            on_stale: What to do when a camera frame or an inference misses its
                deadline. ``ABORT`` (default) raises, as the executor did before
                this was declared. ``CONTROLLED_STOP`` runs out the accepted
                waypoints and ends the run with a ``stale: ...`` reason.
                ``HOLD`` retries, and applies to camera staleness only — see
                :class:`OnStale`.
            hold_budget_s: How long ``HOLD`` may keep retrying before escalating
                to ``CONTROLLED_STOP``. Measured from the first stale tick, so
                it does not depend on how fast the loop spins.
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
                add one, extend that list, e.g. ``[*default_pipeline(), EaseIn(duration_s=1.0)]``.
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
        if hold_budget_s < 0:
            raise ValueError("hold_budget_s must not be negative")
        self._inference_timeout_s = inference_timeout_s
        self._on_stale = OnStale(on_stale)
        self._hold_budget_s = hold_budget_s
        self._stale_since: float | None = None
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
        self._policy_timelines: dict[str, _PolicyTimeline] = {}
        self._observed_at_ms: dict[str, ServerMs | None] = {}
        self._observed_clock_ms: dict[str, ServerMs] = {}
        self._gc_frozen = False
        self._rerun: PolicyRerunLogger | None = None
        self._logged_first_images = False
        self._reported_missing_motion = False

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
        self.result = None
        self._reported_missing_motion = False  # once per run, not per executor
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
        timeout, an external :meth:`stop`, or stale data under
        ``CONTROLLED_STOP`` — owes the caller the waypoints it accepted, so they
        are drained first. A run that ended in an error or an e-stop drops them:
        ``session.stop`` cancels immediately, which is the point.
        """
        if self.status.phase is not Phase.ERROR:
            await self._drain_sessions()

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
        # A fresh episode gets a fresh replay window and timeline anchor.

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
            self._log_completion()
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
            observation_time = time.monotonic()
            try:
                action, start_time = await self._observe_and_infer(robot_states, step, start_time)
            except _StaleHold:
                await asyncio.sleep(_HOLD_RETRY_PERIOD_S)
                continue
            except _StaleStop as stale:
                await self._drain_sessions()
                return _result(
                    f"stale: {stale.what}",
                    step,
                    start_time or time.monotonic(),
                    last_obs,
                )
            if step == 0:
                tick_start = start_time
            last_obs = robot_states
            self._log_observation_to_first_waypoint(
                action,
                robot_states,
                step=step + 1,
                observation_time=observation_time,
            )
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

    async def _get_policy_actions(
        self,
        robot_states: dict[str, RobotState],
        images: dict[str, Any] | None,
    ) -> ActionChunk:
        """Synchronize a queue, request its action, and apply relative targets."""
        self._synchronize_policy_action_timestep()
        async with asyncio.timeout(self._inference_timeout_s or None):
            action = await self._policy.get_actions(
                robot_states,
                self._schema,
                images,
                self._all_io_values or None,
            )
        self._report_missing_motion(action)
        # Units first: a relative target is a delta in the policy's units, and
        # adding a degree-space delta to a radian state is exactly the error the
        # operators exist to prevent.
        if action.resolved:
            return action
        return resolve_actions(self._schema, action, robot_states)

    def _report_missing_motion(self, action: ActionChunk) -> None:
        """Log once when the policy's answer moves none of the expected motion groups.

        A client that cannot decode its server's reply hands back an empty chunk,
        which otherwise only shows as a robot that never moves until the timeout.
        """
        if action.joints or action.tcp or action.ios or self._reported_missing_motion:
            return
        expected = self._schema.motion_action_keys
        if not expected:
            return
        self._reported_missing_motion = True
        logger.error(
            "The policy returned no motion for any motion group (expected %s). "
            "Check that the policy client decodes its server's reply.",
            ", ".join(expected),
        )

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

    def _bridges_in_queue(self) -> dict[str, list[list[float]]]:
        """Each session's bridge in the chunk just queued: its steps up to the policy's first."""
        bridges: dict[str, list[list[float]]] = {}
        for group_id, session in self._sessions.items():
            queued = session.queued_chunk
            if queued is not None and queued.start_step > 0:
                bridges[group_id] = queued.steps[: queued.start_step + 1]
        return bridges

    async def _anchor_policy_timeline(
        self,
        action: ActionChunk,
        bridges: dict[str, list[list[float]]],
        target_chunks: dict[str, int],
        *,
        step: int,
        start_time: float,
        last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        """A timestep queue continues from its first bridged chunk: remember when the policy's
        first step of it is due, per motion group, as the queue's timeline."""
        start_steps = {group_id: len(steps) - 1 for group_id, steps in bridges.items()}
        logger.info(
            "Policy chunk=%d uses continuous bridge steps=%s policy_start_steps=%s",
            step + 1,
            {group_id: len(steps) for group_id, steps in bridges.items()},
            start_steps,
        )
        # Wait until the session has sent the chunk and dated it (a few ms), not until it is due.
        while any(
            self._sessions[group_id].scheduled_chunk_count < target
            for group_id, target in target_chunks.items()
        ):
            termination = self._termination_result(step, start_time, last_obs)
            if termination is not None:
                return termination
            if self._stop_event.is_set():
                return _result("stopped", step, start_time, last_obs)
            check_sessions(self._sessions)
            await asyncio.sleep(0.001)
        for group_id, policy_start_step in start_steps.items():
            # Asked for by this queue's own step index: the session drops leading waypoints
            # whose moment has passed, so indexing the sent request shifts this origin by
            # however many it dropped — on every chunk, compounding.
            policy_start_ms = self._sessions[group_id].scheduled_timestamp_for_step(
                policy_start_step
            )
            if policy_start_ms is None:
                continue
            # Policy waypoint zero already has an exact timestamp on NOVA's jogger timer.
            # Preserve it as the immutable queue origin.
            self._policy_timelines[group_id] = _PolicyTimeline(
                action_timestep=action.action_timestep,
                policy_dt_ms=action.dt_ms,
                server_timestamp_ms=float(policy_start_ms),
            )
        logger.info("Policy action timeline initialized: %s", self._policy_timelines)
        return None

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

    # -------------------------------------------------------------------------
    # Stale data
    # -------------------------------------------------------------------------

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

        self._log_policy_action(action, step)
        target_chunks = self._send(trimmed, from_observation=True, effects=action)
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
        if (
            bridges
            and isinstance(self._execution, ContinuousExecution)
            and action.action_timestep >= 0
        ):
            return await self._anchor_policy_timeline(
                action, bridges, target_chunks, step=step, start_time=start_time, last_obs=last_obs
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
            _StaleHold: An input missed its deadline and the run should retry.
            _StaleStop: An input missed its deadline and the run should end.
        """
        images = self._read_policy_images()
        self._log_first_image_shapes(images)
        self._last_obs = robot_states
        self._log_policy_observation(robot_states, images, step)

        if start_time is None:
            await self._prepare_policy(robot_states, images)
            self._freeze_setup_objects()
            start_time = time.monotonic()
            self.status.phase = Phase.EXECUTING
            self.status.message = "Running policy..."

        return await self._infer(robot_states, images), start_time

    def _freeze_setup_objects(self) -> None:
        """Keep full garbage collections from pausing the control loop.

        Everything allocated up to here (SDK clients, sessions, a loaded model) lives for the
        whole run, yet every full collection traverses it again: on a virtual UR10e that stopped
        the event loop, the policy loop and the state stream together, for 135 to 240 ms every
        7 s or so. ``gc.freeze()`` takes those objects out of collection, so collections only
        traverse what the loop allocates (gaps below 45 ms); the collector stays on. ``run()``
        unfreezes at the end, so a long-lived host still collects cycles left by finished runs.
        ``gc.unfreeze()`` releases everything frozen, including what the interpreter or the host
        froze; only a pre-forking server that froze objects for copy-on-write would notice.
        """
        gc.collect()
        gc.freeze()
        self._gc_frozen = True

    def _read_policy_images(self) -> dict[str, Any] | None:
        """Read one frame per camera, or report the channel as stale."""
        if not self._cameras.active:
            return None
        try:
            images = self._cameras.read()
        except RuntimeError as exc:
            self._on_stale_input(f"camera: {exc}", allow_hold=True)
            raise  # unreachable: _on_stale_input always raises
        self._stale_since = None
        return images

    async def _infer(
        self,
        robot_states: dict[str, RobotState],
        images: dict[str, Any] | None,
    ) -> ActionChunk:
        """Query the policy under the executor's inference deadline."""
        try:
            action = await self._get_policy_actions(robot_states, images)
        except TimeoutError:
            # The timed-out call is still running on a worker thread, so
            # retrying would re-enter a client whose previous call may still
            # complete and advance its timestep. HOLD is refused here; see
            # OnStale.
            self._on_stale_input(
                f"inference exceeded {self._inference_timeout_s:.0f}s", allow_hold=False
            )
            raise  # unreachable: _on_stale_input always raises
        self._stale_since = None
        return action

    def _on_stale_input(self, what: str, *, allow_hold: bool) -> None:
        """Turn a missed deadline into the declared response. Never returns.

        Args:
            what: Human-readable description, carried into the result reason.
            allow_hold: Whether ``HOLD`` is meaningful for this trigger. False
                for an inference deadline, whose timed-out call is still in
                flight on a worker thread.

        Raises:
            RuntimeError: ``on_stale`` is ``ABORT``.
            _StaleHold: Retry after a pause.
            _StaleStop: End the run, running out the accepted waypoints.
        """
        now = time.monotonic()
        if self._stale_since is None:
            self._stale_since = now

        if self._on_stale is OnStale.ABORT:
            msg = f"Stale policy input ({what})"
            raise RuntimeError(msg)

        if self._on_stale is OnStale.HOLD:
            if not allow_hold:
                logger.warning("Stale policy input (%s) cannot be held through; stopping.", what)
                raise _StaleStop(what)
            held_for = now - self._stale_since
            if held_for < self._hold_budget_s:
                logger.warning(
                    "Holding: stale policy input (%s), %.1fs of %.1fs budget used.",
                    what,
                    held_for,
                    self._hold_budget_s,
                )
                raise _StaleHold(what)
            logger.warning(
                "Stale policy input (%s) outlasted the %.1fs hold budget; stopping.",
                what,
                self._hold_budget_s,
            )

        raise _StaleStop(what)

    async def _drain_sessions(self) -> None:
        """Let the waypoints already accepted run out before the sessions stop."""
        for session in self._sessions.values():
            with contextlib.suppress(MotionError, EmergencyStopError, OSError, RuntimeError):
                await session.drain()

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

        Both halves are needed. The server clock passing a chunk's last commanded
        timestamp says the deadline elapsed, not that the robot came to rest — it
        is still braking through the hold padding that follows. Standstill alone
        is just as insufficient: before a chunk's waypoints are reached the robot
        is legitimately still, so it would end the wait before any motion began.

        Standstill is measured from the joint positions
        (:attr:`WaypointJoggingSession.is_at_standstill`) rather than taken from
        the jogging state stream. NOVA reports no "commanded waypoints ran out"
        state: since 26.6 ``PAUSED_BY_USER`` follows a Pause/Stop *request* only,
        and this executor never sends one, so a drained queue reports ``RUNNING``
        indefinitely. It is still accepted here when it does appear, which is a
        genuine standstill by any other route.
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
    # Session operations (inlined from former PolicyRunner)
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
                io_values=self._all_io_values,
                target_joints=target_joints,
                target_ios=target_ios,
            )
            for condition in self._stop_conditions:
                if condition(ctx):
                    return getattr(condition, "__name__", repr(condition))
        return None

    def _send(
        self,
        chunk: ActionChunk,
        *,
        from_observation: bool = False,
        effects: ActionChunk | None = None,
        start_steps: Mapping[str, int] | None = None,
    ) -> dict[str, int]:
        """Send an action chunk to the motion groups.

        Placement is decided per session by :func:`novapolicy.chunking.placement`.
        ``from_observation`` says the chunk answers the latest observation as the policy
        returned it (no bridge in front), so continuous execution dates it by that observation.

        Contract: the IO and computed actions of ``effects`` (the policy's chunk) run when the
        policy's first step is due: step ``start_steps[group]`` of what is sent, 0 without a
        bridge. A group with IO but no motion, and computed actions of a chunk without motion,
        run at once.
        """
        place = placement(chunk, execution=self._execution, from_observation=from_observation)
        # A one-step chunk placed ahead is replaced by the next step before it is due: padding
        # it with a hold would make the controller brake towards every step.
        hold = not (
            place.lead
            and is_single_step(chunk)
            and isinstance(self._execution, ContinuousExecution)
        )
        effects = effects if effects is not None else chunk
        moving = [*chunk.joints, *chunk.tcp]
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
                action_timestep=chunk.action_timestep,
                extend_buffer=hold,
                lead=place.lead and server_timestamp_ms is None,
                observed_at_ms=self._observed_ms(group_id, chunk) if place.lead else None,
                when_due=self._effects_when_due(
                    effects, group_id, (start_steps or {}).get(group_id, 0), moving
                ),
                retained_prefix_steps=chunk.retained_prefix_steps,
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
                action_timestep=chunk.action_timestep,
                extend_buffer=hold,
                lead=place.lead and server_timestamp_ms is None,
                observed_at_ms=self._observed_ms(group_id, chunk) if place.lead else None,
                when_due=self._effects_when_due(
                    effects, group_id, (start_steps or {}).get(group_id, 0), moving
                ),
                retained_prefix_steps=chunk.retained_prefix_steps,
            )
            target_chunks[group_id] = session.queued_chunk_count
        self._schedule_ios(effects, skip=set(moving))
        if not moving and self._schema.has_computed_actions:
            task = asyncio.create_task(self._schema.run_computed_actions(effects))
            self._io_tasks.add(task)
            task.add_done_callback(self._io_tasks.discard)
        return target_chunks

    def _observed_ms(self, group_id: str, chunk: ActionChunk) -> ServerMs | None:
        """When the observation behind ``chunk`` was taken, on the server's clock; ``None`` in
        sequential execution, where the send is the reference (the robot was at rest)."""
        if isinstance(self._execution, SequentialExecution):
            return None
        if is_single_step(chunk):
            return self._observed_at_ms.get(group_id)
        return self._observed_clock_ms.get(group_id)

    def _effects_when_due(
        self, effects: ActionChunk, group_id: str, step: int, moving: list[str]
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

        return (step, run) if ios or computed else None

    def _schedule_ios(self, chunk: ActionChunk, *, skip: set[str] | None = None) -> None:
        """Write the chunk's IO at once, except for the groups in ``skip``."""
        for group_id, ios in (chunk.ios or {}).items():
            session = self._sessions.get(group_id)
            if session is None or group_id in (skip or set()):
                continue
            task = asyncio.create_task(session.write_ios(ios))
            self._io_tasks.add(task)
            task.add_done_callback(self._io_tasks.discard)

    def _policy_server_timestamp_ms(
        self,
        chunk: ActionChunk,
        group_id: str,
    ) -> ServerMs | None:
        """Map a policy action timestep onto its immutable NOVA timeline."""
        timeline = self._policy_timelines.get(group_id)
        if chunk.action_timestep < 0 or timeline is None:
            return None
        elapsed_steps = chunk.action_timestep - timeline.action_timestep
        return ServerMs(int(timeline.server_timestamp_ms + elapsed_steps * timeline.policy_dt_ms))

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

    @property
    def _all_io_values(self) -> dict[str, object]:
        """Merged IO values across all controller streams."""
        return self._io_streams.all_values

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
