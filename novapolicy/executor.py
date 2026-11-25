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

from novapolicy.cameras.manager import CameraManager
from novapolicy.mapping.relative import apply_relative_mode
from novapolicy.pipeline.base import split_pipeline
from novapolicy.pipeline.defaults import default_pipeline
from novapolicy.pipeline.modules.endpoint_ramp import EndpointRamp
from novapolicy.pipeline.modules.timeline import TimestepTimeline
from novapolicy.pipeline.placement import is_single_step, placement, trim_chunk
from novapolicy.providers.base import PolicyClient
from novapolicy.transport.io import IOStreamManager
from novapolicy.transport.lifecycle import SessionLifecycle, move_to_start_joint_position
from novapolicy.transport.state import first_fired
from novapolicy.types import (
    ActionChunk,
    ActionChunkConfig,
    ContinuousExecution,
    EmergencyStopError,
    ExecutionMode,
    MotionError,
    SequentialExecution,
    StopContext,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Coroutine, Sequence

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.mapping.schema import PolicySchema
    from novapolicy.rerun import PolicyRerunLogger
    from novapolicy.transport.clock import ServerMs
    from novapolicy.transport.session import WaypointJoggingSession
    from novapolicy.types import StopCondition

logger = logging.getLogger(__name__)

_ASYNC_REPLACEMENT_LEAD_STEPS = 1
_DEFAULT_ACTION_CHUNK_CONFIG = ActionChunkConfig()
_DEFAULT_EXECUTION = SequentialExecution()

# Default freshness bound for the policy's camera frames, and the deadline for one inference
# (ADR 0018).
_DEFAULT_CAMERA_MAX_AGE_S = 1.0
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
    """Raise ``ValueError`` for a setup the executor would run wrongly (contracts T12); warn
    for one it runs as asked but rarely as meant."""
    if [type(m) for m in pipeline] == [type(m) for m in default_pipeline()]:
        logger.warning(
            "The pipeline is the joggers' default (default_pipeline() without arguments): no "
            "Bridge or JointClamp. For the executor use default_pipeline(execution=..., policy=...)"
        )
    queue = any(isinstance(m, TimestepTimeline) for m in policy.default_modules)
    if not isinstance(execution, SequentialExecution):
        if any(isinstance(m, EndpointRamp) for m in pipeline):
            logger.warning(
                "EndpointRamp brings every chunk to rest, so in ContinuousExecution the arm stops "
                "at each replacement; leave it out unless that is meant"
            )
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


def resolve_actions(
    schema: PolicySchema, chunk: ActionChunk, states: dict[str, RobotState]
) -> ActionChunk:
    """A policy's raw chunk as the executor sends it (contracts M4): inverse operators, relative
    targets composed with ``states``, TCP targets in the world. For dry runs, after
    ``await schema.load_coordinate_systems()``."""
    chunk = schema.apply_inverse_ops(chunk)  # NOVA units, rotation vectors, policy frames
    chunk = apply_relative_mode(
        chunk,
        schema.policy_frame_states(states),
        schema.relative_motion_groups(),
        schema.relative_tcp_deltas(),
    )
    return schema.tcp_targets_to_world(chunk)


class PolicyExecutor:
    """Runs one policy episode: observe → query policy → send actions → repeat, until the
    timeout, a stop condition or :meth:`stop`; the policy never signals "done"."""

    def __init__(
        self,
        schema: PolicySchema,
        policy: PolicyClient,
        *,
        stop_conditions: list[StopCondition] | None = None,
        timeout_s: float = 0,
        camera_max_age_s: float = _DEFAULT_CAMERA_MAX_AGE_S,
        inference_timeout_s: float = _DEFAULT_INFERENCE_TIMEOUT_S,
        motion: ActionChunkConfig = _DEFAULT_ACTION_CHUNK_CONFIG,
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
            timeout_s: Maximum execution duration [s]; 0 = no timeout.
            camera_max_age_s: Default freshness bound for the policy's camera frames [s]; a
                channel overrides it with ``Observation.image(..., max_age_s=...)``. An older
                frame ends the run with ``RuntimeError``.
            inference_timeout_s: Deadline for one ``get_actions`` call [s]; 0 = none. A
                liveness guard, not a real-time bound.
            motion: Waypoint jogging configuration.
            execution: ``SequentialExecution`` settles each chunk before the next inference;
                ``ContinuousExecution`` replaces the lookahead as fast as possible or at its rate.
            n_action_steps: Steps executed from each chunk: ``None`` the policy's own horizon if
                it declares one, else all; ``0`` all; ``N > 0`` the first N (contracts M6).
            start_joint_position: Joint positions per motion group to PTP-move to first.
            pipeline: The modules between the policy's chunk and NOVA, in order, exactly as
                given; ``None`` runs :func:`~novapolicy.pipeline.defaults.default_pipeline`. To
                add one, extend that list with the same execution and policy, e.g.
                ``[*default_pipeline(execution=execution, policy=policy), EaseIn(duration_s=1.0)]``.
                Each motion group runs its own copy.
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

        self._lifecycle = SessionLifecycle(
            config=motion, stop_conditions=self._stop_conditions, pipeline=self._pipeline
        )
        self._sessions: dict[str, WaypointJoggingSession] = self._lifecycle.sessions
        self._cameras = CameraManager(camera_max_age_s)
        self._stop_event = asyncio.Event()
        self._last_obs: dict[str, Any] | None = None
        self._io_streams = IOStreamManager(self._motion_groups, schema.io_keys_by_controller())
        self._io_tasks: set[asyncio.Task[None]] = set()
        self._effect_error: BaseException | None = None
        """The first error of an IO write or computed action run without motion; ends the run."""
        self._observed_at_ms: dict[str, ServerMs | None] = {}
        self._observed_clock_ms: dict[str, ServerMs] = {}
        self._gc_frozen = False
        self._start_time: float | None = None
        """When the first inference ended, after the policy's preparation; ``None`` before."""
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
        """Run one episode, blocking until it ends; an error sets ``Phase.ERROR`` and re-raises.

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
        self._effect_error = None
        self.status = ExecutorStatus(phase=Phase.CONNECTING, message="Connecting...")
        try:
            await self._run_episode()
        except Exception as exc:
            typed = isinstance(exc, (MotionError, EmergencyStopError))
            message = str(exc) if typed else f"{type(exc).__name__}: {exc}"
            self.status = ExecutorStatus(phase=Phase.ERROR, step=self.status.step, message=message)
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
        """Outer teardown, always run: pending IO write tasks, the policy connection, the
        final status and result logging."""
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

    async def _run_episode(self) -> None:
        """Create the sessions, run the observe-act loop and tear down: after a normal end the
        sessions drain, after an error they stop at once (contracts T10)."""
        for group_id, session in self._build_sessions().items():
            session.add_state_observer(
                lambda state, ts_ms, _generated, group_id=group_id: self._schema.observe_state(
                    group_id, state.joints, ts_ms / 1000.0
                )
            )

        failed = False
        try:
            await self._start()
            await self._complete(await self._execute())
        except Exception:
            failed = True
            raise
        finally:
            if self._gc_frozen:
                gc.unfreeze()
                self._gc_frozen = False
            await self._lifecycle.close(drain=not failed)
            if self._cameras.active:
                await self._cameras.disconnect()
            await self._schema.stop_sources()
            await self._io_streams.stop()

    async def _complete(self, result: ExecutionResult) -> None:
        """End a run that ended normally: let the effects already started finish (one that
        raises ends the run with its error after all, and no result), then keep the result and
        report completion."""
        await self._finish_effects()
        self.result = result
        if self._rerun is not None:
            self._rerun.log_completion(result.reason, result.steps, result.duration_s)
        self.status.phase = Phase.COMPLETED

    async def _start(self) -> None:
        """Connect the policy and let it check the schema, move to the start position, connect
        cameras, IO and sources, and start the sessions with their monitoring; returns once the
        server acknowledged them. A schema the policy refuses fails before anything moves
        (ADR 0014)."""
        await self._policy.connect(self.mg_ids)
        await self._policy.validate_schema(self._schema)
        await move_to_start_joint_position(self._start_joint_position)
        image_sources = self._schema.image_sources
        if image_sources:
            logger.info("Connecting cameras...")
            await self._cameras.connect(image_sources, self._schema.image_max_age_s)
            logger.info("All cameras ready")
        await self._lifecycle.start_sessions()
        await self._io_streams.start()
        self._io_streams.wire_to_sessions(self._sessions)
        await self._schema.load_coordinate_systems()
        await self._schema.start_sources()
        self._rerun = await self._lifecycle.start_monitoring(
            camera_names=self._cameras.names,
            tcp=self._schema.tcp,
            image_reader=self._cameras.read_latest_frames if self._cameras.active else None,
        )
        await self._lifecycle.wait_ready()

    # -------------------------------------------------------------------------
    # Observe → act loop
    # -------------------------------------------------------------------------

    async def _execute(self) -> ExecutionResult:
        """Run the observe-act loop until termination.

        Raises MotionError, EmergencyStopError directly.
        """
        self._start_time = None
        self._last_obs = None
        self.status.message = "Preparing policy..."

        fixed_rate_period = (
            1.0 / self._execution.rate_hz
            if isinstance(self._execution, ContinuousExecution)
            and self._execution.rate_hz is not None
            else 0.0
        )
        tick_due: float | None = None  # when the current tick was due on the fixed-rate grid

        while not self._stop_event.is_set():
            # Yield every iteration, so stop() runs even when the policy answers synchronously.
            await asyncio.sleep(0)

            self._check_effects()
            termination = self._termination_result()
            if termination is not None:
                return termination

            tick_start = time.monotonic()

            robot_states = self._observe()
            if not robot_states:
                await asyncio.sleep(0.01)
                continue
            action = await self._observe_and_infer(robot_states)
            if self.status.step == 0 and self._start_time is not None:
                tick_start = self._start_time  # the first tick starts after the preparation
            termination = await self._send_policy_action(action, robot_states)
            if termination is not None:
                return termination
            self.status.step += 1

            # Raises on a fault; a stop condition firing on a session tick ends the run normally.
            self._check_effects()
            stopped_by = self._lifecycle.check()
            if stopped_by is not None:
                return self._result(f"stop condition: {stopped_by}")

            tick_due = tick_start if tick_due is None else tick_due
            tick_due = await self._wait_after_send(tick_due, fixed_rate_period)

        return self._result("stopped")

    def _result(self, reason: str) -> ExecutionResult:
        """The run's result, ending now: its steps, the time since :attr:`_start_time` and the
        last observation."""
        now = time.monotonic()
        start = self._start_time if self._start_time is not None else now
        return ExecutionResult(
            reason=reason, steps=self.status.step, duration_s=now - start, last_state=self._last_obs
        )

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
        self, action: ActionChunk, robot_states: dict[str, RobotState]
    ) -> ExecutionResult | None:
        """Trim, place and send one chunk. Returns a result when the run ends here."""
        trimmed = trim_chunk(action, self._n_action_steps)
        stopped_by = self._check_stop_conditions_pre_send(trimmed, robot_states)
        if stopped_by is not None:
            return self._result(f"stop condition: {stopped_by}")

        step = self.status.step
        if self._rerun is not None:
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

    async def _observe_and_infer(self, robot_states: dict[str, RobotState]) -> ActionChunk:
        """Read the cameras, prepare on the first tick, and query the policy.

        Sets :attr:`_start_time` on the first tick, after policy preparation, so model loading
        is excluded from the execution timeout.

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
            self._rerun.log_observation(robot_states, self.status.step)
            if images:
                self._rerun.log_images(images)

        if self._start_time is None:
            await self._policy.prepare(
                robot_states, self._schema, images, self._io_streams.all_values or None
            )
            self._freeze_setup_objects()
            self._start_time = time.monotonic()
            self.status.phase = Phase.EXECUTING
            self.status.message = "Running policy..."

        return await self._infer(robot_states, images)

    def _freeze_setup_objects(self) -> None:
        """Collect, then freeze everything allocated so far out of garbage collection until
        :meth:`_run_episode` unfreezes it (ADR 0007)."""
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
            if action.resolved:
                return action
            return resolve_actions(self._schema, action, robot_states)
        except TimeoutError as exc:
            msg = f"Stale policy input (inference exceeded {self._inference_timeout_s:.0f}s)"
            raise RuntimeError(msg) from exc

    def _termination_result(self) -> ExecutionResult | None:
        if self._timed_out():
            return self._result("timeout")
        stopped_by = self._lifecycle.stop_condition
        if stopped_by is not None:
            return self._result(f"stop condition: {stopped_by}")
        return None

    def _timed_out(self) -> bool:
        """Whether the run has used up ``timeout_s`` since :attr:`_start_time`."""
        start = self._start_time
        return start is not None and 0 < self._timeout_s <= time.monotonic() - start

    async def _wait_after_send(self, tick_due: float, fixed_rate_period: float) -> float:
        """Pace the loop after a send; returns when the next tick is due.

        Sequential execution waits for the sessions to settle. A fixed rate sleeps until the next
        tick on an absolute grid, which restarts when more than a period behind (ADR 0006).
        """
        if isinstance(self._execution, SequentialExecution):
            await self._wait_until_sessions_settled()
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

    async def _wait_until_sessions_settled(self) -> None:
        """Wait until every queued chunk was sent, its last step is past and the robot stands
        still (or reports ``PAUSED_BY_USER``); see executor.md, "Detecting standstill"."""
        target_chunks = {
            group_id: session.queued_chunk_count for group_id, session in self._sessions.items()
        }
        while True:
            if self._stop_event.is_set():
                return
            if self._timed_out():
                return
            if self._effect_error is not None or self._lifecycle.check() is not None:
                return

            if all(
                session.scheduled.chunk_count >= target_chunks[group_id]
                and session.last_server_timestamp_ms >= session.scheduled.until_server_ms
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
                # When the observation was taken on the session clock: the clock estimate for
                # multi-step chunks, the state's own timestamp for one-step ones (ADR 0003).
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
        """Evaluate stop conditions against the chunk's targets and IO before it is sent; returns
        the name of the first condition that fired, or ``None``."""
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
            fired = first_fired(self._stop_conditions, ctx)
            if fired is not None:
                return fired
        return None

    def _send(self, chunk: ActionChunk, effects: ActionChunk) -> None:
        """Hand an action chunk to each motion group's session, placed by
        :func:`novapolicy.pipeline.placement.placement` (the session's modules may still move it).

        The IO and computed actions of ``effects`` (the policy's chunk) run when the policy's
        first step is due: a moving group's IO with its own step, the computed actions with the
        first moving group's; the rest at once (contracts T7). IO is written before the
        computed actions.
        """
        place = placement(chunk, execution=self._execution)
        moving = [*chunk.joints, *chunk.tcp]
        # Every group's steps first: a bad one must not leave another group's chunk queued.
        for group_id, steps in [*chunk.joints.items(), *chunk.tcp.items()]:
            session = self._sessions.get(group_id)
            if session is not None:
                session.check_steps(steps)
        for group_id, steps in [*chunk.joints.items(), *chunk.tcp.items()]:
            session = self._sessions.get(group_id)
            if session is None:
                logger.warning("Unknown motion group in chunk: %s", group_id)
                continue
            run = self._effects(effects, [group_id], computed=group_id == moving[0])
            session.update_chunk(
                steps=steps,
                dt_ms=chunk.dt_ms,
                first_timestamp_ms=place.first_timestamp_ms,
                action_timestep=chunk.action_timestep,
                extend_buffer=place.hold,
                lead=place.lead,
                observed_at_ms=self._observed_ms(group_id, chunk) if place.lead else None,
                when_due=None if run is None else (0, run),
                retained_prefix_steps=chunk.retained_prefix_steps,
            )
        idle = [group_id for group_id in effects.ios or {} if group_id not in moving]
        run = self._effects(effects, idle, computed=not moving)
        if run is not None:
            task = asyncio.create_task(run())
            self._io_tasks.add(task)
            task.add_done_callback(self._effect_done)

    def _effect_done(self, task: asyncio.Task[None]) -> None:
        """Forget a finished effects task, keeping its error, the first one only, for
        :meth:`_check_effects` (contracts T7: a hook that raises ends the run)."""
        self._io_tasks.discard(task)
        if task.cancelled() or self._effect_error is not None:
            return
        self._effect_error = task.exception()

    def _check_effects(self) -> None:
        """Raise the error of an IO write or computed action that ran without motion."""
        if self._effect_error is not None:
            raise self._effect_error

    async def _finish_effects(self) -> None:
        """At a normal end, let the effects already started finish and raise the first error,
        so one of the last tick ends the run like any other."""
        tasks = list(self._io_tasks)
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        for task in tasks:
            self._effect_done(task)
        self._check_effects()

    def _observed_ms(self, group_id: str, chunk: ActionChunk) -> ServerMs | None:
        """When the observation behind ``chunk`` was taken, on the server's clock; ``None`` in
        sequential execution, where the send is the reference (the robot was at rest)."""
        if isinstance(self._execution, SequentialExecution):
            return None
        if is_single_step(chunk):
            return self._observed_at_ms.get(group_id)
        return self._observed_clock_ms.get(group_id)

    def _effects(
        self, effects: ActionChunk, group_ids: list[str], *, computed: bool
    ) -> Callable[[], Coroutine[Any, Any, None]] | None:
        """Write the IO of ``group_ids``, then, if ``computed``, run the chunk's computed
        actions (interim, ADR 0005); ``None`` when there is nothing to run."""
        writes = [
            (self._sessions[group_id], ios)
            for group_id in group_ids
            if group_id in self._sessions and (ios := (effects.ios or {}).get(group_id))
        ]
        computed = computed and self._schema.has_computed_actions
        if not writes and not computed:
            return None

        async def run() -> None:
            for session, ios in writes:
                await session.write_ios(ios)
            if computed:
                await self._schema.run_computed_actions(effects)

        return run

    def _build_sessions(self) -> dict[str, WaypointJoggingSession]:
        """A fresh session per motion group: TCP poses for the schema's TCP action groups,
        joints for the rest."""
        tcp_groups = self._schema.tcp_action_groups()
        return self._lifecycle.build({
            mg: (
                "cartesian" if mg.id in tcp_groups else "joint",
                tcp_groups.get(mg.id) or self._schema.tcp,
            )
            for mg in self._motion_groups
        })

    @property
    def _n_action_steps(self) -> int:
        """Steps executed from each chunk (0 = all): the executor's argument, ``0`` included,
        else the policy's own horizon."""
        if self._n_action_steps_cfg is not None:
            return self._n_action_steps_cfg
        return self._policy.n_action_steps or 0
