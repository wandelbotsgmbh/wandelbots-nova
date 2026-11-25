"""Behavioural tests for PolicyExecutor.

The executor's job is to run a policy against one or more robots and report
*why* it stopped. These tests state that contract through the public surface
(``run()`` -> ``ExecutionResult``), substituting only the robot transport
(``ActionChunkSession``) and the e-stop monitor, since there is no real
robot in a unit test. Nothing reaches into the executor's private fields.
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable, Iterator
import contextlib
from dataclasses import dataclass
import gc
import math
import time
from types import SimpleNamespace
from typing import TYPE_CHECKING
from unittest.mock import AsyncMock, MagicMock, patch

import numpy as np
import pytest

from novapolicy.executor import Phase, PolicyExecutor
from novapolicy.mapping.ops import Rad2Deg
from novapolicy.mapping.schema import Action, Observation, ObservationEntry, PolicySchema
from novapolicy.pipeline.base import ModuleContext, split_pipeline
from novapolicy.pipeline.defaults import default_pipeline, timestep_queue_modules
from novapolicy.pipeline.modules.endpoint_ramp import EndpointRamp
from novapolicy.pipeline.modules.smoothing import Smoothing
from novapolicy.providers.base import CallbackPolicyClient, PolicyClient
from novapolicy.providers.queued import QueuedPolicyClient
from novapolicy.tests.doubles import fake_session as _fake_session, mg as _mg
from novapolicy.transport.action_chunks import PendingChunk
from novapolicy.transport.clock import ServerMs
from novapolicy.transport.session import Scheduled
from novapolicy.types import (
    ActionChunk,
    ActionChunkConfig,
    ActionMode,
    ContinuousExecution,
    EmergencyStopError,
    MotionError,
    SequentialExecution,
    StopContext,
)

if TYPE_CHECKING:
    from typing import Any

    from numpy.typing import NDArray

    from nova.types import RobotState

MG_ID = "0@ur10e"


# ---------------------------------------------------------------------------
# Test doubles: a single-arm schema and a fake robot transport.
# ---------------------------------------------------------------------------


def _schema(mode: ActionMode = "absolute") -> PolicySchema:
    obs: list[ObservationEntry] = [Observation.joint_positions("arm", source=_mg(), mode=mode)]
    return PolicySchema(observations=obs)


async def _hold_action(_obs: object) -> ActionChunk:
    """A trivial policy: hold all six joints at zero."""
    return ActionChunk(joints={MG_ID: [[0.0] * 6]})


def _callback(fn: Callable[[object], Awaitable[ActionChunk]]) -> CallbackPolicyClient:
    return CallbackPolicyClient(fn)


async def _wait_until(predicate: Callable[[], bool], *, timeout: float = 0.5) -> None:
    async def poll() -> None:
        while not predicate():
            await asyncio.sleep(0)

    await asyncio.wait_for(poll(), timeout=timeout)


_hold = _callback(_hold_action)


class _TestPolicy(CallbackPolicyClient):
    def __init__(
        self,
        fn: Callable[[object], Awaitable[ActionChunk]],
        *,
        requires_bridge: bool = False,
    ) -> None:
        super().__init__(fn)
        self._requires_bridge = requires_bridge
        self.synchronize_action_timestep = MagicMock()

    @property
    def default_modules(self) -> list[object]:
        return timestep_queue_modules() if self._requires_bridge else []


_SLOW_SETUP_S = 1.0


class _SlowSetupPolicy(PolicyClient):
    def __init__(self) -> None:
        self.prepare_calls = 0
        self.phase_during_prepare: object | None = None
        self.executor: PolicyExecutor | None = None

    async def connect(self, motion_group_ids: list[str]) -> None:
        pass

    async def validate_schema(self, schema: PolicySchema) -> None:
        pass

    async def prepare(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> None:
        self.prepare_calls += 1
        self.phase_during_prepare = self.executor.phase if self.executor is not None else None
        await asyncio.sleep(_SLOW_SETUP_S)

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        return ActionChunk(joints={MG_ID: [[0.0] * 6]}, dt_ms=1.0)

    async def close(self) -> None:
        pass


def _unsmoothed_queue_defaults() -> list[object]:
    """A timestep queue's continuous default pipeline without smoothing."""

    class _Queue:
        default_modules = timestep_queue_modules()

    return [
        m
        for m in default_pipeline(execution=ContinuousExecution(), policy=_Queue())
        if not isinstance(m, Smoothing)
    ]


def _without_ramp() -> list[object]:
    """The sequential default pipeline without its endpoint ramp."""
    return [
        m
        for m in default_pipeline(execution=SequentialExecution())
        if not isinstance(m, EndpointRamp)
    ]


def _after_chunk_modules(
    executor: PolicyExecutor,
    kwargs: dict[str, Any],
    *,
    position: list[float] | None = None,
    scheduled_start: tuple[int, ServerMs] | None = None,
) -> PendingChunk:
    """What the session's chunk modules make of what the executor handed it, from a robot
    standing at ``position`` (all zeros by default)."""
    steps = kwargs["steps"]
    chunk = PendingChunk(
        steps=[list(s) for s in steps],
        dt_ms=kwargs["dt_ms"],
        first_timestamp_ms=kwargs.get("first_timestamp_ms"),
        timestamp_offset_steps=kwargs.get("timestamp_offset_steps", 0),
        caller_step_count=len(steps),
        lead=kwargs.get("lead", False),
        observed_at_ms=kwargs.get("observed_at_ms"),
        extend_buffer=kwargs.get("extend_buffer", True),
        when_due=kwargs.get("when_due"),
        action_timestep=kwargs.get("action_timestep", -1),
    )
    context = ModuleContext(
        now_ms=ServerMs(0),
        position=[0.0] * len(steps[0]) if position is None else position,
        step_ms=50.0,
        motion_group_id=MG_ID,
        scheduled_start=scheduled_start,
    )
    for module in split_pipeline(executor._pipeline).chunk_modules:
        chunk = module.on_chunk(chunk, context)
    return chunk


@dataclass
class _Robot:
    """The faked robot the executor talks to during a test."""

    session: MagicMock
    estop: MagicMock


@pytest.fixture(autouse=True)
def _disable_rerun() -> Iterator[None]:
    """Keep executor behavior independent of Rerun state left by other suites."""
    with patch("novapolicy.rerun._is_rerun_active", return_value=False):
        yield


@contextlib.contextmanager
def _patched_robot(**session_class: object) -> Iterator[MagicMock]:
    """Patch the executor's robot transport (configured by ``session_class``) and e-stop
    monitor; yields the e-stop monitor."""
    estop = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)
    with (
        patch("novapolicy.transport.lifecycle.ActionChunkSession", **session_class),
        patch("novapolicy.transport.lifecycle.EstopMonitor", return_value=estop),
    ):
        yield estop


@pytest.fixture
def robot() -> Iterator[_Robot]:
    """Patch the executor's robot transport and e-stop monitor for the test.

    The patches are active for the whole test, so ``executor.run()`` picks up
    the fakes. A test configures the robot by mutating ``robot.session`` (e.g.
    ``has_failed``) or ``robot.estop.error`` before calling ``run()``.
    """
    session = _fake_session()
    with _patched_robot(return_value=session) as estop:
        yield _Robot(session=session, estop=estop)


async def _send_once(
    robot: _Robot,
    chunk: ActionChunk,
    *,
    schema: PolicySchema | None = None,
    requires_bridge: bool = False,
    delay_s: float = 0.0,
    **kwargs: Any,
) -> tuple[PolicyExecutor, dict[str, Any]]:
    """Run until the policy (answering ``chunk`` once, after ``delay_s``) is sent; returns the
    executor and the keyword arguments of its first ``update_chunk``."""
    executor: PolicyExecutor

    async def policy(_obs: object) -> ActionChunk:
        await asyncio.sleep(delay_s)
        executor.stop()
        return chunk

    client = _TestPolicy(policy, requires_bridge=True) if requires_bridge else _callback(policy)
    executor = PolicyExecutor(schema or _schema(), client, **{"timeout_s": 1.0, **kwargs})
    await executor.run()
    return executor, robot.session.update_chunk.call_args_list[0].kwargs


# ---------------------------------------------------------------------------
# Why a run ends: timeout / stop() / plain-function policy
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_run_ends_with_timeout_once_the_deadline_passes(robot: _Robot):
    """With timeout_s set, the executor stops itself and reports 'timeout'."""
    executor = PolicyExecutor(_schema(), _hold, motion=ActionChunkConfig(), timeout_s=0.2)
    result = await executor.run()
    assert result.reason == "timeout"
    assert result.steps > 0


async def _sequential_second_inference(
    robot: _Robot,
) -> tuple[asyncio.Task[object], asyncio.Event, Callable[[], int]]:
    """Start a sequential run that stops itself on its second inference.

    Returns the run task, an event set when that second inference happens, and a
    reader for the inference count, so each test only has to script the session
    signals it cares about.
    """
    inference_count = 0
    second_inference = asyncio.Event()
    executor: PolicyExecutor

    async def policy(_obs: object) -> ActionChunk:
        nonlocal inference_count
        inference_count += 1
        if inference_count == 2:
            second_inference.set()
            executor.stop()
        return ActionChunk(joints={MG_ID: [[0.0] * 6]}, dt_ms=10.0)

    executor = PolicyExecutor(_schema(), _callback(policy), timeout_s=1.0)
    run_task = asyncio.create_task(executor.run())
    await _wait_until(lambda: robot.session.update_chunk.call_count > 0)
    await asyncio.sleep(0.05)
    assert inference_count == 1
    return run_task, second_inference, lambda: inference_count


@pytest.mark.asyncio
async def test_sequential_mode_delays_next_inference_until_deadline_and_standstill(robot: _Robot):
    """Sequential mode waits for the chunk's final timestamp *and* a standstill."""
    run_task, second_inference, inference_count = await _sequential_second_inference(robot)

    before = inference_count()
    robot.session.scheduled = Scheduled(chunk_count=1, until_server_ms=ServerMs(100))
    robot.session.is_at_standstill = True
    robot.session.last_server_timestamp_ms = 99
    await asyncio.sleep(0.03)
    assert inference_count() == before  # standstill alone is not enough

    robot.session.last_server_timestamp_ms = 100
    await asyncio.wait_for(second_inference.wait(), timeout=0.5)
    _ = await run_task
    assert inference_count() == 2


@pytest.mark.asyncio
async def test_sequential_mode_waits_out_the_braking_ramp_after_the_deadline(robot: _Robot):
    """A passed deadline with the robot still moving does not release the wait.

    The server clock reaching the last commanded timestamp only means the
    deadline elapsed; the robot is still braking through the hold padding that
    follows. Inferring here observes a moving robot, which is the one thing
    sequential execution exists to avoid.
    """
    run_task, second_inference, inference_count = await _sequential_second_inference(robot)

    before = inference_count()
    robot.session.scheduled = Scheduled(chunk_count=1, until_server_ms=ServerMs(100))
    robot.session.last_server_timestamp_ms = 150  # deadline well past
    robot.session.is_at_standstill = False  # ...but still braking
    await asyncio.sleep(0.05)
    assert inference_count() == before

    robot.session.is_at_standstill = True
    await asyncio.wait_for(second_inference.wait(), timeout=0.5)
    _ = await run_task
    assert inference_count() == 2


@pytest.mark.parametrize(
    ("jogging_state", "at_standstill"),
    [("RUNNING", True), ("PAUSED_BY_USER", False)],
    ids=["standstill_while_only_running", "server_reported_pause"],
)
@pytest.mark.asyncio
async def test_sequential_mode_settles_on_a_standstill_or_a_server_reported_pause(
    robot: _Robot, jogging_state: str, at_standstill: bool
):
    """A jogger that never leaves RUNNING must not stall sequential execution, and a
    server-reported ``PAUSED_BY_USER`` still ends the wait on its own.

    Regression test for NOVA 26.6: ``PAUSED_BY_USER`` is reported only in response to a
    Pause/Stop request, which this executor never sends, so a drained queue reports
    ``RUNNING`` forever; requiring that state hung every episode after one chunk.
    """
    run_task, second_inference, inference_count = await _sequential_second_inference(robot)

    robot.session.scheduled = Scheduled(chunk_count=1, until_server_ms=ServerMs(100))
    robot.session.last_server_timestamp_ms = 100
    robot.session.jogging_state = jogging_state
    robot.session.is_at_standstill = at_standstill

    await asyncio.wait_for(second_inference.wait(), timeout=0.5)
    _ = await run_task
    assert inference_count() == 2


@pytest.mark.asyncio
async def test_bridge_and_policy_are_sent_as_one_continuous_chunk(robot: _Robot):
    """Bridge and policy waypoints share one request with no standstill at their boundary."""
    inference_count = 0
    executor: PolicyExecutor

    async def policy(_obs: object) -> ActionChunk:
        nonlocal inference_count
        inference_count += 1
        if inference_count == 2:
            executor.stop()
        return ActionChunk(
            joints={MG_ID: [[3.0] * 6, [4.0] * 6, [5.0] * 6]},
            dt_ms=10.0,
        )

    def acknowledge_chunk(*_args: object, **kwargs: object) -> None:
        robot.session.queued_chunk_count += 1
        count = len(kwargs["steps"])
        stamps = tuple(range(100, 100 * (count + 1), 100))
        robot.session.scheduled = Scheduled(
            chunk_count=robot.session.queued_chunk_count,
            until_server_ms=ServerMs(stamps[-1]),
            waypoint_timestamps=stamps,
        )
        robot.session.last_server_timestamp_ms = stamps[-1]
        robot.session.jogging_state = "PAUSED_BY_USER"

    robot.session.update_chunk.side_effect = acknowledge_chunk
    executor = PolicyExecutor(
        _schema(),
        _callback(policy),
        timeout_s=1.0,
        execution=SequentialExecution(),
        pipeline=_without_ramp(),
    )

    await executor.run()

    # The executor hands over the policy's steps; the session's Bridge connects them.
    first_send = robot.session.update_chunk.call_args_list[0].kwargs
    assert first_send["steps"] == [[3.0] * 6, [4.0] * 6, [5.0] * 6]
    assert first_send["dt_ms"] == 10.0
    bridged = _after_chunk_modules(executor, first_send)
    assert bridged.start_step == 3
    assert [s[0] for s in bridged.steps[: bridged.start_step + 1]] == [0.0, 1.0, 2.0, 3.0]


@pytest.mark.parametrize(
    ("steps", "hold"),
    [
        ([[3.0] * 6], False),  # one step per inference: no braking hold
        ([[3.0] * 6, [4.0] * 6], True),  # a chunk keeps its hold padding
    ],
)
@pytest.mark.asyncio
async def test_continuous_chunks_are_dated_by_their_observation(
    robot: _Robot, steps: list[list[float]], hold: bool
):
    robot.session.current_state_server_ms = 1_000  # when the state was measured
    robot.session.estimated_server_timestamp_ms = 1_006  # server clock at observation
    chunk = ActionChunk(joints={MG_ID: steps}, dt_ms=1000 / 30)
    _, first_send = await _send_once(robot, chunk, execution=ContinuousExecution())

    assert first_send["steps"] == steps  # handed over as is: the session bridges, if anything
    assert first_send["lead"] is True
    assert first_send["extend_buffer"] is hold
    # One step: the state's own timestamp. A chunk: the clock estimate at observation time.
    assert first_send["observed_at_ms"] == (1_006 if hold else 1_000)


@pytest.mark.asyncio
async def test_continuous_async_queue_policy_requests_a_measured_state_bridge(robot: _Robot):
    """Async ACT lookahead connects to the robot even though its executor rate is continuous."""
    executor, first_send = await _send_once(
        robot,
        ActionChunk(joints={MG_ID: [[0.5] * 6, [1.5] * 6, [2.5] * 6]}, dt_ms=10.0),
        requires_bridge=True,
        execution=ContinuousExecution(rate_hz=20),
        pipeline=_unsmoothed_queue_defaults(),
    )
    assert first_send["steps"] == [[0.5] * 6, [1.5] * 6, [2.5] * 6]
    # The session's Bridge puts a measured-state hold in front.
    assert _after_chunk_modules(executor, first_send).steps[:4] == [
        [0.0] * 6,
        [0.5] * 6,
        [1.5] * 6,
        [2.5] * 6,
    ]


@pytest.mark.asyncio
async def test_async_queue_replacements_preserve_the_initial_policy_timeline(robot: _Robot):
    """Only timestep zero bridges; later lookaheads retain absolute queue timing."""
    executor: PolicyExecutor
    call_count = 0

    async def get_actions(_obs: object) -> ActionChunk:
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            return ActionChunk(
                joints={MG_ID: [[0.5] * 6, [1.5] * 6, [2.5] * 6]},
                dt_ms=10.0,
                action_timestep=0,
            )
        if call_count == 2:
            return ActionChunk(
                joints={MG_ID: [[0.75] * 6, [1.75] * 6, [2.75] * 6]},
                dt_ms=10.0,
                action_timestep=2,
            )
        if call_count == 3:
            return ActionChunk(
                joints={MG_ID: [[1.0] * 6, [2.0] * 6, [3.0] * 6]},
                dt_ms=10.0,
                action_timestep=4,
            )
        executor.stop()
        return ActionChunk(
            joints={MG_ID: [[1.25] * 6, [2.25] * 6, [3.25] * 6]},
            dt_ms=10.0,
            action_timestep=6,
        )

    queued_chunks: list[PendingChunk] = []
    started: list[tuple[int, ServerMs]] = []  # what the session reports once a chunk is sent

    def acknowledge_chunk(*_args: object, **kwargs: Any) -> None:
        robot.session.queued_chunk_count += 1
        # What the session queues after its chunk modules: the Bridge in front of the first
        # chunk, the TimestepTimeline placing the later ones.
        queued = _after_chunk_modules(
            executor, kwargs, scheduled_start=started[0] if started else None
        )
        queued_chunks.append(queued)
        robot.session.queued_chunk = queued
        base = queued.first_timestamp_ms or 0
        stamps = [base + index * 10 for index in range(queued.caller_step_count)]
        robot.session.scheduled = Scheduled(
            chunk_count=robot.session.queued_chunk_count,
            until_server_ms=ServerMs(stamps[-1]),
            waypoint_timestamps=tuple(stamps),
        )
        robot.session.last_server_timestamp_ms = stamps[-1]
        start_ms = stamps[queued.start_step]
        started.append((queued.action_timestep, ServerMs(start_ms)))
        robot.session.executing_action_timestep = (
            queued.action_timestep + (stamps[-1] - start_ms) // 10
        )
        robot.session.estimated_server_timestamp_ms = (
            100_000  # must not affect controller-timer placement
        )

    policy = _TestPolicy(get_actions, requires_bridge=True)
    robot.session.update_chunk.side_effect = acknowledge_chunk
    executor = PolicyExecutor(
        _schema(),
        policy,
        timeout_s=1.0,
        execution=ContinuousExecution(rate_hz=100),
        pipeline=_unsmoothed_queue_defaults(),
    )

    await executor.run()

    first_send, replacement, *_ = robot.session.update_chunk.call_args_list
    # A timestep queue places chunks on its own timeline, never by the observation.
    assert all(call.kwargs["lead"] is False for call in robot.session.update_chunk.call_args_list)
    assert first_send.kwargs["steps"] == [[0.5] * 6, [1.5] * 6, [2.5] * 6]
    assert queued_chunks[0].steps[:4] == [
        [0.0] * 6,
        [0.5] * 6,
        [1.5] * 6,
        [2.5] * 6,
    ]
    assert first_send.kwargs["first_timestamp_ms"] is None
    assert replacement.kwargs["steps"] == [
        [0.75] * 6,
        [1.75] * 6,
        [2.75] * 6,
    ]
    # Policy action zero is index one in [measured state, action 0, ...], so its exact
    # controller timestamp is 10 ms. Every replacement stays on that immutable policy grid
    # (the session's TimestepTimeline places it); client elapsed time may not move the same
    # absolute action timestep.
    assert replacement.kwargs["first_timestamp_ms"] is None  # the executor hands it over
    assert [q.first_timestamp_ms for q in queued_chunks[1:]] == [30, 50, 70]
    policy.synchronize_action_timestep.assert_any_call(3)


@pytest.mark.asyncio
async def test_a_bridged_chunk_hands_its_io_and_computed_action_to_the_policys_first_step(
    robot: _Robot,
):
    """The session runs the side effects when policy waypoint zero is due, behind the bridge."""
    computed_fired = asyncio.Event()

    async def computed(_chunk: ActionChunk) -> None:
        computed_fired.set()

    schema = PolicySchema(
        observations=[Observation.joint_positions("arm", source=_mg())],
        actions=[Action.computed(computed)],
    )
    chunk = ActionChunk(
        joints={MG_ID: [[3.0] * 6, [4.0] * 6, [5.0] * 6]},
        ios={MG_ID: {"digital_out[0]": True}},
        dt_ms=10.0,
    )
    executor, first_send = await _send_once(robot, chunk, schema=schema)

    robot.session.write_ios.assert_not_awaited()  # not at send
    assert not computed_fired.is_set()
    # The executor hands the IO to the policy's first step; the session's Bridge puts
    # [current, 1, 2] in front (index 3) and its EndpointRamp moves it with its step (to 5).
    step, run = first_send["when_due"]
    assert step == 0
    queued = _after_chunk_modules(executor, first_send)
    assert queued.when_due is not None
    assert queued.when_due[0] == queued.start_step == 5
    await run()
    robot.session.write_ios.assert_awaited_once_with({"digital_out[0]": True})
    assert computed_fired.is_set()


@pytest.mark.asyncio
async def test_fixed_rate_makes_up_slow_ticks_instead_of_lowering_the_rate(robot: _Robot):
    """A tick that overruns its period is caught up by the next one, as ROS Rate.sleep."""
    period_s = 0.04
    calls: list[float] = []
    executor: PolicyExecutor

    async def get_actions(_obs: object) -> ActionChunk:
        calls.append(time.monotonic())
        if len(calls) % 2:
            await asyncio.sleep(1.5 * period_s)  # every other tick overruns by half a period
        if len(calls) == 13:
            executor.stop()
        return ActionChunk(joints={MG_ID: [[0.0] * 6]})

    executor = PolicyExecutor(
        _schema(),
        _TestPolicy(get_actions),
        timeout_s=5.0,
        execution=ContinuousExecution(rate_hz=1 / period_s),
    )
    await executor.run()

    mean_interval_s = (calls[-1] - calls[0]) / (len(calls) - 1)
    # Relative pacing (sleep the rest of each tick) averages 1.25 periods here.
    assert mean_interval_s < 1.12 * period_s


def test_continuous_execution_rejects_invalid_fixed_rates() -> None:
    for rate_hz in (0.0, -1.0, float("inf"), float("nan")):
        with pytest.raises(ValueError, match="positive finite"):
            ContinuousExecution(rate_hz=rate_hz)


@pytest.mark.asyncio
async def test_calling_stop_ends_the_run_with_stopped(robot: _Robot):
    """An external stop() request ends an open-ended run and reports 'stopped'."""
    executor = PolicyExecutor(_schema(), _hold, motion=ActionChunkConfig(), timeout_s=0)

    async def stop_soon() -> None:
        await asyncio.sleep(0.1)
        executor.stop()

    stopper = asyncio.create_task(stop_soon())
    result = await executor.run()
    await stopper

    assert result.reason == "stopped"
    assert result.steps > 0


def test_a_plain_async_function_requires_an_explicit_adapter() -> None:
    with pytest.raises(TypeError, match="CallbackPolicyClient"):
        PolicyExecutor(
            _schema(),
            _hold_action,  # type: ignore[arg-type]
            motion=ActionChunkConfig(),
            timeout_s=0.1,
        )


@pytest.mark.asyncio
async def test_policy_prepare_time_does_not_count_towards_execution_timeout(robot: _Robot):
    """Backend setup can be slow; timeout starts with the first real policy call."""
    policy = _SlowSetupPolicy()
    executor = PolicyExecutor(
        _schema(),
        policy,
        motion=ActionChunkConfig(),
        timeout_s=0.05,
        execution=ContinuousExecution(),
    )
    policy.executor = executor

    result = await executor.run()

    assert policy.prepare_calls == 1
    assert policy.phase_during_prepare == Phase.CONNECTING
    assert result.reason == "timeout"
    assert result.steps > 0
    # Far below the setup time, which would count otherwise; loose enough for a busy CI runner.
    assert result.duration_s < _SLOW_SETUP_S / 2


# ---------------------------------------------------------------------------
# Stop conditions: stop the run *before* the unsafe command reaches the robot
# ---------------------------------------------------------------------------


def joints_out_of_bounds(ctx: StopContext) -> bool:
    return bool(ctx.target_joints) and any(j > 5.0 for step in ctx.target_joints for j in step)


def forbids_output_7(ctx: StopContext) -> bool:
    return bool(ctx.target_ios and ctx.target_ios.get("digital_out[7]"))


@pytest.mark.parametrize(
    ("chunk", "condition"),
    [
        (ActionChunk(joints={MG_ID: [[9.0] * 6]}), joints_out_of_bounds),
        (
            ActionChunk(joints={MG_ID: [[0.1] * 6]}, ios={MG_ID: {"digital_out[7]": True}}),
            forbids_output_7,
        ),
        (ActionChunk(ios={MG_ID: {"digital_out[7]": True}}), forbids_output_7),  # IO-only tick
    ],
    ids=["commanded_joints", "io_write", "io_only_chunk"],
)
@pytest.mark.asyncio
async def test_a_stop_condition_vetoes_a_chunk_before_anything_is_sent(
    robot: _Robot, chunk: ActionChunk, condition: Callable[[StopContext], bool]
):
    """A condition that inspects the commanded joints or the intended IO (also of an IO-only
    queue tick) stops the run before any motion is streamed or any output is set."""

    async def policy(_obs: object) -> ActionChunk:
        return chunk

    executor = PolicyExecutor(
        _schema(), _callback(policy), timeout_s=5.0, stop_conditions=[condition]
    )
    result = await executor.run()

    assert result.reason == f"stop condition: {condition.__name__}"
    robot.session.update_chunk.assert_not_called()  # nothing was streamed to the robot
    robot.session.write_ios.assert_not_awaited()  # IO never written


@pytest.mark.asyncio
async def test_a_stop_condition_fired_mid_run_ends_the_run_and_names_itself(robot: _Robot):
    """A condition evaluated against live robot state (after a chunk was sent)
    ends the run normally, naming itself in ``result.reason``."""

    def fire_after_first_send(*_a: object, **_kw: object) -> None:
        robot.session.stop_condition_triggered = "operator_stop"

    robot.session.update_chunk = MagicMock(side_effect=fire_after_first_send)

    executor = PolicyExecutor(_schema(), _hold, motion=ActionChunkConfig(), timeout_s=5.0)
    result = await executor.run()

    assert result.reason == "stop condition: operator_stop"
    robot.session.update_chunk.assert_called()  # the chunk went out before the stop


# ---------------------------------------------------------------------------
# Faults raise out of run() (the README "Execution lifecycle" table)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("source", "error"),
    [
        ("session", MotionError(MG_ID, "joint_limit")),  # self-collision / joint limit
        ("estop", EmergencyStopError(MG_ID, "protective_stop")),  # seen by the e-stop monitor
        ("session", RuntimeError("jogging connection lost")),
    ],
    ids=["joint_limit", "protective_stop", "lost_connection"],
)
@pytest.mark.asyncio
async def test_a_fault_raises_out_of_run(robot: _Robot, source: str, error: Exception):
    if source == "estop":
        robot.estop.error = error
    else:
        robot.session.has_failed = True
        robot.session.failure_exception = error
    executor = PolicyExecutor(_schema(), _hold, motion=ActionChunkConfig(), timeout_s=5.0)
    with pytest.raises(type(error)) as raised:
        await executor.run()
    assert raised.value is error


# ---------------------------------------------------------------------------
# Mode auto-selection: joint vs cartesian, chosen from the schema
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_send_routes_joint_and_tcp_targets_to_their_own_sessions():
    """A mixed schema drives _send's two branches without cross-wiring.

    One arm is joint-controlled, the other TCP-controlled, in a single schema.
    The executor must open a joint session for the first and a cartesian session
    for the second, then route each arm's slice of the policy output to the
    right session: joint radians to the joint arm, a 6-D pose to the TCP arm —
    never the other way round.
    """
    arm = _mg("0@ur5e-left")
    eef = _mg("0@ur5e-right")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=arm),
            Observation.tcp("eef", source=eef, action=True),
        ]
    )

    async def mixed_policy(_obs: object) -> ActionChunk:
        return ActionChunk(
            joints={"0@ur5e-left": [[float(i) for i in range(1, 7)]]},
            tcp={"0@ur5e-right": [[500.0, 200.0, 300.0, 0.0, 3.14, 0.0]]},
        )

    sessions: dict[str, MagicMock] = {}

    def make_session(*, motion_group: object, mode: str, **_kw: object) -> MagicMock:
        s = _fake_session()
        s.motion_group_id = motion_group.id  # type: ignore[attr-defined]
        s.mode = mode
        sessions[motion_group.id] = s  # type: ignore[attr-defined]
        return s

    with _patched_robot(side_effect=make_session):
        await PolicyExecutor(schema, _callback(mixed_policy), timeout_s=0.05).run()

    # Joint arm: joint mode, six joint radians, routed via the joints branch.
    joint_session = sessions["0@ur5e-left"]
    assert joint_session.mode == "joint"
    assert joint_session.update_chunk.call_args.kwargs["steps"] == [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]]

    # TCP arm: cartesian mode, one 6-D pose, routed via the tcp branch.
    tcp_session = sessions["0@ur5e-right"]
    assert tcp_session.mode == "cartesian"
    assert tcp_session.update_chunk.call_args.kwargs["steps"] == [
        [500.0, 200.0, 300.0, 0.0, 3.14, 0.0]
    ]


@pytest.mark.asyncio
async def test_a_resolved_chunk_is_sent_without_resolving_it_again():
    """A queued client's chunk is already absolute; relative mode must not apply twice."""
    eef = _mg("0@ur5e-right")
    schema = PolicySchema(
        observations=[Observation.tcp("eef", source=eef)],
        actions=[Action.tcp("eef", target=eef, mode="relative")],
    )
    target = [500.0, 200.0, 300.0, 0.0, 3.14, 0.0]

    async def resolved_policy(_obs: object) -> ActionChunk:
        return ActionChunk(tcp={eef.id: [target]}, dt_ms=33.0, resolved=True)

    session = _fake_session()
    session.motion_group_id = eef.id
    session.current_state.pose = SimpleNamespace(
        position=[400.0, 200.0, 300.0], orientation=[0.0, 3.14, 0.0]
    )
    with _patched_robot(return_value=session):
        await PolicyExecutor(schema, _callback(resolved_policy), timeout_s=0.05).run()
    assert session.update_chunk.call_args.kwargs["steps"][-1] == target


@pytest.mark.asyncio
async def test_a_queued_client_runs_continuously_on_the_timestep_timeline(robot: _Robot):
    """QueuedPolicyClient through the executor: the chunk is resolved once, not twice.

    The fake session's clock does not advance, so the run stays at its first chunk;
    merging across replacements is covered by test_queued_client.py.
    """
    eef = _mg(MG_ID)
    schema = PolicySchema(
        observations=[Observation.tcp("eef", source=eef)],
        actions=[Action.tcp("eef", target=eef, mode="relative", accumulate=True)],
    )
    robot.session.current_state.pose = SimpleNamespace(
        position=[400.0, 200.0, 300.0], orientation=[0.0, 3.14, 0.0]
    )

    async def creep(_obs: object) -> ActionChunk:
        await asyncio.sleep(0.01)
        return ActionChunk(tcp={MG_ID: [[0.5, 0.0, 0.0, 0.0, 0.0, 0.0]] * 16}, dt_ms=33.0)

    result = await PolicyExecutor(
        schema,
        QueuedPolicyClient(_callback(creep)),
        timeout_s=0.3,
        execution=ContinuousExecution(),
        n_action_steps=0,
    ).run()

    assert result.reason == "timeout"
    calls = robot.session.update_chunk.call_args_list
    assert calls
    xs = [step[0] for step in calls[0].kwargs["steps"]]
    assert all(399.0 < x < 410.0 for x in xs)  # absolute, resolved once from 400 mm


def test_a_jogger_pipeline_or_a_ramp_under_continuous_execution_is_warned_about(caplog):
    """Both run as asked, but rarely as meant: the joggers' default has no Bridge or JointClamp,
    and an EndpointRamp stops the arm at every replacement."""
    from novapolicy import EndpointRamp, default_pipeline

    with caplog.at_level("WARNING"):
        PolicyExecutor(
            _schema(), _hold, execution=SequentialExecution(), pipeline=default_pipeline()
        )
    assert "joggers' default" in caplog.text
    caplog.clear()
    with caplog.at_level("WARNING"):
        PolicyExecutor(
            _schema(),
            _hold,
            execution=ContinuousExecution(),
            pipeline=[EndpointRamp(), *default_pipeline()],
        )
    assert "EndpointRamp brings every chunk to rest" in caplog.text
    caplog.clear()
    with caplog.at_level("WARNING"):
        PolicyExecutor(
            _schema(), _hold, execution=SequentialExecution()
        )  # the defaults: no warning
        PolicyExecutor(_schema(), _hold, execution=ContinuousExecution())
    assert "joggers' default" not in caplog.text and "EndpointRamp" not in caplog.text


def test_a_timestep_queue_under_sequential_execution_is_rejected():
    """Sequential execution never replaces a running plan, so a queue gets no timing."""
    with pytest.raises(ValueError, match="requires ContinuousExecution"):
        PolicyExecutor(
            _schema(),
            _TestPolicy(_hold_action, requires_bridge=True),
            execution=SequentialExecution(),
        )


@pytest.mark.asyncio
async def test_a_chunk_with_its_own_timestamp_is_not_bridged(robot: _Robot):
    """A bridge would rebuild the chunk without the policy's timestamp; it keeps it instead."""
    chunk = ActionChunk(
        joints={MG_ID: [[3.0] * 6, [3.1] * 6]}, dt_ms=1000 / 30, first_timestamp_ms=5_000
    )
    _, first_send = await _send_once(robot, chunk, execution=SequentialExecution())

    assert first_send["first_timestamp_ms"] == 5_000
    assert first_send["steps"][0] == [3.0] * 6


@pytest.mark.asyncio
async def test_io_of_a_continuous_chunk_goes_with_its_first_step(robot: _Robot):
    """The motion runs one lead after the send; the chunk's IO goes with step 0, not the send."""
    chunk = ActionChunk(
        joints={MG_ID: [[0.0] * 6] * 4}, dt_ms=1000 / 30, ios={MG_ID: {"digital_out[0]": True}}
    )
    _, first_send = await _send_once(robot, chunk, execution=ContinuousExecution())

    robot.session.write_ios.assert_not_awaited()
    step, run = first_send["when_due"]
    assert step == 0
    await run()
    robot.session.write_ios.assert_awaited_once_with({"digital_out[0]": True})


@pytest.mark.asyncio
async def test_a_chunk_without_motion_writes_its_io_before_its_computed_action():
    """Nothing is due later without motion: the IO is written at once, then the computed
    actions run, in that order as on a moving group's step."""
    order: list[str] = []

    async def computed(_chunk: ActionChunk) -> None:
        order.append("computed")

    schema = PolicySchema(
        observations=[Observation.joint_positions("arm", source=_mg())],
        actions=[Action.computed(computed)],
    )
    session = _fake_session()
    session.write_ios = AsyncMock(side_effect=lambda _ios: order.append("io"))
    executor = PolicyExecutor(schema, _hold)
    executor._sessions = {MG_ID: session}
    chunk = ActionChunk(joints={}, ios={MG_ID: {"digital_out[0]": True}})
    executor._send(chunk, chunk)
    await asyncio.gather(*executor._io_tasks)

    session.update_chunk.assert_not_called()
    assert order == ["io", "computed"]


def test_the_pipeline_is_handed_to_every_session():
    """The sessions run exactly the list given, in order; the executor's defaults without one."""
    from novapolicy import EaseIn
    from novapolicy.pipeline.base import Timestamps
    from novapolicy.pipeline.modules.servo_horizon import ServoHorizon

    pipeline = [ServoHorizon(), Timestamps(), EaseIn(duration_s=1.0)]
    with patch("novapolicy.transport.lifecycle.ActionChunkSession") as session_cls:
        executor = PolicyExecutor(
            _schema(), _hold, execution=ContinuousExecution(), pipeline=pipeline
        )
        executor._build_sessions()
        assert session_cls.call_args.kwargs["pipeline"] == tuple(pipeline)
        PolicyExecutor(_schema(), _hold)._build_sessions()
        defaults = default_pipeline(execution=SequentialExecution())
        assert session_cls.call_args.kwargs["pipeline"] == tuple(defaults)


def test_a_misordered_pipeline_is_refused_before_anything_moves():
    from novapolicy import EaseIn
    from novapolicy.pipeline.modules.servo_horizon import ServoHorizon

    with pytest.raises(ValueError, match="before Timestamps"):
        PolicyExecutor(
            _schema(),
            _hold,
            execution=ContinuousExecution(),
            pipeline=[EaseIn(duration_s=1.0), ServoHorizon()],
        )


@pytest.mark.asyncio
async def test_setup_objects_are_frozen_only_while_the_loop_runs(robot: _Robot):
    """Full collections skip what setup allocated while the loop runs; afterwards, also after a
    failed run, nothing stays frozen."""
    before = gc.get_freeze_count()  # the interpreter freezes some objects at startup
    frozen: list[int] = []

    def policy_that(*, fails: bool):
        async def policy(_obs: object) -> ActionChunk:
            frozen.append(gc.get_freeze_count())
            if fails:
                raise RuntimeError("policy crashed")
            executor.stop()
            return ActionChunk(joints={MG_ID: [[0.0] * 6]}, dt_ms=1000 / 30)

        return _callback(policy)

    for fails in (False, True):
        executor = PolicyExecutor(
            _schema(), policy_that(fails=fails), timeout_s=1.0, execution=ContinuousExecution()
        )
        with contextlib.suppress(RuntimeError):
            await executor.run()
        assert frozen.pop() > before
        assert gc.get_freeze_count() == 0


@pytest.mark.asyncio
async def test_a_sequential_chunk_is_dated_from_its_send_and_keeps_its_hold(robot: _Robot):
    """The robot was at rest when observed, so inference time cannot cost a sequential chunk
    any steps: it is due one lead after the send, padded with a hold as before."""
    robot.session.current_state_server_ms = 1_000
    _, first_send = await _send_once(
        robot,
        ActionChunk(joints={MG_ID: [[0.0] * 6]}, dt_ms=1000 / 30),
        delay_s=0.3,  # longer than the 100 ms lead plus the chunk
        timeout_s=2.0,
        execution=SequentialExecution(),
    )

    assert first_send["lead"] is True
    assert first_send["observed_at_ms"] is None  # the send is the reference
    assert first_send["extend_buffer"] is True


# ---------------------------------------------------------------------------
# n_action_steps: policy-declared horizon vs. explicit executor argument
# ---------------------------------------------------------------------------


class _HorizonPolicy(PolicyClient):
    """Policy returning a four-step chunk and declaring its own horizon."""

    def __init__(self, declared: int | None) -> None:
        self._declared = declared

    @property
    def n_action_steps(self) -> int | None:
        return self._declared

    async def get_actions(self, states, schema, images=None, io_values=None) -> ActionChunk:
        return ActionChunk(joints={MG_ID: [[float(step)] * 6 for step in range(4)]}, dt_ms=10.0)


async def _first_sent_steps(policy: PolicyClient, robot: _Robot, **kwargs) -> list:
    """Steps in the first chunk sent, with the endpoint ramp off so the count is the trim."""
    executor = PolicyExecutor(
        _schema(),
        policy,
        timeout_s=0.2,
        execution=SequentialExecution(),
        **kwargs,
    )
    await executor.run()
    return robot.session.update_chunk.call_args_list[0].kwargs["steps"]


@pytest.mark.asyncio
async def test_the_policys_declared_horizon_is_used_when_none_is_given(robot: _Robot):
    """A client that reads its horizon from a checkpoint needs no executor argument."""
    steps = await _first_sent_steps(_HorizonPolicy(2), robot)

    assert len(steps) == 2


@pytest.mark.parametrize(
    ("declared", "kwargs", "sent"),
    [
        # 0 means 'all steps' and overrides a declared horizon: the continuous asynchronous-queue
        # setup passes it deliberately, and deriving over it would start trimming that path.
        (2, {"n_action_steps": 0}, 4),
        (2, {"n_action_steps": 3}, 3),  # an explicit horizon overrides the policy's
        (None, {}, 4),  # a policy declaring none executes every step
    ],
    ids=["explicit_zero", "explicit_horizon", "none_declared"],
)
@pytest.mark.asyncio
async def test_an_explicit_horizon_wins_and_none_executes_every_step(
    robot: _Robot, declared: int | None, kwargs: dict[str, int], sent: int
):
    steps = await _first_sent_steps(_HorizonPolicy(declared), robot, **kwargs)

    assert len(steps) == sent


# ---------------------------------------------------------------------------
# Stale inputs: a declared response instead of an implicit one
# ---------------------------------------------------------------------------


class _StaleCamera:
    """Camera whose frames go stale after a given number of reads."""

    def __init__(self, fresh_reads: int = 0) -> None:
        self._remaining = fresh_reads
        self.reads = 0

    async def connect(self) -> None:
        pass

    def read(self, max_age_s: float = 5.0):
        self.reads += 1
        if self._remaining > 0:
            self._remaining -= 1
            return np.zeros((4, 4, 3), dtype=np.uint8)
        raise RuntimeError(f"frame stale (9.9s > {max_age_s:.1f}s)")

    def get_latest_frame(self, max_age_s: float = 5.0):
        return np.zeros((4, 4, 3), dtype=np.uint8)

    async def disconnect(self) -> None:
        pass


def _camera_schema(camera: object, *, max_age_s: float | None = None) -> PolicySchema:
    return PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=_mg()),
            Observation.image("scene", source=camera, max_age_s=max_age_s),
        ]
    )


class _SleepingPolicy(PolicyClient):
    """Policy whose first inference outlasts the deadline, then returns anyway.

    Mirrors the real hazard: a timed-out gRPC call keeps running on a worker
    thread after the executor has given up on it.
    """

    def __init__(self, delay_s: float) -> None:
        self._delay_s = delay_s
        self.calls = 0
        self.completed = 0

    async def get_actions(self, states, schema, images=None, io_values=None) -> ActionChunk:
        self.calls += 1
        await asyncio.sleep(self._delay_s)
        self.completed += 1
        return ActionChunk(joints={MG_ID: [[0.0] * 6]}, dt_ms=10.0)


@pytest.mark.asyncio
async def test_a_stale_camera_aborts_by_default(robot: _Robot):
    """ABORT is the default and keeps the pre-declaration behaviour: it raises."""
    executor = PolicyExecutor(_camera_schema(_StaleCamera()), _hold, timeout_s=1.0)

    with pytest.raises(RuntimeError, match="Stale policy input"):
        await executor.run()

    assert executor.phase is Phase.ERROR


@pytest.mark.asyncio
async def test_a_per_channel_max_age_overrides_the_executor_default(robot: _Robot):
    """The tolerance is declared per camera; the executor default is the fallback."""
    seen: list[float] = []

    class _RecordingCamera(_StaleCamera):
        def read(self, max_age_s: float = 5.0):
            seen.append(max_age_s)
            return super().read(max_age_s=max_age_s)

    camera = _RecordingCamera(fresh_reads=1)
    executor = PolicyExecutor(
        _camera_schema(camera, max_age_s=0.25),
        _hold,
        timeout_s=1.0,
        camera_max_age_s=9.0,
    )

    with contextlib.suppress(RuntimeError):  # the frame goes stale, or the run times out
        await executor.run()

    assert seen and all(age == 0.25 for age in seen)


@pytest.mark.asyncio
async def test_an_inference_deadline_ends_the_run(robot: _Robot):
    policy = _SleepingPolicy(delay_s=5.0)
    executor = PolicyExecutor(
        _schema(),
        policy,
        timeout_s=2.0,
        inference_timeout_s=0.1,
    )

    with pytest.raises(RuntimeError, match="inference exceeded"):
        await executor.run()


@pytest.mark.asyncio
async def test_no_inference_deadline_when_it_is_disabled(robot: _Robot):
    policy = _SleepingPolicy(delay_s=0.2)
    executor = PolicyExecutor(_schema(), policy, timeout_s=0.5, inference_timeout_s=0)

    result = await executor.run()

    assert result.reason == "timeout"
    assert policy.completed >= 1


@pytest.mark.asyncio
async def test_an_error_drops_the_accepted_waypoints_instead_of_draining(robot: _Robot):
    """`stop` cancels immediately — that is the point on a failure path."""
    robot.estop.error = EmergencyStopError("protective stop")

    with pytest.raises(EmergencyStopError):
        await PolicyExecutor(_schema(), _hold, timeout_s=1.0).run()

    assert robot.session.drain.await_count == 0
    assert robot.session.stop.await_count >= 1


@pytest.mark.asyncio
async def test_a_normal_end_drains_before_stopping(robot: _Robot):
    await PolicyExecutor(_schema(), _hold, timeout_s=0.2).run()

    assert robot.session.drain.await_count >= 1
    assert robot.session.stop.await_count >= 1


def test_a_negative_inference_timeout_is_rejected():
    with pytest.raises(ValueError, match="must not be negative"):
        PolicyExecutor(_schema(), _hold, inference_timeout_s=-1.0)


# ---------------------------------------------------------------------------
# Unit operators on the action path
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_units_are_inverted_before_relative_deltas_resolve(robot: _Robot):
    """A degree-space delta added to a radian state is the bug ops exist to prevent.

    The policy returns +90 in its own (degree) units against a robot at zero.
    Correct: 90deg -> pi/2 rad, added to 0 rad. Wrong: 90 added to 0 rad, then
    converted, or converted after the addition — both land somewhere else.
    """
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=_mg(), mode="relative", ops=[Rad2Deg()])
        ]
    )

    async def policy(_obs: object) -> ActionChunk:
        return ActionChunk(joints={MG_ID: [[90.0] * 6]}, dt_ms=10.0)

    executor = PolicyExecutor(
        schema,
        _callback(policy),
        timeout_s=0.2,
        execution=SequentialExecution(),
    )
    await executor.run()

    sent = robot.session.update_chunk.call_args_list[0].kwargs["steps"][0]
    assert sent == pytest.approx([math.pi / 2] * 6)


@pytest.mark.asyncio
async def test_absolute_targets_reach_the_robot_in_nova_units(robot: _Robot):
    schema = PolicySchema(
        observations=[Observation.joint_positions("arm", source=_mg(), ops=[Rad2Deg()])]
    )

    async def policy(_obs: object) -> ActionChunk:
        return ActionChunk(joints={MG_ID: [[180.0] * 6]}, dt_ms=10.0)

    executor = PolicyExecutor(
        schema,
        _callback(policy),
        timeout_s=0.2,
        execution=SequentialExecution(),
    )
    await executor.run()

    sent = robot.session.update_chunk.call_args_list[0].kwargs["steps"][0]
    assert sent == pytest.approx([math.pi] * 6)


@pytest.mark.asyncio
async def test_the_start_move_keeps_the_cells_collision_setups(robot: _Robot):
    """Regression: the start move used to clear the collision setups and plan without them."""
    mg = MagicMock()
    setup = SimpleNamespace(collision_setups={"cell": object()})
    mg.active_tcp_name = AsyncMock(return_value="Flange")
    mg.get_setup = AsyncMock(return_value=setup)
    mg.plan = AsyncMock(return_value="trajectory")
    mg.execute = AsyncMock()
    collision = setup.collision_setups["cell"]

    executor = PolicyExecutor(_schema(), _hold, timeout_s=0.1, start_joint_position={mg: [0.1] * 6})
    await executor.run()

    mg.execute.assert_awaited_once()
    assert mg.plan.await_args.kwargs["motion_group_setup"] is setup
    assert setup.collision_setups == {"cell": collision}


def test_a_timestep_queue_without_its_timeline_is_refused_before_anything_moves():
    """Without its TimestepTimeline every chunk of a queue would restart at the send."""
    policy = _TestPolicy(_hold_action, requires_bridge=True)
    with pytest.raises(ValueError, match="TimestepTimeline"):
        PolicyExecutor(
            _schema(), policy, execution=ContinuousExecution(), pipeline=default_pipeline()
        )
    PolicyExecutor(_schema(), policy, execution=ContinuousExecution())  # its defaults have it


@pytest.mark.asyncio
async def test_the_bridge_a_session_put_in_front_is_logged_to_rerun():
    """The executor no longer builds the bridge; it logs the one the session's Bridge queued."""
    session = _fake_session()

    def queue_with_bridge(*_args: object, **kwargs: Any) -> None:
        steps = [[0.0] * 6, [0.5] * 6, *kwargs["steps"]]
        session.queued_chunk = PendingChunk(
            steps=steps, dt_ms=kwargs["dt_ms"], caller_step_count=len(steps), start_step=2
        )

    session.update_chunk = MagicMock(side_effect=queue_with_bridge)
    executor = PolicyExecutor(_schema(), _hold, execution=SequentialExecution())
    executor._sessions = {MG_ID: session}
    executor._rerun = MagicMock()
    action = ActionChunk(joints={MG_ID: [[1.0] * 6, [2.0] * 6]}, dt_ms=10.0)
    executor.status.step = 3
    await executor._send_policy_action(action, {})
    bridge, step = executor._rerun.log_bridge_chunk.call_args.args
    assert step == 3
    assert bridge.joints == {MG_ID: [[0.0] * 6, [0.5] * 6, [1.0] * 6]}


@pytest.mark.asyncio
async def test_a_schema_the_policy_refuses_fails_before_the_start_move(robot: _Robot):
    """ADR 0014: connect and validate_schema run before anything moves, so a schema the
    policy cannot serve never sends the robot to its start position."""

    class _RefusingPolicy(CallbackPolicyClient):
        async def validate_schema(self, schema: PolicySchema) -> None:
            raise ValueError("Missing state observations: ['gripper']")

    mg = MagicMock()
    mg.active_tcp_name = AsyncMock(return_value="Flange")
    mg.get_setup = AsyncMock(return_value=SimpleNamespace(collision_setups={}))
    mg.plan = AsyncMock(return_value="trajectory")
    mg.execute = AsyncMock()

    executor = PolicyExecutor(
        _schema(),
        _RefusingPolicy(_hold_action),
        timeout_s=0.1,
        start_joint_position={mg: [0.1] * 6},
    )
    with pytest.raises(ValueError, match="Missing state observations"):
        await executor.run()

    mg.plan.assert_not_awaited()
    mg.execute.assert_not_awaited()
    robot.session.start.assert_not_awaited()


@pytest.mark.parametrize("stop_after_first", [False, True], ids=["mid-run", "last-tick"])
@pytest.mark.asyncio
async def test_a_failed_io_write_without_motion_ends_the_run_with_its_error(
    robot: _Robot, stop_after_first: bool
):
    """Contract T7: an IO write or computed action that raises ends the run with that error,
    also for a chunk without motion, whose effects run at once in a task of their own, and
    also when it was the run's last tick."""
    failure = OSError("digital_out[0]: write refused")
    robot.session.write_ios = AsyncMock(side_effect=failure)
    executor: PolicyExecutor

    async def io_only(_obs: object) -> ActionChunk:
        if stop_after_first:
            executor.stop()
        await asyncio.sleep(0.01)
        return ActionChunk(joints={}, ios={MG_ID: {"digital_out[0]": True}})

    executor = PolicyExecutor(_schema(), _callback(io_only), timeout_s=2.0)
    with pytest.raises(OSError, match="write refused") as raised:
        await executor.run()
    assert raised.value is failure
    assert executor.phase is Phase.ERROR
    assert executor.result is None  # an error, not a normal end
    assert robot.session.drain.await_count == 0  # an error drops, it does not drain


@pytest.mark.asyncio
async def test_a_failed_computed_action_without_motion_ends_the_run_with_its_error(
    robot: _Robot,
):
    failure = RuntimeError("gripper did not answer")

    async def computed(_chunk: ActionChunk) -> None:
        raise failure

    schema = PolicySchema(
        observations=[Observation.joint_positions("arm", source=_mg())],
        actions=[Action.computed(computed)],
    )

    async def no_motion(_obs: object) -> ActionChunk:
        await asyncio.sleep(0.01)
        return ActionChunk(joints={})

    with pytest.raises(RuntimeError, match="gripper did not answer"):
        await PolicyExecutor(schema, _callback(no_motion), timeout_s=2.0).run()


@pytest.mark.asyncio
async def test_a_chunk_with_an_invalid_value_ends_the_run_before_any_arm_gets_it():
    """The session refuses a NaN or a step of the wrong width; the executor checks every arm's
    steps before it queues any, so a bad second arm does not leave the first one moving."""
    left, right = _mg("0@ur5e-left"), _mg("0@ur5e-right")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left", source=left),
            Observation.joint_positions("right", source=right),
        ]
    )

    async def policy(_obs: object) -> ActionChunk:
        return ActionChunk(
            joints={left.id: [[0.0] * 6], right.id: [[0.0, float("nan"), 0.0, 0.0, 0.0, 0.0]]},
            dt_ms=33.0,
        )

    sessions: dict[str, MagicMock] = {}

    def make_session(*, motion_group: object, mode: str, **_kw: object) -> MagicMock:
        s = _fake_session()  # with the session's own check_steps
        s.motion_group_id = motion_group.id  # type: ignore[attr-defined]
        sessions[motion_group.id] = s  # type: ignore[attr-defined]
        return s

    with _patched_robot(side_effect=make_session):
        executor = PolicyExecutor(schema, _callback(policy), timeout_s=2.0)
        with pytest.raises(ValueError, match=r"Step 0 .*'0@ur5e-right'.*not finite"):
            await executor.run()

    assert executor.phase is Phase.ERROR
    for session in sessions.values():
        session.update_chunk.assert_not_called()
