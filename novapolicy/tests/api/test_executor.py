"""Behavioural tests for PolicyExecutor.

The executor's job is to run a policy against one or more robots and report
*why* it stopped. These tests state that contract through the public surface
(``run()`` -> ``ExecutionResult``), substituting only the robot transport
(``WaypointJoggingSession``) and the e-stop monitor, since there is no real
robot in a unit test. Nothing reaches into the executor's private fields.
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable, Iterator
from dataclasses import dataclass
import math
from typing import TYPE_CHECKING
from unittest.mock import AsyncMock, MagicMock, patch

import numpy as np
import pytest

from novapolicy.executor import Phase, PolicyExecutor
from novapolicy.ops import Rad2Deg
from novapolicy.policy_client import CallbackPolicyClient, PolicyClient
from novapolicy.schema import Action, Observation, ObservationEntry, PolicySchema
from novapolicy.types import (
    ActionChunk,
    ActionMode,
    ContinuousExecution,
    EmergencyStopError,
    EndpointRamp,
    MotionError,
    OnStale,
    SequentialExecution,
    StopContext,
    WaypointConfig,
)

if TYPE_CHECKING:
    from typing import Any

    from numpy.typing import NDArray

    from nova.types import RobotState

MG_ID = "0@ur10e"


# ---------------------------------------------------------------------------
# Test doubles: a single-arm schema and a fake robot transport.
# ---------------------------------------------------------------------------


def _mg(mg_id: str = MG_ID) -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.split("@")[1]
    mg._cell = "cell"
    mg._api_client = MagicMock(close=AsyncMock())
    return mg


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
        rtc: object | None = None,
    ) -> None:
        super().__init__(fn)
        self._requires_bridge = requires_bridge
        self._rtc = rtc
        self.synchronize_action_timestep = MagicMock()

    @property
    def requires_first_waypoint_bridge(self) -> bool:
        return self._requires_bridge

    @property
    def rtc(self) -> object | None:
        return self._rtc


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
        await asyncio.sleep(0.25)

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


def _fake_session() -> MagicMock:
    """A stand-in for a live WaypointJoggingSession (the robot transport)."""
    session = MagicMock()
    session.motion_group = MagicMock()
    session.motion_group_id = MG_ID
    session.has_failed = False
    session.failure_reason = ""
    session.failure_exception = None
    session.stop_condition_triggered = None
    session.session_elapsed_ms = 0
    session.is_running = False
    session.jogging_state = None
    # Explicit, not left to auto-attribute: a bare MagicMock attribute is
    # truthy, which would silently satisfy the settle check's standstill term
    # and stop these tests exercising it at all.
    session.is_at_standstill = False
    session.standstill_ms = 0.0
    session.queued_chunk_count = 0
    session.scheduled_chunk_count = 0
    session.scheduled_until_server_ms = 0
    session.scheduled_waypoint_timestamps = ()
    session.last_server_timestamp_ms = 0

    def scheduled_timestamp_for_step(step: int) -> int | None:
        """Mirror the real session: caller step index -> absolute timestamp.

        Nothing is trimmed in these tests, so the caller's steps and the sent
        waypoints are the same list.
        """
        timestamps = session.scheduled_waypoint_timestamps
        if 0 <= step < len(timestamps):
            return timestamps[step]
        return None

    session.scheduled_timestamp_for_step = MagicMock(side_effect=scheduled_timestamp_for_step)
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.drain = AsyncMock(return_value=True)
    session.wait_ready = AsyncMock()
    session.write_ios = AsyncMock()

    def update_chunk(*_args: object, **_kwargs: object) -> None:
        session.queued_chunk_count += 1

    session.update_chunk = MagicMock(side_effect=update_chunk)
    state = MagicMock()
    state.joints = (0.0,) * 6
    state.pose = state.tcp = None
    session.current_state = state
    return session


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


@pytest.fixture
def robot() -> Iterator[_Robot]:
    """Patch the executor's robot transport and e-stop monitor for the test.

    The patches are active for the whole test, so ``executor.run()`` picks up
    the fakes. A test configures the robot by mutating ``robot.session`` (e.g.
    ``has_failed``) or ``robot.estop.error`` before calling ``run()``.
    """
    session = _fake_session()
    estop = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)
    with (
        patch("novapolicy.executor.WaypointJoggingSession", return_value=session),
        patch("novapolicy.executor.EstopMonitor", return_value=estop),
    ):
        yield _Robot(session=session, estop=estop)


# ---------------------------------------------------------------------------
# Why a run ends: timeout / stop() / plain-function policy
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_run_ends_with_timeout_once_the_deadline_passes(robot: _Robot):
    """With timeout_s set, the executor stops itself and reports 'timeout'."""
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=0.2)
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
    robot.session.scheduled_chunk_count = 1
    robot.session.scheduled_until_server_ms = 100
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
    robot.session.scheduled_chunk_count = 1
    robot.session.scheduled_until_server_ms = 100
    robot.session.last_server_timestamp_ms = 150  # deadline well past
    robot.session.is_at_standstill = False  # ...but still braking
    await asyncio.sleep(0.05)
    assert inference_count() == before

    robot.session.is_at_standstill = True
    await asyncio.wait_for(second_inference.wait(), timeout=0.5)
    _ = await run_task
    assert inference_count() == 2


@pytest.mark.asyncio
async def test_sequential_mode_settles_while_the_state_stream_only_reports_running(
    robot: _Robot,
):
    """A jogger that never leaves RUNNING must not stall sequential execution.

    Regression test for NOVA 26.6. ``PAUSED_BY_USER`` is now reported only in
    response to a Pause/Stop request, which this executor never sends, so a
    drained waypoint queue reports ``RUNNING`` forever. Requiring that state
    made every episode execute exactly one chunk and then hang until the
    timeout.
    """
    run_task, second_inference, inference_count = await _sequential_second_inference(robot)

    robot.session.scheduled_chunk_count = 1
    robot.session.scheduled_until_server_ms = 100
    robot.session.last_server_timestamp_ms = 100
    robot.session.jogging_state = "RUNNING"  # and it never becomes anything else
    robot.session.is_at_standstill = True

    await asyncio.wait_for(second_inference.wait(), timeout=0.5)
    _ = await run_task
    assert inference_count() == 2


@pytest.mark.asyncio
async def test_sequential_mode_still_accepts_a_server_reported_pause(robot: _Robot):
    """``PAUSED_BY_USER`` remains a valid settle signal on its own.

    The standstill measurement replaced this state as the *primary* signal, but
    a server that does report it has genuinely brought the robot to rest, and
    that must keep ending the wait.
    """
    run_task, second_inference, inference_count = await _sequential_second_inference(robot)

    robot.session.scheduled_chunk_count = 1
    robot.session.scheduled_until_server_ms = 100
    robot.session.last_server_timestamp_ms = 100
    robot.session.is_at_standstill = False
    robot.session.jogging_state = "PAUSED_BY_USER"

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
        robot.session.scheduled_chunk_count = robot.session.queued_chunk_count
        count = len(kwargs["steps"])
        robot.session.scheduled_waypoint_timestamps = tuple(range(100, 100 * (count + 1), 100))
        robot.session.scheduled_until_server_ms = robot.session.scheduled_waypoint_timestamps[-1]
        robot.session.last_server_timestamp_ms = robot.session.scheduled_until_server_ms
        robot.session.jogging_state = "PAUSED_BY_USER"

    robot.session.update_chunk.side_effect = acknowledge_chunk
    executor = PolicyExecutor(
        _schema(),
        _callback(policy),
        timeout_s=1.0,
        execution=SequentialExecution(endpoint_ramp=None),
    )

    await executor.run()

    first_send = robot.session.update_chunk.call_args_list[0].kwargs
    assert first_send["steps"] == [
        [0.0] * 6,
        [1.0] * 6,
        [2.0] * 6,
        [3.0] * 6,
        [4.0] * 6,
        [5.0] * 6,
    ]
    assert first_send["dt_ms"] == 10.0


@pytest.mark.asyncio
async def test_continuous_mode_does_not_bridge_chunks(robot: _Robot):
    """Non-negative policy rates replace chunks directly without a settled bridge."""
    executor: PolicyExecutor

    async def policy(_obs: object) -> ActionChunk:
        executor.stop()
        return ActionChunk(
            joints={MG_ID: [[3.0] * 6, [4.0] * 6, [5.0] * 6]},
            dt_ms=10.0,
        )

    executor = PolicyExecutor(
        _schema(),
        _callback(policy),
        timeout_s=1.0,
        execution=ContinuousExecution(),
    )
    await executor.run()

    first_send = robot.session.update_chunk.call_args_list[0].kwargs
    assert first_send["steps"] == [[3.0] * 6, [4.0] * 6, [5.0] * 6]


@pytest.mark.asyncio
async def test_continuous_async_queue_policy_requests_a_measured_state_bridge(robot: _Robot):
    """Async ACT lookahead connects to the robot even though its executor rate is continuous."""
    executor: PolicyExecutor

    async def get_actions(_obs: object) -> ActionChunk:
        executor.stop()
        return ActionChunk(
            joints={MG_ID: [[0.5] * 6, [1.5] * 6, [2.5] * 6]},
            dt_ms=10.0,
        )

    policy = _TestPolicy(get_actions, requires_bridge=True)
    executor = PolicyExecutor(
        _schema(),
        policy,
        timeout_s=1.0,
        execution=ContinuousExecution(rate_hz=20),
    )
    await executor.run()

    first_send = robot.session.update_chunk.call_args_list[0].kwargs
    assert first_send["steps"] == [
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

    def acknowledge_chunk(*_args: object, **kwargs: object) -> None:
        robot.session.queued_chunk_count += 1
        robot.session.scheduled_chunk_count = robot.session.queued_chunk_count
        count = len(kwargs["steps"])
        base = kwargs["first_timestamp_ms"] or 0
        robot.session.scheduled_waypoint_timestamps = tuple(
            base + index * 10 for index in range(count)
        )
        robot.session.scheduled_until_server_ms = robot.session.scheduled_waypoint_timestamps[-1]
        robot.session.last_server_timestamp_ms = robot.session.scheduled_until_server_ms
        robot.session.session_elapsed_ms = 100_000  # must not affect controller-timer placement

    policy = _TestPolicy(get_actions, requires_bridge=True)
    robot.session.update_chunk.side_effect = acknowledge_chunk
    executor = PolicyExecutor(
        _schema(),
        policy,
        timeout_s=1.0,
        execution=ContinuousExecution(rate_hz=100),
    )

    await executor.run()

    first_send, replacement, second_replacement, third_replacement = (
        robot.session.update_chunk.call_args_list
    )
    assert first_send.kwargs["steps"] == [
        [0.0] * 6,
        [0.5] * 6,
        [1.5] * 6,
        [2.5] * 6,
    ]
    assert first_send.kwargs["first_timestamp_ms"] is None
    assert first_send.kwargs["timestamp_offset_steps"] == 0
    assert first_send.kwargs["server_dt_ms"] == 10.0
    assert replacement.kwargs["steps"] == [
        [0.75] * 6,
        [1.75] * 6,
        [2.75] * 6,
    ]
    # Policy action zero is index one in [measured state, action 0, ...], so
    # its exact controller timestamp is 10 ms. Every replacement stays on the
    # immutable policy grid; client elapsed time may not move the same absolute
    # action timestep.
    assert replacement.kwargs["first_timestamp_ms"] == 30
    assert replacement.kwargs["timestamp_offset_steps"] == 0
    assert replacement.kwargs["server_dt_ms"] == 10.0
    assert second_replacement.kwargs["first_timestamp_ms"] == 50
    assert third_replacement.kwargs["first_timestamp_ms"] == 70
    policy.synchronize_action_timestep.assert_any_call(3)


@pytest.mark.asyncio
async def test_connected_chunk_defers_io_and_computed_action_to_policy_boundary(robot: _Robot):
    computed_fired = asyncio.Event()

    async def computed(_chunk: ActionChunk) -> None:
        computed_fired.set()

    schema = PolicySchema(
        observations=[Observation.joint_positions("arm", source=_mg())],
        actions=[Action.computed(computed)],
    )

    async def policy(_obs: object) -> ActionChunk:
        return ActionChunk(
            joints={MG_ID: [[3.0] * 6, [4.0] * 6, [5.0] * 6]},
            ios={MG_ID: {"digital_out[0]": True}},
            dt_ms=10.0,
        )

    def schedule_without_progress(*_args: object, **kwargs: object) -> None:
        robot.session.queued_chunk_count += 1
        robot.session.scheduled_chunk_count = robot.session.queued_chunk_count
        count = len(kwargs["steps"])
        robot.session.scheduled_waypoint_timestamps = tuple(range(100, 100 * (count + 1), 100))
        robot.session.scheduled_until_server_ms = robot.session.scheduled_waypoint_timestamps[-1]
        robot.session.last_server_timestamp_ms = 599
        robot.session.jogging_state = "RUNNING"

    robot.session.update_chunk.side_effect = schedule_without_progress
    executor = PolicyExecutor(
        schema,
        _callback(policy),
        timeout_s=1.0,
    )
    run_task = asyncio.create_task(executor.run())

    await _wait_until(lambda: robot.session.update_chunk.call_count > 0)
    await asyncio.sleep(0)
    robot.session.write_ios.assert_not_awaited()
    assert not computed_fired.is_set()

    # Combined steps are [current, 1, 2, policy[0], policy[1], policy[2]].
    # Acceleration interpolation remaps policy waypoint zero from index 3 to 5,
    # so its scheduled boundary is timestamp 600.
    robot.session.last_server_timestamp_ms = 600
    await asyncio.wait_for(computed_fired.wait(), timeout=0.5)
    robot.session.write_ios.assert_awaited_once_with({"digital_out[0]": True})

    executor.stop()
    robot.session.last_server_timestamp_ms = 1000
    robot.session.jogging_state = "PAUSED_BY_USER"
    _ = await run_task


def test_endpoint_ramp_validates_interpolation_steps() -> None:
    with pytest.raises(ValueError, match="interpolation_steps must be at least 2"):
        EndpointRamp(interpolation_steps=1)


def test_continuous_execution_rejects_invalid_fixed_rates() -> None:
    for rate_hz in (0.0, -1.0, float("inf"), float("nan")):
        with pytest.raises(ValueError, match="positive finite"):
            ContinuousExecution(rate_hz=rate_hz)


@pytest.mark.asyncio
async def test_calling_stop_ends_the_run_with_stopped(robot: _Robot):
    """An external stop() request ends an open-ended run and reports 'stopped'."""
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=0)

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
            motion=WaypointConfig(),
            timeout_s=0.1,
        )


@pytest.mark.asyncio
async def test_policy_prepare_time_does_not_count_towards_execution_timeout(robot: _Robot):
    """Backend setup can be slow; timeout starts with the first real policy call."""
    policy = _SlowSetupPolicy()
    executor = PolicyExecutor(
        _schema(),
        policy,
        motion=WaypointConfig(),
        timeout_s=0.05,
        execution=ContinuousExecution(),
    )
    policy.executor = executor

    result = await executor.run()

    assert policy.prepare_calls == 1
    assert policy.phase_during_prepare == Phase.CONNECTING
    assert result.reason == "timeout"
    assert result.steps > 0
    assert result.duration_s < 0.2


# ---------------------------------------------------------------------------
# Stop conditions: stop the run *before* the unsafe command reaches the robot
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_stop_condition_halts_the_run_before_any_motion_is_sent(robot: _Robot):
    """A condition that vetoes the commanded joints stops without moving."""

    async def reach_far(_obs: object) -> ActionChunk:
        return ActionChunk(joints={MG_ID: [[9.0] * 6]})

    def joints_out_of_bounds(ctx: StopContext) -> bool:
        return bool(ctx.target_joints) and any(j > 5.0 for step in ctx.target_joints for j in step)

    executor = PolicyExecutor(
        _schema(),
        _callback(reach_far),
        motion=WaypointConfig(),
        timeout_s=5.0,
        stop_conditions=[joints_out_of_bounds],
    )
    result = await executor.run()

    assert result.reason == "stop condition: joints_out_of_bounds"
    robot.session.update_chunk.assert_not_called()  # nothing was streamed to the robot


@pytest.mark.asyncio
async def test_a_stop_condition_can_veto_an_io_write_before_it_fires(robot: _Robot):
    """A condition that inspects the intended IO stops before the output is set."""

    async def set_forbidden_output(_obs: object) -> ActionChunk:
        return ActionChunk(joints={MG_ID: [[0.1] * 6]}, ios={MG_ID: {"digital_out[7]": True}})

    def forbids_output_7(ctx: StopContext) -> bool:
        return bool(ctx.target_ios and ctx.target_ios.get("digital_out[7]"))

    executor = PolicyExecutor(
        _schema(),
        _callback(set_forbidden_output),
        motion=WaypointConfig(),
        timeout_s=5.0,
        stop_conditions=[forbids_output_7],
    )
    result = await executor.run()

    assert result.reason == "stop condition: forbids_output_7"
    robot.session.write_ios.assert_not_awaited()  # IO never written


@pytest.mark.asyncio
async def test_a_stop_condition_can_veto_an_io_only_chunk(robot: _Robot):
    """IO-only queue ticks receive the same pre-send guard checks as motion chunks."""

    async def set_forbidden_output(_obs: object) -> ActionChunk:
        return ActionChunk(ios={MG_ID: {"digital_out[7]": True}})

    def forbids_output_7(ctx: StopContext) -> bool:
        return bool(ctx.target_ios and ctx.target_ios.get("digital_out[7]"))

    executor = PolicyExecutor(
        _schema(),
        _callback(set_forbidden_output),
        timeout_s=5.0,
        stop_conditions=[forbids_output_7],
    )
    result = await executor.run()

    assert result.reason == "stop condition: forbids_output_7"
    robot.session.update_chunk.assert_not_called()
    robot.session.write_ios.assert_not_awaited()


@pytest.mark.asyncio
async def test_a_stop_condition_fired_mid_run_ends_the_run_and_names_itself(robot: _Robot):
    """A condition evaluated against live robot state (after a chunk was sent)
    ends the run normally, naming itself in ``result.reason``."""

    def fire_after_first_send(*_a: object, **_kw: object) -> None:
        robot.session.stop_condition_triggered = "operator_stop"

    robot.session.update_chunk = MagicMock(side_effect=fire_after_first_send)

    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=5.0)
    result = await executor.run()

    assert result.reason == "stop condition: operator_stop"
    robot.session.update_chunk.assert_called()  # the chunk went out before the stop


# ---------------------------------------------------------------------------
# Faults raise out of run() (the README "Execution lifecycle" table)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_joint_limit_raises_motion_error(robot: _Robot):
    """A self-collision / joint-limit fault surfaces as MotionError out of run()."""
    robot.session.has_failed = True
    robot.session.failure_exception = MotionError(MG_ID, "joint_limit")
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=5.0)
    with pytest.raises(MotionError):
        await executor.run()


@pytest.mark.asyncio
async def test_a_protective_stop_raises_emergency_stop_error(robot: _Robot):
    """An e-stop / protective stop detected by the monitor raises out of run()."""
    robot.estop.error = EmergencyStopError(MG_ID, "protective_stop")
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=5.0)
    with pytest.raises(EmergencyStopError):
        await executor.run()


@pytest.mark.asyncio
async def test_a_lost_connection_raises_runtime_error(robot: _Robot):
    """A dropped jogging connection surfaces as RuntimeError out of run()."""
    robot.session.has_failed = True
    robot.session.failure_exception = RuntimeError("jogging connection lost")
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=5.0)
    with pytest.raises(RuntimeError, match="connection lost"):
        await executor.run()


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

    estop = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)
    with (
        patch("novapolicy.executor.WaypointJoggingSession", side_effect=make_session),
        patch("novapolicy.executor.EstopMonitor", return_value=estop),
    ):
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


# ---------------------------------------------------------------------------
# RTC requires overlapping placement
# ---------------------------------------------------------------------------


def test_rtc_without_overlapping_placement_is_rejected():
    """RTC + wait-for-chunk would silently drop the seam backdate — reject it."""
    policy = _TestPolicy(_hold_action, rtc=object())
    with pytest.raises(ValueError, match="RTC"):
        PolicyExecutor(
            _schema(),
            policy,
            motion=WaypointConfig(),
            execution=SequentialExecution(),
        )


def test_rtc_with_continuous_execution_is_accepted():
    """RTC and continuous replacement are a valid combination."""
    policy = _TestPolicy(_hold_action, rtc=object())
    PolicyExecutor(
        _schema(),
        policy,
        motion=WaypointConfig(),
        execution=ContinuousExecution(rate_hz=20),
    )  # no raise


def test_continuous_chunks_are_backdated_by_inference_time():
    """A continuous chunk's step zero belongs where the observation was taken."""
    import time

    from novapolicy.types import ActionChunk

    executor = PolicyExecutor(
        _schema(),
        _callback(AsyncMock()),
        execution=ContinuousExecution(),
    )
    chunk = ActionChunk(joints={MG_ID: [[0.0] * 6] * 48}, dt_ms=66.7)

    aligned = executor._align_seam_to_observation(chunk, time.monotonic() - 0.2)
    assert aligned.seam_backdate_steps == 2  # 200 ms of thinking at 66.7 ms steps

    # A chunk whose client already computed its own seam is left alone.
    rtc = chunk.model_copy(update={"seam_backdate_steps": 5})
    assert executor._align_seam_to_observation(rtc, time.monotonic() - 0.2) is rtc

    # Sequential execution never backdates: its chunks start from standstill.
    settled = PolicyExecutor(_schema(), _callback(AsyncMock()), execution=SequentialExecution())
    assert settled._align_seam_to_observation(chunk, time.monotonic() - 0.2) is chunk


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
        execution=SequentialExecution(endpoint_ramp=None),
        **kwargs,
    )
    await executor.run()
    return robot.session.update_chunk.call_args_list[0].kwargs["steps"]


@pytest.mark.asyncio
async def test_the_policys_declared_horizon_is_used_when_none_is_given(robot: _Robot):
    """A client that reads its horizon from a checkpoint needs no executor argument."""
    steps = await _first_sent_steps(_HorizonPolicy(2), robot)

    assert len(steps) == 2


@pytest.mark.asyncio
async def test_an_explicit_zero_keeps_the_full_horizon(robot: _Robot):
    """`n_action_steps=0` means 'all steps' and must override a declared horizon.

    The continuous asynchronous-queue setup passes 0 deliberately; deriving
    over it would silently start trimming that path.
    """
    steps = await _first_sent_steps(_HorizonPolicy(2), robot, n_action_steps=0)

    assert len(steps) == 4


@pytest.mark.asyncio
async def test_an_explicit_horizon_overrides_the_policys(robot: _Robot):
    steps = await _first_sent_steps(_HorizonPolicy(2), robot, n_action_steps=3)

    assert len(steps) == 3


@pytest.mark.asyncio
async def test_a_policy_declaring_no_horizon_executes_every_step(robot: _Robot):
    steps = await _first_sent_steps(_HorizonPolicy(None), robot)

    assert len(steps) == 4


# ---------------------------------------------------------------------------
# Stale inputs: a declared response instead of an implicit one
# ---------------------------------------------------------------------------


class _StaleCamera:
    """Camera whose frames go stale after a given number of reads."""

    def __init__(self, fresh_reads: int = 0) -> None:
        self._remaining = fresh_reads
        self.reads = 0

    async def connect(self) -> None: ...

    def read(self, max_age_s: float = 5.0):
        self.reads += 1
        if self._remaining > 0:
            self._remaining -= 1
            return np.zeros((4, 4, 3), dtype=np.uint8)
        raise RuntimeError(f"frame stale (9.9s > {max_age_s:.1f}s)")

    def get_latest_frame(self, max_age_s: float = 5.0):
        return np.zeros((4, 4, 3), dtype=np.uint8)

    async def disconnect(self) -> None: ...


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
async def test_controlled_stop_ends_the_run_and_names_the_stale_input(robot: _Robot):
    executor = PolicyExecutor(
        _camera_schema(_StaleCamera()),
        _hold,
        timeout_s=1.0,
        on_stale=OnStale.CONTROLLED_STOP,
    )

    result = await executor.run()

    assert result.reason.startswith("stale: camera")
    assert "frame stale" in result.reason
    assert executor.phase is Phase.COMPLETED


@pytest.mark.asyncio
async def test_controlled_stop_runs_out_the_waypoints_already_accepted(robot: _Robot):
    """The accepted lookahead is owed to the caller, so it drains before stopping."""
    executor = PolicyExecutor(
        _camera_schema(_StaleCamera(fresh_reads=1)),
        _hold,
        timeout_s=1.0,
        on_stale=OnStale.CONTROLLED_STOP,
    )

    await executor.run()

    assert robot.session.drain.await_count >= 1


@pytest.mark.asyncio
async def test_hold_retries_then_escalates_when_the_budget_runs_out(robot: _Robot):
    """A camera that never recovers must not hold forever."""
    camera = _StaleCamera()
    executor = PolicyExecutor(
        _camera_schema(camera),
        _hold,
        timeout_s=2.0,
        on_stale=OnStale.HOLD,
        hold_budget_s=0.15,
    )

    result = await executor.run()

    assert result.reason.startswith("stale: camera")
    assert camera.reads > 1, "expected retries before escalating"


@pytest.mark.asyncio
async def test_hold_recovers_when_the_camera_comes_back(robot: _Robot):
    """A transient stale frame must not end the run."""

    class _FlakyCamera(_StaleCamera):
        def read(self, max_age_s: float = 5.0):
            self.reads += 1
            if self.reads == 2:
                raise RuntimeError("frame stale (9.9s > 1.0s)")
            return np.zeros((4, 4, 3), dtype=np.uint8)

    camera = _FlakyCamera()
    executor = PolicyExecutor(
        _camera_schema(camera),
        _hold,
        timeout_s=0.4,
        # Continuous mode ticks without waiting for a standstill, so the loop
        # reaches the recovered frame within the test's deadline.
        execution=ContinuousExecution(),
        on_stale=OnStale.HOLD,
        hold_budget_s=1.0,
    )

    result = await executor.run()

    assert result.reason == "timeout"
    assert camera.reads > 2


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
        on_stale=OnStale.CONTROLLED_STOP,
    )

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
        on_stale=OnStale.CONTROLLED_STOP,
    )

    result = await executor.run()

    assert result.reason.startswith("stale: inference exceeded")


@pytest.mark.asyncio
async def test_hold_is_refused_for_an_inference_deadline(robot: _Robot):
    """The timed-out call is still in flight, so re-entering the client is unsafe."""
    policy = _SleepingPolicy(delay_s=5.0)
    executor = PolicyExecutor(
        _schema(),
        policy,
        timeout_s=2.0,
        inference_timeout_s=0.1,
        on_stale=OnStale.HOLD,
        hold_budget_s=10.0,
    )

    result = await executor.run()

    assert result.reason.startswith("stale: inference exceeded")
    assert policy.calls == 1, "the client must not be re-entered while a call is in flight"


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


@pytest.mark.parametrize(
    ("field", "value"), [("inference_timeout_s", -1.0), ("hold_budget_s", -1.0)]
)
def test_negative_stale_settings_are_rejected(field: str, value: float):
    with pytest.raises(ValueError, match="must not be negative"):
        PolicyExecutor(_schema(), _hold, **{field: value})


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
        execution=SequentialExecution(endpoint_ramp=None),
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
        execution=SequentialExecution(endpoint_ramp=None),
    )
    await executor.run()

    sent = robot.session.update_chunk.call_args_list[0].kwargs["steps"][0]
    assert sent == pytest.approx([math.pi] * 6)
