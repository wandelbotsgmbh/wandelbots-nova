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
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from novapolicy.executor import Phase, PolicyExecutor
from novapolicy.policy_client import CallbackPolicyClient, PolicyClient
from novapolicy.schema import Action, Observation, ObservationEntry, PolicySchema
from novapolicy.types import (
    ActionChunk,
    ActionMode,
    EmergencyStopError,
    MotionError,
    StopContext,
    WaypointConfig,
)

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
        states: dict[str, object],
        schema: PolicySchema,
        images: dict[str, object] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> None:
        self.prepare_calls += 1
        self.phase_during_prepare = self.executor.phase if self.executor is not None else None
        await asyncio.sleep(0.25)

    async def get_actions(
        self,
        states: dict[str, object],
        schema: PolicySchema,
        images: dict[str, object] | None = None,
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
    session.queued_chunk_count = 0
    session.scheduled_chunk_count = 0
    session.scheduled_until_server_ms = 0
    session.scheduled_waypoint_timestamps = ()
    session.last_server_timestamp_ms = 0
    session.speed_ratio = 1.0
    session.start = AsyncMock()
    session.stop = AsyncMock()
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


@pytest.mark.asyncio
async def test_sequential_mode_delays_next_inference_until_chunk_deadline_and_pause(robot: _Robot):
    """Sequential mode waits for the submitted chunk's final timestamp and pause."""
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

    executor = PolicyExecutor(
        _schema(),
        _callback(policy),
        timeout_s=1.0,
    )
    run_task = asyncio.create_task(executor.run())

    while robot.session.update_chunk.call_count == 0:
        await asyncio.sleep(0.01)
    await asyncio.sleep(0.05)
    assert inference_count == 1

    robot.session.scheduled_chunk_count = 1
    robot.session.scheduled_until_server_ms = 100
    robot.session.jogging_state = "PAUSED_BY_USER"
    robot.session.last_server_timestamp_ms = 99
    await asyncio.sleep(0.03)
    assert inference_count == 1

    robot.session.last_server_timestamp_ms = 100
    await asyncio.wait_for(second_inference.wait(), timeout=0.5)
    await run_task
    assert inference_count == 2


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
        policy_rate_hz=0,
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
    executor = PolicyExecutor(_schema(), policy, timeout_s=1.0, policy_rate_hz=20)
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
            robot.session.speed_ratio = 1.12
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
    robot.session.speed_ratio = 1.09
    robot.session.update_chunk.side_effect = acknowledge_chunk
    executor = PolicyExecutor(_schema(), policy, timeout_s=1.0, policy_rate_hz=100)

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
    # immutable policy grid; neither client elapsed time nor speed_ratio may
    # move the same absolute action timestep.
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
        interpolate_chunk_ramps=True,
    )
    run_task = asyncio.create_task(executor.run())

    while robot.session.update_chunk.call_count == 0:
        await asyncio.sleep(0.001)
    await asyncio.sleep(0.01)
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
    await run_task


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
        policy_rate_hz=0,
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
async def test_a_tcp_action_schema_opens_a_cartesian_session_and_sends_pose_waypoints():
    """A schema with Observation.tcp(action=True) auto-selects cartesian mode.

    The README/jogging.md claim is that request type is picked from the schema,
    not configured by hand. So a policy returning a flat pose dict must (1) open
    its session in ``mode="cartesian"`` (the PoseWaypointsRequest path) and (2)
    have its six pose values routed through as a single 6-D waypoint — never
    mistaken for joint targets.
    """
    mg = _mg()
    schema = PolicySchema(observations=[Observation.tcp("eef", source=mg, action=True)])

    async def pose_policy(_obs: object) -> ActionChunk:
        return ActionChunk(tcp={mg.id: [[500.0, 200.0, 300.0, 0.0, 3.14, 0.0]]})

    session = _fake_session()
    estop = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)
    with (
        patch("novapolicy.executor.WaypointJoggingSession", return_value=session) as session_cls,
        patch("novapolicy.executor.EstopMonitor", return_value=estop),
    ):
        await PolicyExecutor(schema, _callback(pose_policy), timeout_s=0.05).run()

    # (1) The session was opened in cartesian mode.
    assert session_cls.call_args.kwargs["mode"] == "cartesian"
    # (2) The pose landed as one 6-D TCP waypoint, in [x, y, z, rx, ry, rz] order.
    session.update_chunk.assert_called()
    assert session.update_chunk.call_args.kwargs["steps"] == [[500.0, 200.0, 300.0, 0.0, 3.14, 0.0]]


@pytest.mark.asyncio
async def test_a_joint_schema_opens_a_joint_session(robot: _Robot):
    """The default (no TCP action) opens the session in joint mode."""
    with patch(
        "novapolicy.executor.WaypointJoggingSession", return_value=robot.session
    ) as session_cls:
        await PolicyExecutor(_schema(), _hold, timeout_s=0.05).run()
    assert session_cls.call_args.kwargs["mode"] == "joint"


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
        PolicyExecutor(_schema(), policy, motion=WaypointConfig(), policy_rate_hz=-1)


def test_rtc_with_overlapping_placement_is_accepted():
    """RTC + a non-negative policy_rate_hz is the valid combination."""
    policy = _TestPolicy(_hold_action, rtc=object())
    PolicyExecutor(_schema(), policy, motion=WaypointConfig(), policy_rate_hz=20)  # no raise
