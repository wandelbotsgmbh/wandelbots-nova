"""Behavioural tests for PolicyExecutor.

The executor's job is to run a policy against one or more robots and report
*why* it stopped. These tests state that contract through the public surface
(``run()`` -> ``ExecutionResult``), substituting only the robot transport
(``WaypointJoggingSession``) and the e-stop monitor, since there is no real
robot in a unit test. Nothing reaches into the executor's private fields.
"""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
from dataclasses import dataclass
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from novapolicy.executor import PolicyExecutor
from novapolicy.schema import Observation, ObservationEntry, PolicySchema
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


async def _hold(_obs: object) -> ActionChunk:
    """A trivial policy: hold all six joints at zero."""
    return ActionChunk(joints={MG_ID: [[0.0] * 6]})


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
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.wait_ready = AsyncMock()
    session.write_ios = AsyncMock()
    session.update_chunk = MagicMock()
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


@pytest.mark.asyncio
async def test_a_plain_async_function_is_accepted_as_a_policy(robot: _Robot):
    """A bare ``async def obs -> action`` works without a PolicyClient wrapper."""
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=0.1)
    result = await executor.run()
    assert result.reason == "timeout"
    assert result.steps > 0


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
        reach_far,
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
        set_forbidden_output,
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
    its session in ``mode="cartesian"`` (the POSE waypoint path) and (2) have
    its six pose values routed through as a single 6-D waypoint — never mistaken
    for joint targets.
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
        await PolicyExecutor(schema, pose_policy, timeout_s=0.05).run()

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
        await PolicyExecutor(schema, mixed_policy, timeout_s=0.05).run()

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
    policy = MagicMock(get_actions=AsyncMock(), rtc=object())
    with pytest.raises(ValueError, match="RTC"):
        PolicyExecutor(_schema(), policy, motion=WaypointConfig(), policy_rate_hz=-1)


def test_rtc_with_overlapping_placement_is_accepted():
    """RTC + a non-negative policy_rate_hz is the valid combination."""
    policy = MagicMock(get_actions=AsyncMock(), rtc=object())
    PolicyExecutor(_schema(), policy, motion=WaypointConfig(), policy_rate_hz=20)  # no raise
