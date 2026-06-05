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
import contextlib
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from policy.executor import PolicyExecutor
from policy.schema import Observation, PolicySchema
from policy.types import ActionChunk, StopContext, WaypointConfig

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


def _schema(mode: str = "absolute") -> PolicySchema:
    return PolicySchema(observations=[Observation.joint_positions("arm", source=_mg(), mode=mode)])


async def _hold(_obs: object) -> dict[str, float]:
    """A trivial policy: hold all six joints at zero."""
    return {f"arm_{i}": 0.0 for i in range(1, 7)}


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
    state.pose = state.tcp = state.joint_torques = state.joint_currents = None
    session.current_state = state
    return session


@contextlib.contextmanager
def _robot(session: MagicMock) -> Iterator[None]:
    """Patch the executor's robot transport and e-stop monitor with fakes."""
    estop = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)
    with (
        patch("policy.executor.WaypointJoggingSession", return_value=session),
        patch("policy.executor.EstopMonitor", return_value=estop),
    ):
        yield


# ---------------------------------------------------------------------------
# Why a run ends: timeout / stop() / plain-function policy
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_run_ends_with_timeout_once_the_deadline_passes():
    """With timeout_s set, the executor stops itself and reports 'timeout'."""
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=0.2)
    with _robot(_fake_session()):
        result = await executor.run()
    assert result.reason == "timeout"
    assert result.steps > 0


@pytest.mark.asyncio
async def test_calling_stop_ends_the_run_with_stopped():
    """An external stop() request ends an open-ended run and reports 'stopped'."""
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=0)

    async def stop_soon() -> None:
        await asyncio.sleep(0.1)
        executor.stop()

    with _robot(_fake_session()):
        stopper = asyncio.create_task(stop_soon())
        result = await executor.run()
        await stopper

    assert result.reason == "stopped"
    assert result.steps > 0


@pytest.mark.asyncio
async def test_a_plain_async_function_is_accepted_as_a_policy():
    """A bare ``async def obs -> action`` works without a PolicyClient wrapper."""
    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=0.1)
    with _robot(_fake_session()):
        result = await executor.run()
    assert result.reason == "timeout"
    assert result.steps > 0


# ---------------------------------------------------------------------------
# Stop conditions: stop the run *before* the unsafe command reaches the robot
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_a_stop_condition_halts_the_run_before_any_motion_is_sent():
    """A condition that vetoes the commanded joints stops without moving."""

    async def reach_far(_obs: object) -> ActionChunk:
        return ActionChunk(joints={MG_ID: [[9.0] * 6]})

    def joints_out_of_bounds(ctx: StopContext) -> bool:
        return bool(ctx.target_joints) and any(j > 5.0 for step in ctx.target_joints for j in step)

    session = _fake_session()
    executor = PolicyExecutor(
        _schema(),
        reach_far,
        motion=WaypointConfig(),
        timeout_s=5.0,
        stop_conditions=[joints_out_of_bounds],
    )
    with _robot(session):
        result = await executor.run()

    assert result.reason == "stop condition: joints_out_of_bounds"
    session.update_chunk.assert_not_called()  # nothing was streamed to the robot


@pytest.mark.asyncio
async def test_a_stop_condition_can_veto_an_io_write_before_it_fires():
    """A condition that inspects the intended IO stops before the output is set."""

    async def set_forbidden_output(_obs: object) -> ActionChunk:
        return ActionChunk(joints={MG_ID: [[0.1] * 6]}, ios={MG_ID: {"digital_out[7]": True}})

    def forbids_output_7(ctx: StopContext) -> bool:
        return bool(ctx.target_ios and ctx.target_ios.get("digital_out[7]"))

    session = _fake_session()
    executor = PolicyExecutor(
        _schema(),
        set_forbidden_output,
        motion=WaypointConfig(),
        timeout_s=5.0,
        stop_conditions=[forbids_output_7],
    )
    with _robot(session):
        result = await executor.run()

    assert result.reason == "stop condition: forbids_output_7"
    session.write_ios.assert_not_awaited()  # IO never written


@pytest.mark.asyncio
async def test_a_stop_condition_fired_mid_run_ends_the_run_and_names_itself():
    """A condition evaluated against live robot state (after a chunk was sent)
    ends the run normally, naming itself in ``result.reason``."""
    session = _fake_session()

    def fire_after_first_send(*_a: object, **_kw: object) -> None:
        session.stop_condition_triggered = "operator_stop"

    session.update_chunk = MagicMock(side_effect=fire_after_first_send)

    executor = PolicyExecutor(_schema(), _hold, motion=WaypointConfig(), timeout_s=5.0)
    with _robot(session):
        result = await executor.run()

    assert result.reason == "stop condition: operator_stop"
    session.update_chunk.assert_called()  # the chunk went out before the stop


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


# ---------------------------------------------------------------------------
# Integration tests — drive a real PolicyExecutor against a live NOVA instance.
#
# These reuse existing controllers named in NOVA_TEST_CONTROLLERS (default
# "ur5e-left,ur5e-right") when present — e.g. the real arms on a robot cell —
# and otherwise provision virtual UR5e controllers, so the same test runs both
# on a physical-cell instance and on a freshly provisioned CI instance. The
# tests drive *two* robots through the full pipeline end to end: schema ->
# executor -> waypoint jogging session -> NOVA motion API -> state stream. They
# require NOVA_API / NOVA_ACCESS_TOKEN and are skipped unless run with
# `-m integration`.
# ---------------------------------------------------------------------------


def _dual_arm_names() -> list[str]:
    import os

    raw = os.environ.get("NOVA_TEST_CONTROLLERS", "ur5e-left,ur5e-right")
    return [n.strip() for n in raw.split(",") if n.strip()]


async def _ensure_controller(nova_instance, name: str):
    """Reuse a controller by name, or provision a virtual UR5e with that name."""
    from nova import api
    from nova.cell import virtual_controller

    cell = nova_instance.cell()
    return await cell.ensure_controller(
        virtual_controller(
            name=name,
            manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
            type="universalrobots-ur5e",
        )
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_executor_jogs_both_arms_for_the_full_timeout():
    """Run a dual-arm policy for 5 s and confirm both arms jog for ~5 s.

    Proves two things end to end against a live instance: (1) a timeout_s of 5
    really keeps the robots jogging for about 5 s (the executor does not return
    early), and (2) *both* motion groups are driven — both appear in the final
    observed state. Each arm gets a small sinusoid so there is genuine motion.
    """
    import math
    import time

    from nova import Nova

    names = _dual_arm_names()
    assert len(names) == 2, "dual-arm test needs exactly two controller names"

    async with Nova() as nova_instance:
        controllers = [await _ensure_controller(nova_instance, n) for n in names]
        async with contextlib.AsyncExitStack() as stack:
            mgs = [await stack.enter_async_context(c[0]) for c in controllers]
            homes = {mg.id: list(await mg.joints()) for mg in mgs}

            async def dual_wiggle(obs):
                elapsed = obs.get("elapsed_s", 0.0)
                joints = {}
                for mg in mgs:
                    home = homes[mg.id]
                    steps = []
                    for i in range(8):  # 8 steps * 50 ms = 400 ms per chunk
                        t = elapsed + i * 0.05
                        target = list(home)
                        target[0] = home[0] + 0.05 * math.sin(2 * math.pi * 0.3 * t)
                        steps.append(target)
                    joints[mg.id] = steps
                return ActionChunk(joints=joints, dt_ms=50.0)

            schema = PolicySchema(
                observations=[
                    Observation.joint_positions(f"arm{i}", source=mg) for i, mg in enumerate(mgs)
                ]
            )
            executor = PolicyExecutor(schema, dual_wiggle, timeout_s=5.0)

            wall_start = time.monotonic()
            result = await executor.run()
            wall_elapsed = time.monotonic() - wall_start

    # Ended because the timeout elapsed, not for any other reason.
    assert result.reason == "timeout"
    # Jogged for ~5 s: the loop runs the full timeout (small poll/chunk overhang).
    assert 4.8 <= result.duration_s <= 6.5, result.duration_s
    assert wall_elapsed >= 4.8
    # Both arms were actually driven — both show up in the final observed state.
    assert result.last_state is not None
    assert set(result.last_state) == {mg.id for mg in mgs}
    assert result.steps > 0


@pytest.mark.integration
@pytest.mark.asyncio
async def test_stop_condition_ends_dual_arm_run_normally():
    """A stop condition fired during a live dual-arm run ends the run normally.

    The condition is evaluated every loop iteration; after a few ticks it
    returns True and the run ends with the condition's name in result.reason
    (no exception, no failure) while both arms were being jogged.
    """
    from nova import Nova

    names = _dual_arm_names()

    async with Nova() as nova_instance:
        controllers = [await _ensure_controller(nova_instance, n) for n in names]
        async with contextlib.AsyncExitStack() as stack:
            mgs = [await stack.enter_async_context(c[0]) for c in controllers]
            homes = {mg.id: list(await mg.joints()) for mg in mgs}

            async def hold(obs):
                return ActionChunk(joints={mg.id: [homes[mg.id]] for mg in mgs})

            ticks = {"n": 0}

            def stop_after_a_few_ticks(ctx: StopContext) -> bool:
                ticks["n"] += 1
                return ticks["n"] >= 5

            schema = PolicySchema(
                observations=[
                    Observation.joint_positions(f"arm{i}", source=mg) for i, mg in enumerate(mgs)
                ]
            )
            executor = PolicyExecutor(
                schema,
                hold,
                stop_conditions=[stop_after_a_few_ticks],
                timeout_s=10.0,
            )

            result = await executor.run()

    assert result.reason == "stop condition: stop_after_a_few_ticks"
    assert result.last_state is not None
    assert set(result.last_state) == {mg.id for mg in mgs}
