"""Tests for PolicyExecutor control flow — timeout, stop, failure detection."""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from policy.executor import PolicyExecutor
from policy.schema import Observation, PolicySchema
from policy.types import WaypointConfig


def _mg(mg_id: str = "0@ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.split("@")[1] if "@" in mg_id else mg_id
    mg._cell = "cell"
    mg._api_client = MagicMock()
    mg._api_client.close = AsyncMock()
    return mg


def _schema() -> PolicySchema:
    return PolicySchema(
        observations=[
            Observation.joint_positions("arm_joints", source=_mg()),
        ]
    )


def _fake_session() -> MagicMock:
    """Create a mock WaypointJoggingSession."""
    session = MagicMock()
    session.motion_group = MagicMock()
    session.motion_group_id = "0@ur10e"
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
    state.joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    state.pose = None
    state.tcp = None
    state.joint_torques = None
    state.joint_currents = None
    session.current_state = state
    return session


@pytest.mark.asyncio
async def test_timeout_returns_result():
    """Executor stops after timeout_s and returns 'timeout' reason."""
    s = _schema()

    async def policy(obs):
        return {f"arm_joints_{i}": 0.0 for i in range(1, 7)}

    executor = PolicyExecutor(s, policy, motion=WaypointConfig(), timeout_s=0.2)

    with (
        patch("policy.executor.WaypointJoggingSession") as mock_session_cls,
        patch("policy.executor.EstopMonitor") as mock_estop,
    ):
        mock_session_cls.return_value = _fake_session()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        result = await executor.run()

    assert result.reason == "timeout"


@pytest.mark.asyncio
async def test_stop_returns_stopped():
    """Calling stop() signals executor to end with 'stopped' reason."""
    s = _schema()

    async def policy(obs):
        return {f"arm_joints_{i}": 0.0 for i in range(1, 7)}

    executor = PolicyExecutor(s, policy, motion=WaypointConfig(), timeout_s=0)

    async def stop_after_delay():
        await asyncio.sleep(0.1)
        executor.stop()

    with (
        patch("policy.executor.WaypointJoggingSession") as mock_session_cls,
        patch("policy.executor.EstopMonitor") as mock_estop,
    ):
        mock_session_cls.return_value = _fake_session()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        task = asyncio.create_task(stop_after_delay())
        result = await executor.run()
        await task

    assert result.reason == "stopped"
    assert result.steps > 0


@pytest.mark.asyncio
async def test_bare_function_accepted_as_policy():
    """A bare async function (not PolicyClient subclass) is auto-wrapped."""
    s = _schema()

    async def my_policy(obs):
        return {f"arm_joints_{i}": 0.0 for i in range(1, 7)}

    executor = PolicyExecutor(s, my_policy, motion=WaypointConfig(), timeout_s=0.1)

    with (
        patch("policy.executor.WaypointJoggingSession") as mock_session_cls,
        patch("policy.executor.EstopMonitor") as mock_estop,
    ):
        mock_session_cls.return_value = _fake_session()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        result = await executor.run()

    assert result.reason == "timeout"
    assert result.steps > 0


@pytest.mark.asyncio
async def test_last_observation_populated():
    """last_observation is set during execution."""
    s = _schema()

    async def policy(obs):
        return {f"arm_joints_{i}": 0.0 for i in range(1, 7)}

    executor = PolicyExecutor(s, policy, motion=WaypointConfig(), timeout_s=0.1)

    with (
        patch("policy.executor.WaypointJoggingSession") as mock_session_cls,
        patch("policy.executor.EstopMonitor") as mock_estop,
    ):
        mock_session_cls.return_value = _fake_session()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        await executor.run()

    assert executor.last_observation is not None
    assert "0@ur10e" in executor.last_observation


def test_apply_relative_mode():
    """_apply_relative_mode adds current joints to chunk steps."""
    from policy.executor import PolicyExecutor
    from policy.schema import Observation, PolicySchema
    from policy.types import ActionChunk

    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg, mode="relative"),
        ]
    )
    executor = PolicyExecutor(schema, lambda obs: obs, motion=WaypointConfig(), timeout_s=1)

    # Simulate current state
    states = {"0@ur10e": MagicMock(joints=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0))}

    # Chunk with 2 steps of deltas
    chunk = ActionChunk(
        joints={"0@ur10e": [[0.1, 0.1, 0.1, 0.1, 0.1, 0.1], [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]]},
        dt_ms=33.0,
    )
    result = executor._apply_relative_mode(chunk, states)

    # Cumulative: step[0] = current + delta[0], step[1] = current + delta[0] + delta[1]
    assert abs(result.joints["0@ur10e"][0][0] - 1.1) < 1e-9
    assert abs(result.joints["0@ur10e"][1][0] - 1.3) < 1e-9
    assert result.dt_ms == 33.0


def test_apply_relative_mode_absolute_passthrough():
    """_apply_relative_mode is a no-op for absolute mode."""
    from policy.executor import PolicyExecutor
    from policy.schema import Observation, PolicySchema
    from policy.types import ActionChunk

    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg, mode="absolute"),
        ]
    )
    executor = PolicyExecutor(schema, lambda obs: obs, motion=WaypointConfig(), timeout_s=1)

    chunk = ActionChunk(
        joints={"0@ur10e": [[0.5, -1.0, 0.0, 0.0, 0.0, 0.0]]},
        dt_ms=0.0,
    )
    result = executor._apply_relative_mode(chunk, {})
    assert result is chunk  # same object, not copied


@pytest.mark.asyncio
async def test_stop_condition_halts_before_execution():
    """A stop condition sees target_joints/target_ios BEFORE send and ends the run normally."""
    from policy.types import ActionChunk, StopContext

    s = _schema()
    sent_chunks: list[ActionChunk] = []
    io_writes: list[dict] = []

    # Policy returns joints + IO
    async def policy(obs):
        return ActionChunk(
            joints={"0@ur10e": [[9.0, 9.0, 9.0, 9.0, 9.0, 9.0]]},
            ios={"0@ur10e": {"digital_out[0]": True}},
        )

    # Stop if any target joint exceeds 5.0
    def limit_guard(ctx: StopContext) -> bool:
        if ctx.target_joints:
            for step in ctx.target_joints:
                if any(j > 5.0 for j in step):
                    return True
        return False

    executor = PolicyExecutor(
        s, policy, motion=WaypointConfig(), timeout_s=5.0, stop_conditions=[limit_guard]
    )

    with (
        patch("policy.executor.WaypointJoggingSession") as mock_session_cls,
        patch("policy.executor.EstopMonitor") as mock_estop,
    ):
        session = _fake_session()

        # Track calls to verify nothing was sent
        def track_chunk(steps, dt_ms, **kw):
            sent_chunks.append(steps)

        session.update_chunk = MagicMock(side_effect=track_chunk)

        async def track_ios(ios):
            io_writes.append(ios)

        session.write_ios = AsyncMock(side_effect=track_ios)

        mock_session_cls.return_value = session
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        result = await executor.run()

    # Stop condition triggered → normal stop naming the condition
    assert result.reason == "stop condition: limit_guard"
    # Nothing was sent — the condition stopped before execution
    assert sent_chunks == []
    assert io_writes == []


@pytest.mark.asyncio
async def test_stop_condition_sees_target_ios():
    """A stop condition can inspect intended IO writes and stop before they fire."""
    from policy.types import ActionChunk, StopContext

    s = _schema()

    async def policy(obs):
        return ActionChunk(
            joints={"0@ur10e": [[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]]},
            ios={"0@ur10e": {"digital_out[7]": True}},  # forbidden output
        )

    def io_guard(ctx: StopContext) -> bool:
        if ctx.target_ios and ctx.target_ios.get("digital_out[7]"):
            return True  # stop on forbidden output
        return False

    executor = PolicyExecutor(
        s, policy, motion=WaypointConfig(), timeout_s=5.0, stop_conditions=[io_guard]
    )

    with (
        patch("policy.executor.WaypointJoggingSession") as mock_session_cls,
        patch("policy.executor.EstopMonitor") as mock_estop,
    ):
        session = _fake_session()
        session.write_ios = AsyncMock()
        mock_session_cls.return_value = session
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        result = await executor.run()

    assert result.reason == "stop condition: io_guard"
    # IO was never written
    session.write_ios.assert_not_awaited()


@pytest.mark.asyncio
async def test_stop_condition_triggered_on_session_tick_ends_run_normally():
    """A condition that fires on a session tick (after a chunk was sent) ends
    the run normally, naming the condition in result.reason.

    Unlike the pre-send checks above, this covers the path where a stop
    condition is evaluated against live robot state *inside* the jogging
    session and reported back via session.stop_condition_triggered.
    """
    s = _schema()

    async def policy(obs):
        return {f"arm_joints_{i}": 0.0 for i in range(1, 7)}

    executor = PolicyExecutor(s, policy, motion=WaypointConfig(), timeout_s=5.0)

    with (
        patch("policy.executor.WaypointJoggingSession") as mock_session_cls,
        patch("policy.executor.EstopMonitor") as mock_estop,
    ):
        session = _fake_session()
        session.stop_condition_triggered = None

        # The condition fires once the first chunk reaches the session.
        def fire_after_send(*args, **kwargs):
            session.stop_condition_triggered = "operator_stop"

        session.update_chunk = MagicMock(side_effect=fire_after_send)

        mock_session_cls.return_value = session
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        result = await executor.run()

    assert result.reason == "stop condition: operator_stop"
    # The chunk was sent before the condition fired — this is a tick-time stop.
    session.update_chunk.assert_called()


def test_rtc_enabled_with_wait_for_chunk_mode_rejected():
    """RTC + policy_rate_hz<0 silently drops the seam backdate — reject it."""
    s = _schema()
    policy = MagicMock()
    policy.get_actions = AsyncMock()
    policy.rtc = object()  # any non-None value signals RTC is enabled

    with pytest.raises(ValueError, match="RTC"):
        PolicyExecutor(s, policy, motion=WaypointConfig(), policy_rate_hz=-1)


def test_rtc_enabled_with_overlapping_mode_accepted():
    """RTC + policy_rate_hz>=0 is the valid combination."""
    s = _schema()
    policy = MagicMock()
    policy.get_actions = AsyncMock()
    policy.rtc = object()

    # Should not raise.
    PolicyExecutor(s, policy, motion=WaypointConfig(), policy_rate_hz=20)


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
    import contextlib
    import math
    import time

    from nova import Nova
    from policy.types import ActionChunk

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
    import contextlib

    from nova import Nova
    from policy.types import ActionChunk, StopContext

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
