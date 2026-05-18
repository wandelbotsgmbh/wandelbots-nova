"""Tests for PolicyExecutor control flow — timeout, stop, failure detection."""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from policy.executor import PolicyExecutor
from policy.schema import Observation, PolicySchema
from policy.types import MotionConfig


def _mg(mg_id: str = "0@ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.split("@")[1] if "@" in mg_id else mg_id
    mg._cell = "cell"
    mg._api_client = MagicMock()
    mg._api_client.close = AsyncMock()
    return mg


def _schema() -> PolicySchema:
    return PolicySchema(observations=[
        Observation.joint_positions("arm_joints", source=_mg()),
    ])


def _fake_session() -> MagicMock:
    """Create a mock JoggingSession."""
    session = MagicMock()
    session.motion_group = MagicMock()
    session.motion_group_id = "0@ur10e"
    session.has_failed = False
    session.chunk_done = True
    session.failure_reason = ""
    session._failure_exception = None
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.write_ios = AsyncMock()
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

    executor = PolicyExecutor(s, policy, motion=MotionConfig(), timeout_s=0.2, inference_hz=100)

    with patch("policy.executor.JoggingSession") as mock_session_cls, \
         patch("policy.executor.EstopMonitor") as mock_estop:
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

    executor = PolicyExecutor(s, policy, motion=MotionConfig(), timeout_s=0, inference_hz=100)

    async def stop_after_delay():
        await asyncio.sleep(0.1)
        executor.stop()

    with patch("policy.executor.JoggingSession") as mock_session_cls, \
         patch("policy.executor.EstopMonitor") as mock_estop:
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

    executor = PolicyExecutor(s, my_policy, motion=MotionConfig(), timeout_s=0.1, inference_hz=50)

    with patch("policy.executor.JoggingSession") as mock_session_cls, \
         patch("policy.executor.EstopMonitor") as mock_estop:
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

    executor = PolicyExecutor(s, policy, motion=MotionConfig(), timeout_s=0.1, inference_hz=50)

    with patch("policy.executor.JoggingSession") as mock_session_cls, \
         patch("policy.executor.EstopMonitor") as mock_estop:
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
    schema = PolicySchema(observations=[
        Observation.joint_positions("arm", source=mg, mode="relative"),
    ])
    executor = PolicyExecutor(schema, lambda obs: obs, motion=MotionConfig(), timeout_s=1)

    # Simulate current state
    states = {"0@ur10e": MagicMock(joints=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0))}

    # Chunk with 2 steps of deltas
    chunk = ActionChunk(
        joints={"0@ur10e": [[0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                             [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]]},
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
    schema = PolicySchema(observations=[
        Observation.joint_positions("arm", source=mg, mode="absolute"),
    ])
    executor = PolicyExecutor(schema, lambda obs: obs, motion=MotionConfig(), timeout_s=1)

    chunk = ActionChunk(
        joints={"0@ur10e": [[0.5, -1.0, 0.0, 0.0, 0.0, 0.0]]},
        dt_ms=0.0,
    )
    result = executor._apply_relative_mode(chunk, {})
    assert result is chunk  # same object, not copied


@pytest.mark.asyncio
async def test_guard_rejects_action_before_execution():
    """Guard sees target_joints and target_ios BEFORE send. Rejection prevents motion + IO."""
    from policy.types import ActionChunk, GuardState, GuardStopError

    s = _schema()
    sent_chunks: list[ActionChunk] = []
    io_writes: list[dict] = []

    # Policy returns joints + IO
    async def policy(obs):
        return ActionChunk(
            joints={"0@ur10e": [[9.0, 9.0, 9.0, 9.0, 9.0, 9.0]]},
            ios={"0@ur10e": {"digital_out[0]": True}},
        )

    # Guard rejects if any target joint > 5.0
    def limit_guard(ctx: GuardState) -> bool:
        if ctx.target_joints:
            for step in ctx.target_joints:
                if any(j > 5.0 for j in step):
                    return False
        return True

    executor = PolicyExecutor(s, policy, motion=MotionConfig(), timeout_s=5.0, safety_guards=[limit_guard])

    with patch("policy.executor.JoggingSession") as mock_session_cls, \
         patch("policy.executor.EstopMonitor") as mock_estop:
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

        with pytest.raises(GuardStopError) as exc_info:
            await executor.run()

    # Guard triggered
    assert exc_info.value.guard_name == "limit_guard"
    # Nothing was sent — guard blocked before execution
    assert sent_chunks == []
    assert io_writes == []


@pytest.mark.asyncio
async def test_guard_sees_target_ios_before_firing():
    """Guard can inspect intended IO writes and reject before they fire."""
    from policy.types import ActionChunk, GuardState, GuardStopError

    s = _schema()

    async def policy(obs):
        return ActionChunk(
            joints={"0@ur10e": [[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]]},
            ios={"0@ur10e": {"digital_out[7]": True}},  # forbidden output
        )

    def io_guard(ctx: GuardState) -> bool:
        if ctx.target_ios and ctx.target_ios.get("digital_out[7]"):
            return False  # block writes to safety output
        return True

    executor = PolicyExecutor(s, policy, motion=MotionConfig(), timeout_s=5.0, safety_guards=[io_guard])

    with patch("policy.executor.JoggingSession") as mock_session_cls, \
         patch("policy.executor.EstopMonitor") as mock_estop:
        session = _fake_session()
        session.write_ios = AsyncMock()
        mock_session_cls.return_value = session
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        with pytest.raises(GuardStopError) as exc_info:
            await executor.run()

    assert exc_info.value.guard_name == "io_guard"
    # IO was never written
    session.write_ios.assert_not_awaited()
