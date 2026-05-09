"""Tests for PolicyExecutor control flow — timeout, stop, failure detection."""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from policy.executor import PolicyExecutor
from policy.schema import Observation, PolicySchema


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
    """Create a mock PidJoggingSession."""
    session = MagicMock()
    session.motion_group = MagicMock()
    session.motion_group_id = "0@ur10e"
    session.has_failed = False
    session.failure_reason = ""
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

    executor = PolicyExecutor(s, policy, timeout_s=0.2, inference_hz=100)

    with patch("policy.executor.PidJoggingSession") as mock_session_cls, \
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

    executor = PolicyExecutor(s, policy, timeout_s=0, inference_hz=100)

    async def stop_after_delay():
        await asyncio.sleep(0.1)
        executor.stop()

    with patch("policy.executor.PidJoggingSession") as mock_session_cls, \
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

    executor = PolicyExecutor(s, my_policy, timeout_s=0.1, inference_hz=50)

    with patch("policy.executor.PidJoggingSession") as mock_session_cls, \
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

    executor = PolicyExecutor(s, policy, timeout_s=0.1, inference_hz=50)

    with patch("policy.executor.PidJoggingSession") as mock_session_cls, \
         patch("policy.executor.EstopMonitor") as mock_estop:
        mock_session_cls.return_value = _fake_session()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        await executor.run()

    assert executor.last_observation is not None
    assert "0@ur10e" in executor.last_observation
