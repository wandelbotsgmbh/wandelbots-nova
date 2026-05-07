"""Tests for PolicyExecutor control flow — timeout, stop, failure detection."""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from policy.executor import PolicyExecutor
from policy.feature_map import FeatureGroup, FeatureMap
from policy.types import ActionChunk


def _mg(mg_id: str = "0@ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.split("@")[1] if "@" in mg_id else mg_id
    mg._cell = "cell"
    mg._api_client = MagicMock()
    return mg


def _feature_map() -> FeatureMap:
    return FeatureMap(groups=[FeatureGroup(motion_group=_mg(), name="arm")])


def _action() -> ActionChunk:
    return ActionChunk(joints={"0@ur10e": [[0.1, -1.5, 0.0, 0.0, 0.0, 0.0]]}, dt_ms=0.0)


class _FakeRunner:
    """Minimal mock runner that returns state on observe and accepts sends."""

    def __init__(self):
        self._sessions = {}
        self._entered = False

    async def __aenter__(self):
        self._entered = True
        return self

    async def __aexit__(self, *args):
        pass

    async def observe(self):
        state = MagicMock()
        state.joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        state.pose = None
        state.tcp = None
        return {"0@ur10e": state}

    async def send(self, action):
        pass

    async def stop(self):
        pass

    def set_io_values_ref(self, mg_id, ref):
        pass


@pytest.mark.asyncio
async def test_timeout_returns_result():
    """Executor stops after timeout_s and returns 'timeout' reason."""
    fm = _feature_map()

    call_count = 0

    async def policy(obs):
        nonlocal call_count
        call_count += 1
        return {"arm_joint_position_1": 0.1, "arm_joint_position_2": 0.0,
                "arm_joint_position_3": 0.0, "arm_joint_position_4": 0.0,
                "arm_joint_position_5": 0.0, "arm_joint_position_6": 0.0}

    executor = PolicyExecutor(fm, policy, timeout_s=0.2, inference_hz=100)

    with patch("policy.executor.PolicyRunner") as mock_runner, \
         patch("policy.executor.EstopMonitor") as mock_estop:
        mock_runner.return_value = _FakeRunner()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        result = await executor.run()

    assert result.reason == "timeout"


@pytest.mark.asyncio
async def test_stop_returns_stopped():
    """Calling stop() signals executor to end with 'stopped' reason."""
    fm = _feature_map()

    async def policy(obs):
        return {"arm_joint_position_1": 0.0, "arm_joint_position_2": 0.0,
                "arm_joint_position_3": 0.0, "arm_joint_position_4": 0.0,
                "arm_joint_position_5": 0.0, "arm_joint_position_6": 0.0}

    executor = PolicyExecutor(fm, policy, timeout_s=0, inference_hz=100)

    async def stop_after_delay():
        await asyncio.sleep(0.1)
        executor.stop()

    with patch("policy.executor.PolicyRunner") as mock_runner, \
         patch("policy.executor.EstopMonitor") as mock_estop:
        mock_runner.return_value = _FakeRunner()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        task = asyncio.create_task(stop_after_delay())
        result = await executor.run()
        await task

    assert result.reason == "stopped"
    assert result.steps > 0


@pytest.mark.asyncio
async def test_bare_function_accepted_as_policy():
    """A bare async function (not PolicyClient subclass) is auto-wrapped."""
    fm = _feature_map()

    async def my_policy(obs):
        return {"arm_joint_position_1": 0.0, "arm_joint_position_2": 0.0,
                "arm_joint_position_3": 0.0, "arm_joint_position_4": 0.0,
                "arm_joint_position_5": 0.0, "arm_joint_position_6": 0.0}

    executor = PolicyExecutor(fm, my_policy, timeout_s=0.1, inference_hz=50)

    with patch("policy.executor.PolicyRunner") as mock_runner, \
         patch("policy.executor.EstopMonitor") as mock_estop:
        mock_runner.return_value = _FakeRunner()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        result = await executor.run()

    assert result.reason == "timeout"
    assert result.steps > 0


@pytest.mark.asyncio
async def test_last_observation_populated():
    """last_observation is set during execution."""
    fm = _feature_map()

    async def policy(obs):
        return {"arm_joint_position_1": 0.0, "arm_joint_position_2": 0.0,
                "arm_joint_position_3": 0.0, "arm_joint_position_4": 0.0,
                "arm_joint_position_5": 0.0, "arm_joint_position_6": 0.0}

    executor = PolicyExecutor(fm, policy, timeout_s=0.1, inference_hz=50)

    with patch("policy.executor.PolicyRunner") as mock_runner, \
         patch("policy.executor.EstopMonitor") as mock_estop:
        mock_runner.return_value = _FakeRunner()
        mock_estop.return_value = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)

        await executor.run()

    assert executor.last_observation is not None
    assert "0@ur10e" in executor.last_observation
