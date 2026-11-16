"""Tests for CallbackPolicyClient action-chunk handling."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock

import pytest

from novapolicy.policy_client import CallbackPolicyClient
from novapolicy.schema import Observation, PolicySchema
from novapolicy.types import ActionChunk


def _mg(mg_id: str = "0@ur10e", controller_id: str = "ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = controller_id
    mg._cell = "cell"
    return mg


@pytest.mark.asyncio
async def test_returns_action_chunk_unchanged():
    """The callback returns an ActionChunk, which is passed through as-is."""
    mg = _mg()
    schema = PolicySchema(observations=[Observation.joint_positions("joints", source=mg)])
    expected = ActionChunk(joints={"0@ur10e": [[0.5, -1.0], [0.6, -0.9]]}, dt_ms=50.0)

    async def fn(_obs: dict) -> ActionChunk:
        return expected

    client = CallbackPolicyClient(fn)
    chunk = await client.get_actions(states={}, schema=schema)

    assert chunk is expected


@pytest.mark.asyncio
async def test_callback_client_uses_explicit_no_op_capabilities() -> None:
    async def fn(_obs: dict) -> ActionChunk:
        return ActionChunk()

    client = CallbackPolicyClient(fn)
    await client.connect([])
    await client.validate_schema(PolicySchema(observations=[]))
    await client.prepare({}, PolicySchema(observations=[]))
    client.synchronize_action_timestep(4)
    await client.close()

    assert client.requires_first_waypoint_bridge is False
    assert client.rtc is None


@pytest.mark.asyncio
async def test_a_callback_hz_fills_chunks_without_step_spacing() -> None:
    from types import SimpleNamespace

    async def policy(_obs: object) -> ActionChunk:
        return ActionChunk(joints={"mg": [[0.0]]})

    schema = SimpleNamespace(build_observation=AsyncMock(return_value={}))
    chunk = await CallbackPolicyClient(policy, hz=20.0).get_actions({}, schema)  # type: ignore[arg-type]
    assert chunk.dt_ms == pytest.approx(50.0)
