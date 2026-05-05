"""Tests for NatsPolicyClient."""

from __future__ import annotations

import json
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova_policy.nats_client import NatsPolicyClient
from nova_policy.types import ActionChunk, PolicyDone, PolicyWaiting


def _make_nats_client() -> MagicMock:
    """Create a mock nats.aio.client.Client."""
    nc = MagicMock()
    nc.request = AsyncMock()
    nc.publish = AsyncMock()
    return nc


def _reply(data: dict[str, Any]) -> MagicMock:
    """Create a mock NATS message with JSON payload."""
    msg = MagicMock()
    msg.data = json.dumps(data).encode()
    return msg


@pytest.mark.asyncio
async def test_action_chunk_response():
    nc = _make_nats_client()
    nc.request.return_value = _reply({
        "joints": {"0@ur10e": [[0.1, -1.5, 0.0, 0.0, 0.0, 0.0]]},
        "dt_ms": 33.0,
    })

    client = NatsPolicyClient(nc, subject="test.predict")
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"0@ur10e": {"joints": [0, 0, 0, 0, 0, 0]}})

    assert isinstance(result, ActionChunk)
    assert "0@ur10e" in result.joints
    assert result.dt_ms == 33.0
    nc.request.assert_awaited_once()
    # Verify the subject
    call_args = nc.request.call_args
    assert call_args[0][0] == "test.predict"


@pytest.mark.asyncio
async def test_done_response():
    nc = _make_nats_client()
    nc.request.return_value = _reply({"done": True})

    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"0@ur10e": {"joints": [0, 0, 0, 0, 0, 0]}})

    assert isinstance(result, PolicyDone)


@pytest.mark.asyncio
async def test_waiting_response():
    nc = _make_nats_client()
    nc.request.return_value = _reply({"waiting": True})

    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"0@ur10e": {"joints": [0, 0, 0, 0, 0, 0]}})

    assert isinstance(result, PolicyWaiting)


@pytest.mark.asyncio
async def test_flat_features_response():
    nc = _make_nats_client()
    nc.request.return_value = _reply({
        "features": {"left_joint_1.pos": 0.1, "left_joint_2.pos": -1.5},
    })

    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"left_joint_1.pos": 0.0, "left_joint_2.pos": 0.0})

    assert isinstance(result, dict)
    assert result["left_joint_1.pos"] == 0.1


@pytest.mark.asyncio
async def test_notify_stopped():
    nc = _make_nats_client()
    client = NatsPolicyClient(nc, subject="test.predict")
    await client.connect(["0@ur10e"])
    await client.notify_stopped("estop")

    nc.publish.assert_awaited_once()
    call_args = nc.publish.call_args
    assert call_args[0][0] == "test.predict"
    payload = json.loads(call_args[0][1].decode())
    assert payload["executor_stopped"] is True
    assert payload["reason"] == "estop"


@pytest.mark.asyncio
async def test_close_is_noop():
    nc = _make_nats_client()
    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])
    await client.close()
    # Should not close the nats connection — caller owns it


@pytest.mark.asyncio
async def test_structured_observation_serialization():
    """Verify that structured per-MG observations are properly packaged."""
    nc = _make_nats_client()
    nc.request.return_value = _reply({"waiting": True})

    client = NatsPolicyClient(nc, subject="s")
    await client.connect(["0@ur10e", "0@ur10e-2"])

    obs = {
        "0@ur10e": {"joints": [1, 2, 3, 4, 5, 6], "some_extra": "data"},
        "0@ur10e-2": {"joints": [7, 8, 9, 10, 11, 12]},
    }
    await client.get_actions(obs)

    sent_payload = json.loads(nc.request.call_args[0][1].decode())
    assert "0@ur10e" in sent_payload
    assert "0@ur10e-2" in sent_payload
    assert sent_payload["0@ur10e"]["motion_group_id"] == "0@ur10e"


@pytest.mark.asyncio
async def test_flat_observation_passthrough():
    """Flat feature observations are sent as-is."""
    nc = _make_nats_client()
    nc.request.return_value = _reply({"waiting": True})

    client = NatsPolicyClient(nc, subject="s")
    await client.connect(["0@ur10e"])

    flat_obs = {"left_joint_1.pos": 0.1, "left_joint_2.pos": -1.5, "left_gripper.pos": 50.0}
    await client.get_actions(flat_obs)

    sent_payload = json.loads(nc.request.call_args[0][1].decode())
    assert sent_payload == flat_obs


@pytest.mark.asyncio
async def test_action_with_ios():
    nc = _make_nats_client()
    nc.request.return_value = _reply({
        "joints": {"0@ur10e": [[0.1, -1.5, 0.0, 0.0, 0.0, 0.0]]},
        "ios": {"0@ur10e": {"digital_out[0]": True}},
    })

    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"0@ur10e": {"joints": [0, 0, 0, 0, 0, 0]}})

    assert isinstance(result, ActionChunk)
    assert result.ios == {"0@ur10e": {"digital_out[0]": True}}
