"""Tests for NatsPolicyClient."""

from __future__ import annotations

from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest

from policy.nats_client import NatsPolicyClient
from policy.nats_wire import pack, unpack
from policy.types import ActionChunk


def _make_nats_client() -> MagicMock:
    """Create a mock nats.aio.client.Client."""
    nc = MagicMock()
    nc.request = AsyncMock()
    nc.publish = AsyncMock()
    return nc


def _reply(data: dict[str, Any]) -> MagicMock:
    """Create a mock NATS message with msgpack payload."""
    msg = MagicMock()
    msg.data = pack(data)
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
    call_args = nc.request.call_args
    assert call_args[0][0] == "test.predict"


@pytest.mark.asyncio
async def test_empty_response_raises():
    """Policy must always return joints or features — empty response is an error."""
    nc = _make_nats_client()
    nc.request.return_value = _reply({})

    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])

    with pytest.raises(RuntimeError, match="no joints or features"):
        await client.get_actions({"0@ur10e": {"joints": [0, 0, 0, 0, 0, 0]}})


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
async def test_close_is_noop():
    nc = _make_nats_client()
    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])
    await client.close()


@pytest.mark.asyncio
async def test_structured_observation_serialization():
    """Structured per-MG observations are passed through as-is."""
    nc = _make_nats_client()
    nc.request.return_value = _reply({
        "joints": {"0@ur10e": [[0, 0, 0, 0, 0, 0]], "0@ur10e-2": [[0, 0, 0, 0, 0, 0]]},
    })

    client = NatsPolicyClient(nc, subject="s")
    await client.connect(["0@ur10e", "0@ur10e-2"])

    obs = {
        "0@ur10e": {"joints": [1, 2, 3, 4, 5, 6], "some_extra": "data"},
        "0@ur10e-2": {"joints": [7, 8, 9, 10, 11, 12]},
    }
    await client.get_actions(obs)

    sent_payload = unpack(nc.request.call_args[0][1])
    assert "0@ur10e" in sent_payload
    assert "0@ur10e-2" in sent_payload


@pytest.mark.asyncio
async def test_flat_observation_passthrough():
    """Flat feature observations are sent as-is."""
    nc = _make_nats_client()
    nc.request.return_value = _reply({
        "features": {"left_joint_1.pos": 0.1, "left_joint_2.pos": -1.5, "left_gripper.pos": 50.0},
    })

    client = NatsPolicyClient(nc, subject="s")
    await client.connect(["0@ur10e"])

    flat_obs = {"left_joint_1.pos": 0.1, "left_joint_2.pos": -1.5, "left_gripper.pos": 50.0}
    await client.get_actions(flat_obs)

    sent_payload = unpack(nc.request.call_args[0][1])
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


@pytest.mark.asyncio
async def test_observation_with_images():
    """Images are published on separate NATS subjects, scalars go via request/reply."""
    import numpy as np

    from policy.nats_wire import unpack_image

    nc = _make_nats_client()
    nc.request.return_value = _reply({
        "features": {"left_joint_1.pos": 0.1},
    })

    client = NatsPolicyClient(nc, subject="s")
    await client.connect(["0@ur10e"])

    # Observation with an image
    obs = {
        "left_joint_1.pos": 0.05,
        "flange": np.zeros((480, 640, 3), dtype=np.uint8),
    }
    result = await client.get_actions(obs)

    assert isinstance(result, dict)
    assert result["left_joint_1.pos"] == 0.1

    # Image was published on a separate subject
    nc.publish.assert_awaited_once()
    pub_call = nc.publish.call_args
    assert pub_call[0][0] == "s.images.flange"
    img_decoded = unpack_image(pub_call[0][1])
    assert isinstance(img_decoded, np.ndarray)
    assert img_decoded.shape == (480, 640, 3)

    # Scalars were sent via request (without the image)
    sent_bytes = nc.request.call_args[0][1]
    decoded = unpack(sent_bytes)
    assert decoded["left_joint_1.pos"] == 0.05
    assert "flange" not in decoded
    assert decoded["__images__"] == ["flange"]
