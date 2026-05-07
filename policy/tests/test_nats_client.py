"""Tests for NatsPolicyClient."""

from __future__ import annotations

from typing import Any
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest

from policy.nats_client import NatsPolicyClient
from policy.nats_wire import pack, unpack
from policy.types import ActionChunk


def _nc() -> MagicMock:
    nc = MagicMock()
    nc.request = AsyncMock()
    nc.publish = AsyncMock()
    return nc


def _reply(data: dict[str, Any]) -> MagicMock:
    msg = MagicMock()
    msg.data = pack(data)
    return msg


@pytest.mark.asyncio
async def test_returns_action_chunk() -> None:
    nc = _nc()
    nc.request.return_value = _reply({
        "joints": {"0@ur10e": [[0.1, -1.5, 0.0, 0.0, 0.0, 0.0]]},
        "dt_ms": 33.0,
    })

    client = NatsPolicyClient(nc, subject="test")
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"0@ur10e": {"joints": [0] * 6}})

    assert isinstance(result, ActionChunk)
    assert result.joints["0@ur10e"] == [[0.1, -1.5, 0.0, 0.0, 0.0, 0.0]]
    assert result.dt_ms == 33.0


@pytest.mark.asyncio
async def test_returns_flat_features() -> None:
    nc = _nc()
    nc.request.return_value = _reply({"features": {"left_joint_position_1": 0.1}})

    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"left_joint_position_1": 0.0})

    assert isinstance(result, dict)
    assert result["left_joint_position_1"] == 0.1


@pytest.mark.asyncio
async def test_empty_response_raises() -> None:
    nc = _nc()
    nc.request.return_value = _reply({})

    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])

    with pytest.raises(RuntimeError, match="no joints or features"):
        await client.get_actions({"x": 0.0})


@pytest.mark.asyncio
async def test_images_published_separately() -> None:
    """Images go on sub-subjects, scalars go via request/reply."""
    nc = _nc()
    nc.request.return_value = _reply({"features": {"x": 1.0}})

    client = NatsPolicyClient(nc, subject="s")
    await client.connect(["0@ur10e"])

    obs: dict[str, Any] = {
        "x": 0.5,
        "cam": np.zeros((480, 640, 3), dtype=np.uint8),
    }
    await client.get_actions(obs)

    # Image published on separate subject
    nc.publish.assert_awaited_once()
    assert nc.publish.call_args[0][0] == "s.images.cam"

    # Scalars sent via request (without the image)
    scalars = unpack(nc.request.call_args[0][1])
    assert scalars["x"] == 0.5
    assert "cam" not in scalars
    assert scalars["__images__"] == ["cam"]
