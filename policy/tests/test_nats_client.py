"""Tests for NatsPolicyClient."""

from __future__ import annotations

from typing import Any
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest

from policy.nats import NatsPolicyClient, pack, unpack
from policy.schema import Observation, PolicySchema
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


def _mock_mg(mg_id: str = "0@ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.split("@")[1] if "@" in mg_id else mg_id
    return mg


def _schema(mg_id: str = "0@ur10e") -> PolicySchema:
    mg = _mock_mg(mg_id)
    return PolicySchema(observations=[Observation.joint_positions("arm_joints", source=mg)])


def _robot_state(joints: list[float] | None = None) -> MagicMock:
    state = MagicMock()
    state.joints = tuple(joints or [0.0] * 6)
    state.pose = None
    state.tcp = None
    state.joint_torques = None
    state.joint_currents = None
    return state


@pytest.mark.asyncio
async def test_returns_action_chunk() -> None:
    nc = _nc()
    nc.request.return_value = _reply({
        "joints": {"0@ur10e": [[0.1, -1.5, 0.0, 0.0, 0.0, 0.0]]},
        "dt_ms": 33.0,
    })

    s = _schema()
    client = NatsPolicyClient(nc, subject="test")
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"0@ur10e": _robot_state()}, s)

    assert isinstance(result, ActionChunk)
    assert result.joints["0@ur10e"] == [[0.1, -1.5, 0.0, 0.0, 0.0, 0.0]]
    assert result.dt_ms == 33.0


@pytest.mark.asyncio
async def test_returns_flat_features() -> None:
    """Flat feature response gets parsed back into ActionChunk via schema."""
    nc = _nc()
    nc.request.return_value = _reply({
        f"arm_joints_{i}": float(i) * 0.1 for i in range(1, 7)
    })

    s = _schema()
    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])
    result = await client.get_actions({"0@ur10e": _robot_state()}, s)

    assert isinstance(result, ActionChunk)
    assert "0@ur10e" in result.joints


@pytest.mark.asyncio
async def test_empty_response_raises() -> None:
    nc = _nc()
    nc.request.return_value = _reply({})

    s = _schema()
    client = NatsPolicyClient(nc)
    await client.connect(["0@ur10e"])

    with pytest.raises(RuntimeError, match="no joints or features"):
        await client.get_actions({"0@ur10e": _robot_state()}, s)


@pytest.mark.asyncio
async def test_images_published_separately() -> None:
    """Images go on sub-subjects, scalars go via request/reply."""
    nc = _nc()
    nc.request.return_value = _reply({
        "joints": {"0@ur10e": [[0.1, -1.5, 0.0, 0.0, 0.0, 0.0]]},
    })

    s = _schema()
    client = NatsPolicyClient(nc, subject="s")
    await client.connect(["0@ur10e"])

    images = {"cam": np.zeros((480, 640, 3), dtype=np.uint8)}
    await client.get_actions({"0@ur10e": _robot_state()}, s, images=images)

    # Image published on separate subject
    nc.publish.assert_awaited_once()
    assert nc.publish.call_args[0][0] == "s.images.cam"

    # Scalars sent via request (without the image)
    scalars = unpack(nc.request.call_args[0][1])
    assert "arm_joints_1" in scalars
    assert "cam" not in scalars
    assert scalars["__images__"] == ["cam"]
