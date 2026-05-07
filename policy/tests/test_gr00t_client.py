"""Tests for Gr00tPolicyClient with FeatureMap."""

from __future__ import annotations

import threading
import time
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from policy.feature_map import FeatureGroup, FeatureMap
from policy.gr00t_client import Gr00tMsgSerializer, Gr00tPolicyClient
from policy.types import ActionChunk

zmq = pytest.importorskip("zmq")


class _Server(threading.Thread):
    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self._port = port
        self._stop_event = threading.Event()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://127.0.0.1:{port}")

    def run(self) -> None:
        while not self._stop_event.is_set():
            if self.socket.poll(timeout=100) == 0:
                continue
            request = Gr00tMsgSerializer.from_bytes(self.socket.recv())
            endpoint = request.get("endpoint")
            if endpoint == "ping":
                response = {"status": "ok"}
            elif endpoint == "reset":
                response = {"stateless": True}
            elif endpoint == "get_modality_config":
                response = {
                    "state": {"delta_indices": [0], "modality_keys": ["left_joint_position"]},
                    "action": {"delta_indices": list(range(4)), "modality_keys": ["left_joint_position"]},
                }
            elif endpoint == "get_action":
                obs = request["data"]["observation"]
                current = obs["state"]["left_joint_position"][:, -1, :]
                chunk = np.repeat(current[:, np.newaxis, :], 4, axis=1).astype(np.float32)
                response = ({"left_joint_position": chunk}, {"stateless": True})
            else:
                response = {"error": f"Unknown endpoint: {endpoint}"}
            self.socket.send(Gr00tMsgSerializer.to_bytes(response))

    def close(self) -> None:
        self._stop_event.set()
        self.join(timeout=2)
        self.socket.close(linger=0)
        self.context.term()


def _find_free_port() -> int:
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _mock_motion_group(mg_id: str) -> MagicMock:
    """Create a mock MotionGroup with the necessary attributes."""
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.rsplit("@", maxsplit=1)[-1] if "@" in mg_id else mg_id
    mg._cell = "cell"
    return mg


@pytest.mark.asyncio
async def test_gr00t_client_roundtrip() -> None:
    port = _find_free_port()
    server = _Server(port)
    server.start()
    time.sleep(0.2)

    mg = _mock_motion_group("0@ur10e")
    feature_map = FeatureMap(groups=[FeatureGroup(motion_group=mg, name="left")])

    client = Gr00tPolicyClient(
        host="127.0.0.1",
        port=port,
        feature_map=feature_map,
    )
    await client.connect(["0@ur10e"])

    assert await client.ping() is True

    # Simulate executor observation: {mg_id: RobotState-like object}
    class _FakeState:
        joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    obs: dict[str, Any] = {"0@ur10e": _FakeState()}
    result = await client.get_actions(obs)
    assert isinstance(result, ActionChunk)
    assert "0@ur10e" in result.joints
    assert len(result.joints["0@ur10e"]) == 4  # 4-step horizon from server

    reset_response = await client.reset()
    assert reset_response["stateless"] is True

    modality = await client.get_modality_config()
    assert modality["action"]["modality_keys"] == ["left_joint_position"]

    await client.close()
    server.close()
