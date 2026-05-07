"""Tests for Gr00tPolicyClient ZMQ roundtrip."""

from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from policy.feature_map import FeatureGroup, FeatureMap
from policy.gr00t_client import Gr00tMsgSerializer, Gr00tPolicyClient
from policy.types import ActionChunk

zmq = pytest.importorskip("zmq")


def _find_free_port() -> int:
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


class _MockGr00tServer(threading.Thread):
    """Minimal GR00T ZMQ server that echoes observations back as actions."""

    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self._port = port
        self._shutdown = threading.Event()
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REP)
        self.sock.bind(f"tcp://127.0.0.1:{port}")

    def run(self) -> None:
        while not self._shutdown.is_set():
            if self.sock.poll(100) == 0:
                continue
            req = Gr00tMsgSerializer.from_bytes(self.sock.recv())
            endpoint = req.get("endpoint")

            if endpoint == "ping":
                resp = {"status": "ok"}
            elif endpoint == "get_action":
                # Echo the joint state back as a 4-step action
                obs = req["data"]["observation"]
                current = obs["state"]["left_joint_position"][:, -1, :]
                action = np.repeat(current[:, np.newaxis, :], 4, axis=1).astype(np.float32)
                resp = ({"left_joint_position": action}, {})
            elif endpoint == "get_modality_config":
                resp = {
                    "state": {"modality_keys": ["left_joint_position"]},
                    "action": {"modality_keys": ["left_joint_position"]},
                }
            else:
                resp = {"error": f"Unknown: {endpoint}"}

            self.sock.send(Gr00tMsgSerializer.to_bytes(resp))

    def close(self) -> None:
        self._shutdown.set()
        self.join(2)
        self.sock.close(linger=0)
        self.ctx.term()


@pytest.mark.asyncio
async def test_roundtrip() -> None:
    """Full cycle: connect → get_actions → parse → ActionChunk."""
    port = _find_free_port()
    server = _MockGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        mg = MagicMock()
        mg.id = "0@ur10e"
        mg._controller_id = "ur10e"
        mg._cell = "cell"

        fm = FeatureMap(groups=[FeatureGroup(motion_group=mg, name="left")])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port, feature_map=fm)
        await client.connect(["0@ur10e"])

        assert await client.ping() is True

        # Simulate executor observation
        class _State:
            joints = (0.1, -1.5, 0.0, 0.0, 0.0, 0.0)

        result = await client.get_actions({"0@ur10e": _State()})

        assert isinstance(result, ActionChunk)
        assert "0@ur10e" in result.joints
        assert len(result.joints["0@ur10e"]) == 4  # 4-step horizon

        await client.close()
    finally:
        server.close()
