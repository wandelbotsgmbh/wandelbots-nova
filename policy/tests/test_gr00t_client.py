"""Tests for Gr00tPolicyClient ZMQ roundtrip."""

from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from policy.groot import Gr00tMsgSerializer, Gr00tPolicyClient
from policy.schema import Observation, PolicySchema
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
                obs = req["data"]["observation"]
                current = obs["state"]["left_joints"][:, -1, :]
                action = np.repeat(current[:, np.newaxis, :], 4, axis=1).astype(np.float32)
                resp = ({"left_joints": action}, {})
            elif endpoint == "get_modality_config":
                resp = {
                    "state": {"modality_keys": ["left_joints"]},
                    "action": {"modality_keys": ["left_joints"]},
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

        schema = PolicySchema(observations=[
            Observation.joint_positions("left_joints", source=mg),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        assert await client.ping() is True

        class _State:
            joints = (0.1, -1.5, 0.0, 0.0, 0.0, 0.0)
            pose = None
            tcp = None
            joint_torques = None
            joint_currents = None

        result = await client.get_actions({"0@ur10e": _State()}, schema)

        assert isinstance(result, ActionChunk)
        assert "0@ur10e" in result.joints
        assert len(result.joints["0@ur10e"]) == 4  # 4-step horizon

        await client.close()
    finally:
        server.close()
