from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
import pytest

from nova_policy.gr00t_client import Gr00tMsgSerializer, Gr00tPolicyClient
from nova_policy.types import ActionChunk

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
                    "state": {"delta_indices": [0], "modality_keys": ["left_arm"]},
                    "action": {"delta_indices": list(range(4)), "modality_keys": ["left_arm"]},
                }
            elif endpoint == "get_action":
                obs = request["data"]["observation"]
                current = obs["state"]["left_arm"][:, -1, :]
                chunk = np.repeat(current[:, np.newaxis, :], 4, axis=1).astype(np.float32)
                response = ({"left_arm": chunk}, {"stateless": True})
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


@pytest.mark.asyncio
async def test_gr00t_client_roundtrip() -> None:
    port = _find_free_port()
    server = _Server(port)
    server.start()
    time.sleep(0.2)

    def decode_action(action: dict[str, Any], info: dict[str, Any]) -> ActionChunk:
        assert info["stateless"] is True
        return ActionChunk(joints={"0@ur10e": action["left_arm"][0].tolist()})

    client = Gr00tPolicyClient(host="127.0.0.1", port=port, decode_action=decode_action)
    await client.connect(["0@ur10e"])

    assert await client.ping() is True

    obs = {"state": {"left_arm": np.zeros((1, 1, 6), dtype=np.float32)}}
    result = await client.get_actions(obs)
    assert isinstance(result, ActionChunk)
    assert "0@ur10e" in result.joints
    assert len(result.joints["0@ur10e"]) == 4

    reset_response = await client.reset()
    assert reset_response["stateless"] is True

    modality = await client.get_modality_config()
    assert modality["action"]["modality_keys"] == ["left_arm"]

    await client.close()
    server.close()
