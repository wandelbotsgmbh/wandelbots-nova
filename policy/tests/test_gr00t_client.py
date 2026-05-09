"""Tests for Gr00tPolicyClient ZMQ roundtrip.

Tests verify:
1. The exact numpy observation format sent to the GR00T server
2. Action decoding from GR00T response back to ActionChunk
3. Image handling (single frame and temporal stack)
4. DOF padding/truncation
5. Language instruction handling
6. Multi-arm observations
"""

from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from policy.gr00t import Gr00tMsgSerializer, Gr00tPolicyClient
from policy.schema import Observation, PolicySchema
from policy.types import ActionChunk

zmq = pytest.importorskip("zmq")


def _find_free_port() -> int:
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _mg(mg_id: str = "0@ur10e", controller_id: str = "ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = controller_id
    mg._cell = "cell"
    return mg


def _state(
    joints: tuple[float, ...],
    pose: object | None = None,
) -> MagicMock:
    s = MagicMock()
    s.joints = joints
    s.pose = pose
    s.tcp = None
    s.joint_torques = None
    s.joint_currents = None
    return s


class _RecordingGr00tServer(threading.Thread):
    """GR00T ZMQ server that records observations and returns configurable actions."""

    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self._port = port
        self._shutdown = threading.Event()
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REP)
        self.sock.bind(f"tcp://127.0.0.1:{port}")
        # Recorded from last get_action call
        self.last_observation: dict | None = None
        # Configurable response
        self.action_response: tuple[dict, dict] | None = None

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
                self.last_observation = obs

                if self.action_response is not None:
                    resp = self.action_response
                else:
                    # Default: echo back joints as 4-step action
                    first_state_key = next(iter(obs.get("state", {})))
                    arr = obs["state"][first_state_key]
                    current = arr[:, -1:, :]
                    action = np.repeat(current, 4, axis=1).astype(np.float32)
                    resp = ({first_state_key: action}, {"dt_ms": 33.0})
            elif endpoint == "get_modality_config":
                resp = {"state": {}, "action": {}}
            else:
                resp = {"error": f"Unknown: {endpoint}"}

            self.sock.send(Gr00tMsgSerializer.to_bytes(resp))

    def close(self) -> None:
        self._shutdown.set()
        self.join(2)
        self.sock.close(linger=0)
        self.ctx.term()


# ---------------------------------------------------------------------------
# Observation format tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_single_arm_observation_format() -> None:
    """Verify the exact numpy observation sent for a single arm."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(observations=[
            Observation.joint_positions("left_joints", source=mg),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        joints = (0.1, -1.5, 0.0, 0.5, -0.3, 1.2)
        await client.get_actions({"0@ur10e": _state(joints)}, schema)

        obs = server.last_observation
        assert "state" in obs
        state = obs["state"]

        # Key should match the observation key from schema
        assert "left_joints" in state
        arr = state["left_joints"]
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (1, 1, 6)  # (batch=1, time=1, joints=6)
        np.testing.assert_allclose(arr[0, 0], list(joints), atol=1e-6)

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_dual_arm_observation_format() -> None:
    """Verify both arms appear as separate state keys."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        mg_left = _mg("0@left", "left")
        mg_right = _mg("0@right", "right")
        schema = PolicySchema(observations=[
            Observation.joint_positions("left_arm", source=mg_left),
            Observation.joint_positions("right_arm", source=mg_right),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@left", "0@right"])

        left_joints = (0.1, -1.5, 0.0, 0.5, -0.3, 1.2)
        right_joints = (0.2, -0.8, 0.5, 1.0, -0.5, 0.3)
        await client.get_actions(
            {"0@left": _state(left_joints), "0@right": _state(right_joints)},
            schema,
        )

        state = server.last_observation["state"]
        assert "left_arm" in state
        assert "right_arm" in state
        np.testing.assert_allclose(state["left_arm"][0, 0], list(left_joints), atol=1e-6)
        np.testing.assert_allclose(state["right_arm"][0, 0], list(right_joints), atol=1e-6)

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_io_observation_format() -> None:
    """IO values should appear as state entries with shape (1,1,1)."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        from policy.schema import BoolMapping

        mg = _mg()
        schema = PolicySchema(observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.io("gripper", source=mg, io="digital_out[0]",
                           mapping=BoolMapping(on=100.0)),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        await client.get_actions(
            {"0@ur10e": _state((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))},
            schema,
            io_values={"digital_out[0]": True},
        )

        state = server.last_observation["state"]
        assert "gripper" in state
        arr = state["gripper"]
        assert arr.shape == (1, 1, 1)
        assert arr[0, 0, 0] == pytest.approx(100.0)

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_language_observation() -> None:
    """Language instruction should appear in observation."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.constant("language", value="Pick up the box."),
        ])
        client = Gr00tPolicyClient(
            host="127.0.0.1", port=port,
        )
        await client.connect(["0@ur10e"])

        await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

        obs = server.last_observation
        assert "language" in obs
        lang = obs["language"]
        assert "annotation.language.language_instruction" in lang
        assert lang["annotation.language.language_instruction"] == [["Pick up the box."]]

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_image_observation_single_frame() -> None:
    """Single camera frame should arrive as (1,1,H,W,3) video."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(observations=[
            Observation.joint_positions("arm", source=mg),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        img = np.zeros((224, 224, 3), dtype=np.uint8)
        img[0, 0] = [255, 0, 0]

        await client.get_actions(
            {"0@ur10e": _state((0.0,) * 6)}, schema,
            images={"cam_left": img},
        )

        obs = server.last_observation
        assert "video" in obs
        assert "cam_left" in obs["video"]
        vid = obs["video"]["cam_left"]
        assert vid.shape == (1, 1, 224, 224, 3)
        assert vid[0, 0, 0, 0, 0] == 255

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_image_observation_temporal_stack() -> None:
    """Temporal image stack (T,H,W,3) should arrive as (1,T,H,W,3)."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(observations=[
            Observation.joint_positions("arm", source=mg),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        # Temporal stack: 4 frames of 64x64
        imgs = np.zeros((4, 64, 64, 3), dtype=np.uint8)
        imgs[2, 0, 0] = [42, 0, 0]

        await client.get_actions(
            {"0@ur10e": _state((0.0,) * 6)}, schema,
            images={"wrist": imgs},
        )

        obs = server.last_observation
        vid = obs["video"]["wrist"]
        assert vid.shape == (1, 4, 64, 64, 3)
        assert vid[0, 2, 0, 0, 0] == 42

        await client.close()
    finally:
        server.close()


# ---------------------------------------------------------------------------
# Action decoding tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_action_decoding_single_arm() -> None:
    """Action response with matching key should decode to ActionChunk."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)

    # 4-step action horizon, 6 joints
    action_arr = np.random.randn(1, 4, 6).astype(np.float32)
    server.action_response = ({"arm": action_arr}, {"dt_ms": 50.0})
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(observations=[
            Observation.joint_positions("arm", source=mg),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        result = await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

        assert isinstance(result, ActionChunk)
        assert "0@ur10e" in result.joints
        assert len(result.joints["0@ur10e"]) == 4
        assert len(result.joints["0@ur10e"][0]) == 6
        assert result.dt_ms == 50.0

        # Values should match
        for step_idx in range(4):
            np.testing.assert_allclose(
                result.joints["0@ur10e"][step_idx],
                action_arr[0, step_idx].tolist(),
                atol=1e-6,
            )

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_action_decoding_dual_arm() -> None:
    """Both arms should be decoded from a dual-arm response."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)

    left_action = np.random.randn(1, 4, 6).astype(np.float32)
    right_action = np.random.randn(1, 4, 6).astype(np.float32)
    server.action_response = (
        {"left_arm": left_action, "right_arm": right_action},
        {"dt_ms": 33.0},
    )
    server.start()
    time.sleep(0.1)

    try:
        mg_left = _mg("0@left", "left")
        mg_right = _mg("0@right", "right")
        schema = PolicySchema(observations=[
            Observation.joint_positions("left_arm", source=mg_left),
            Observation.joint_positions("right_arm", source=mg_right),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@left", "0@right"])

        result = await client.get_actions(
            {"0@left": _state((0.0,) * 6), "0@right": _state((0.0,) * 6)},
            schema,
        )

        assert "0@left" in result.joints
        assert "0@right" in result.joints
        assert len(result.joints["0@left"]) == 4
        assert len(result.joints["0@right"]) == 4

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_dof_padding() -> None:
    """When model_dof > actual, joints should be zero-padded."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(observations=[
            Observation.joint_positions("arm", source=mg),
        ])
        # Model expects 7 DOF but UR has 6
        client = Gr00tPolicyClient(host="127.0.0.1", port=port, model_dof=7)
        await client.connect(["0@ur10e"])

        joints = (0.1, -1.5, 0.0, 0.5, -0.3, 1.2)
        await client.get_actions({"0@ur10e": _state(joints)}, schema)

        state = server.last_observation["state"]
        arr = state["arm"]
        assert arr.shape == (1, 1, 7)  # padded to 7
        np.testing.assert_allclose(arr[0, 0, :6], list(joints), atol=1e-6)
        assert arr[0, 0, 6] == 0.0  # pad value

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_dof_truncation_on_action() -> None:
    """When model returns more DOF than robot has, truncate action."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)

    # Model returns 7 joints but robot has 6
    action_arr = np.random.randn(1, 4, 7).astype(np.float32)
    server.action_response = ({"arm": action_arr}, {})
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(observations=[
            Observation.joint_positions("arm", source=mg),
        ])
        client = Gr00tPolicyClient(host="127.0.0.1", port=port, model_dof=7)
        await client.connect(["0@ur10e"])

        result = await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

        # Should be truncated to 6
        assert len(result.joints["0@ur10e"][0]) == 6

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_ping() -> None:
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)
    try:
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect([])
        assert await client.ping() is True
        await client.close()
    finally:
        server.close()
