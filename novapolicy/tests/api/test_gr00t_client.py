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

from novapolicy.gr00t import Gr00tMsgSerializer, Gr00tPolicyClient
from novapolicy.schema import Observation, PolicySchema
from novapolicy.types import ActionChunk

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
        # Configurable modality config
        self._modality_config: dict | None = None

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
                resp = self._modality_config or {"state": {}, "action": {}}
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
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("left_joints", source=mg),
            ]
        )
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
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("left_arm", source=mg_left),
                Observation.joint_positions("right_arm", source=mg_right),
            ]
        )
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
        from novapolicy.schema import BoolMapping

        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.io(
                    "gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=100.0)
                ),
            ]
        )
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
async def test_computed_observation_format() -> None:
    """A numeric Observation.computed value should appear as a (1,1,1) state entry."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server.start()
    time.sleep(0.1)

    try:
        called: list[dict] = []

        async def read_force(obs: dict) -> dict:
            called.append(obs)
            return {"force_z": 0.7}

        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.computed(read_force),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

        assert len(called) == 1  # the function was actually triggered
        state = server.last_observation["state"]
        assert "force_z" in state
        arr = state["force_z"]
        assert arr.shape == (1, 1, 1)
        assert arr[0, 0, 0] == pytest.approx(0.7)

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
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.constant("language", value="Pick up the box."),
            ]
        )
        client = Gr00tPolicyClient(
            host="127.0.0.1",
            port=port,
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
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        img = np.zeros((224, 224, 3), dtype=np.uint8)
        img[0, 0] = [255, 0, 0]

        await client.get_actions(
            {"0@ur10e": _state((0.0,) * 6)},
            schema,
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
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        # Temporal stack: 4 frames of 64x64
        imgs = np.zeros((4, 64, 64, 3), dtype=np.uint8)
        imgs[2, 0, 0] = [42, 0, 0]

        await client.get_actions(
            {"0@ur10e": _state((0.0,) * 6)},
            schema,
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
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
            ]
        )
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
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("left_arm", source=mg_left),
                Observation.joint_positions("right_arm", source=mg_right),
            ]
        )
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


@pytest.mark.asyncio
async def test_io_action_decoding() -> None:
    """IO actions from GR00T response should be decoded into ActionChunk.ios."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)

    # GR00T returns joint action + gripper IO as numpy arrays
    action_arr = np.zeros((1, 4, 6), dtype=np.float32)
    gripper_arr = np.array([[[1.0]]], dtype=np.float32)  # (B=1, T=1, D=1)
    server.action_response = (
        {"arm": action_arr, "gripper": gripper_arr},
        {"dt_ms": 33.0},
    )
    server.start()
    time.sleep(0.1)

    try:
        from novapolicy.schema import BoolMapping

        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.io(
                    "gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=1.0)
                ),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        result = await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

        assert isinstance(result, ActionChunk)
        assert result.ios is not None
        assert "0@ur10e" in result.ios
        assert "digital_out[0]" in result.ios["0@ur10e"]
        # BoolMapping(on=1.0) with threshold=0.5: 1.0 >= 0.5 → True
        assert result.ios["0@ur10e"]["digital_out[0]"] is True

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_io_action_bool_mapping_off() -> None:
    """IO value below threshold should map to False."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)

    action_arr = np.zeros((1, 1, 6), dtype=np.float32)
    gripper_arr = np.array([[[0.0]]], dtype=np.float32)
    server.action_response = (
        {"arm": action_arr, "gripper": gripper_arr},
        {"dt_ms": 33.0},
    )
    server.start()
    time.sleep(0.1)

    try:
        from novapolicy.schema import BoolMapping

        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.io(
                    "gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=1.0)
                ),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        result = await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

        assert result.ios is not None
        assert result.ios["0@ur10e"]["digital_out[0]"] is False

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_io_action_not_in_response() -> None:
    """If GR00T doesn't return an IO key, ios should be None."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)

    action_arr = np.zeros((1, 1, 6), dtype=np.float32)
    server.action_response = ({"arm": action_arr}, {"dt_ms": 33.0})
    server.start()
    time.sleep(0.1)

    try:
        from novapolicy.schema import BoolMapping

        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.io(
                    "gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=1.0)
                ),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        result = await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

        # No gripper key in response → no IOs
        assert result.ios is None

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_tcp_action_decoding() -> None:
    """TCP actions from GR00T response should be decoded into ActionChunk.tcp."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)

    # GR00T returns joint action + TCP action as numpy arrays
    joint_arr = np.zeros((1, 4, 6), dtype=np.float32)
    tcp_arr = np.array(
        [[[100.0, 200.0, 300.0, 0.1, 0.2, 0.3], [101.0, 201.0, 301.0, 0.11, 0.21, 0.31]]],
        dtype=np.float32,
    )  # (1, 2, 6)
    server.action_response = (
        {"arm": joint_arr, "eef": tcp_arr},
        {"dt_ms": 33.0},
    )
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.tcp("eef", source=mg, action=True),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        # Need a state with a pose for the observation
        pose = MagicMock()
        pose.position = MagicMock(x=100.0, y=200.0, z=300.0)
        pose.orientation = MagicMock(x=0.1, y=0.2, z=0.3)
        result = await client.get_actions({"0@ur10e": _state((0.0,) * 6, pose=pose)}, schema)

        assert isinstance(result, ActionChunk)
        assert "0@ur10e" in result.tcp
        assert len(result.tcp["0@ur10e"]) == 2  # 2 timesteps
        np.testing.assert_allclose(
            result.tcp["0@ur10e"][0],
            [100.0, 200.0, 300.0, 0.1, 0.2, 0.3],
            atol=1e-5,
        )

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_tcp_action_not_in_response() -> None:
    """If GR00T doesn't return a TCP key, tcp should be empty."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)

    joint_arr = np.zeros((1, 1, 6), dtype=np.float32)
    server.action_response = ({"arm": joint_arr}, {"dt_ms": 33.0})
    server.start()
    time.sleep(0.1)

    try:
        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.tcp("eef", source=mg, action=True),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])

        pose = MagicMock()
        pose.position = MagicMock(x=0.0, y=0.0, z=0.0)
        pose.orientation = MagicMock(x=0.0, y=0.0, z=0.0)
        result = await client.get_actions({"0@ur10e": _state((0.0,) * 6, pose=pose)}, schema)

        # No eef key in response → empty tcp dict
        assert result.tcp == {}

        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_validate_schema_passes_when_keys_match() -> None:
    """validate_schema passes when the schema provides all required keys."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server._modality_config = {
        "state": {
            "__ModalityConfig_class__": True,
            "as_json": {
                "modality_keys": ["arm_joints"],
            },
        },
        "video": {
            "__ModalityConfig_class__": True,
            "as_json": {
                "modality_keys": ["cam_left"],
            },
        },
        "language": {
            "__ModalityConfig_class__": True,
            "as_json": {
                "modality_keys": ["annotation.language.language_instruction"],
            },
        },
    }
    server.start()
    try:
        mg = _mg()
        cam = MagicMock()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm_joints", source=mg),
                Observation.image("cam_left", source=cam),
                Observation.constant("language", value="Pick up the box."),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])
        await client.validate_schema(schema)  # should not raise
        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_validate_schema_fails_on_missing_state_key() -> None:
    """validate_schema raises ValueError when a state key is missing."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server._modality_config = {
        "state": {
            "__ModalityConfig_class__": True,
            "as_json": {
                "modality_keys": ["left_arm", "right_arm"],
            },
        },
    }
    server.start()
    try:
        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("left_arm", source=mg),
                # missing "right_arm"
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])
        with pytest.raises(ValueError, match="right_arm"):
            await client.validate_schema(schema)
        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_validate_schema_fails_on_missing_image_key() -> None:
    """validate_schema raises ValueError when a video key is missing."""
    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server._modality_config = {
        "state": {
            "__ModalityConfig_class__": True,
            "as_json": {
                "modality_keys": ["arm"],
            },
        },
        "video": {
            "__ModalityConfig_class__": True,
            "as_json": {
                "modality_keys": ["cam_left", "cam_right"],
            },
        },
    }
    server.start()
    try:
        mg = _mg()
        cam = MagicMock()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.image("cam_left", source=cam),
                # missing "cam_right"
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port)
        await client.connect(["0@ur10e"])
        with pytest.raises(ValueError, match="cam_right"):
            await client.validate_schema(schema)
        await client.close()
    finally:
        server.close()
