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

from collections.abc import AsyncIterator, Awaitable, Callable, Iterator
import math
from pathlib import Path
import threading
import time
from typing import ClassVar
from unittest.mock import MagicMock

import numpy as np
import pytest
import pytest_asyncio

from nova.types import Pose
from novapolicy.gr00t import Gr00tMsgSerializer, Gr00tPolicyClient
from novapolicy.schema import Action, BoolMapping, Observation, ObservationEntry, PolicySchema
from novapolicy.tests.doubles import mg as _mg, state as _state
from novapolicy.types import ActionChunk

zmq = pytest.importorskip("zmq")


def _find_free_port() -> int:
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


_WIRE_FORMATS = ("npy", "msgpack-numpy")
"""How GR00T policy servers pack arrays in their answers.

``npy`` is N1.6 to the N1.7 release (``{__ndarray_class__, as_npy}``);
``msgpack-numpy`` is Isaac-GR00T ``main`` since 2026-05 (``{nd, type, shape, data}``).
Both decode requests in the npy format this client sends.
"""


def _encode_response(response: object, wire_format: str) -> bytes:
    if wire_format == "npy":
        return Gr00tMsgSerializer.to_bytes(response)
    import msgpack

    def default(obj: object) -> object:
        if isinstance(obj, (np.ndarray, np.generic)):
            return _msgpack_numpy(obj)
        return obj

    return msgpack.packb(response, default=default)


@pytest.fixture(autouse=True, params=_WIRE_FORMATS)
def server_wire_format(request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch) -> str:
    """Run every test against both GR00T server generations.

    Every test in this module talks to a fake server; the serializer's own
    tests live in ``tests/unit/test_gr00t_serializer.py`` and run once.
    """
    monkeypatch.setattr(_RecordingGr00tServer, "wire_format", request.param)
    return request.param


class _RecordingGr00tServer(threading.Thread):
    """GR00T ZMQ server that records observations and returns configurable actions."""

    wire_format: ClassVar[str] = "npy"

    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self.port = port
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
                    # Default: echo every state back as a 4-step action, as a real
                    # server answers with every action key it was trained on.
                    action = {
                        key: np.repeat(arr[:, -1:, :], 4, axis=1).astype(np.float32)
                        for key, arr in obs.get("state", {}).items()
                    }
                    resp = (action, {"dt_ms": 33.0})
            elif endpoint == "get_modality_config":
                resp = self._modality_config or {"state": {}, "action": {}}
            else:
                resp = {"error": f"Unknown: {endpoint}"}

            self.sock.send(_encode_response(resp, self.wire_format))

    def close(self) -> None:
        self._shutdown.set()
        self.join(2)
        self.sock.close(linger=0)
        self.ctx.term()


@pytest.fixture
def server() -> Iterator[_RecordingGr00tServer]:
    """A started fake GR00T server, closed after the test even when it fails."""
    server = _RecordingGr00tServer(_find_free_port())
    server.start()
    yield server
    server.close()


@pytest_asyncio.fixture
async def client(server: _RecordingGr00tServer) -> AsyncIterator[Gr00tPolicyClient]:
    """A 30 Hz client connected to ``server``, closed after the test."""
    client = Gr00tPolicyClient(host="127.0.0.1", port=server.port, control_freq=30.0)
    await client.connect([])
    yield client
    await client.close()


def _modality(keys: list[str]) -> dict:
    return {"__ModalityConfig_class__": True, "as_json": {"modality_keys": keys}}


@pytest.fixture
def actions_from(
    server: _RecordingGr00tServer, client: Gr00tPolicyClient
) -> Callable[..., Awaitable[ActionChunk]]:
    """Decode one scripted answer for an ``arm`` joint observation plus ``extra`` entries."""

    async def decode(
        response: tuple[dict, dict], *extra: ObservationEntry, pose: Pose | None = None
    ) -> ActionChunk:
        server.action_response = response
        schema = PolicySchema(
            observations=[Observation.joint_positions("arm", source=_mg()), *extra]
        )
        return await client.get_actions({"0@ur10e": _state((0.0,) * 6, pose=pose)}, schema)

    return decode


def _gripper() -> ObservationEntry:
    return Observation.io("gripper", source=_mg(), io="digital_out[0]", mapping=BoolMapping(on=1.0))


# ---------------------------------------------------------------------------
# Observation format tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_single_arm_observation_format(server, client) -> None:
    """Verify the exact numpy observation sent for a single arm."""
    schema = PolicySchema(observations=[Observation.joint_positions("left_joints", source=_mg())])
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


@pytest.mark.asyncio
async def test_dual_arm_observation_format(server, client) -> None:
    """Verify both arms appear as separate state keys."""
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left_arm", source=_mg("0@left")),
            Observation.joint_positions("right_arm", source=_mg("0@right")),
        ]
    )
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


@pytest.mark.asyncio
async def test_io_observation_format(server, client) -> None:
    """IO values should appear as state entries with shape (1,1,1)."""
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.io(
                "gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=100.0)
            ),
        ]
    )
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


@pytest.mark.asyncio
async def test_computed_observation_format(server, client) -> None:
    """A numeric Observation.computed value should appear as a (1,1,1) state entry."""
    called: list[dict] = []

    async def read_force(obs: dict) -> dict:
        called.append(obs)
        return {"force_z": 0.7}

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=_mg()),
            Observation.computed(read_force),
        ]
    )
    await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

    assert len(called) == 1  # the function was actually triggered
    assert called[0]["arm_1"] == 0.0  # the flat observation, as for every client
    state = server.last_observation["state"]
    assert "force_z" in state
    arr = state["force_z"]
    assert arr.shape == (1, 1, 1)
    assert arr[0, 0, 0] == pytest.approx(0.7)


@pytest.mark.asyncio
async def test_language_observation(server, client) -> None:
    """Language instruction should appear in observation."""
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=_mg()),
            Observation.constant("language", value="Pick up the box."),
        ]
    )
    await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema)

    obs = server.last_observation
    assert "language" in obs
    lang = obs["language"]
    assert "annotation.language.language_instruction" in lang
    assert lang["annotation.language.language_instruction"] == [["Pick up the box."]]


@pytest.mark.asyncio
async def test_image_observation_single_frame(server, client) -> None:
    """Single camera frame should arrive as (1,1,H,W,3) video."""
    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=_mg())])
    img = np.zeros((224, 224, 3), dtype=np.uint8)
    img[0, 0] = [255, 0, 0]

    await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema, images={"cam_left": img})

    obs = server.last_observation
    assert "video" in obs
    assert "cam_left" in obs["video"]
    vid = obs["video"]["cam_left"]
    assert vid.shape == (1, 1, 224, 224, 3)
    assert vid[0, 0, 0, 0, 0] == 255


@pytest.mark.asyncio
async def test_image_observation_temporal_stack(server, client) -> None:
    """Temporal image stack (T,H,W,3) should arrive as (1,T,H,W,3)."""
    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=_mg())])
    # Temporal stack: 4 frames of 64x64
    imgs = np.zeros((4, 64, 64, 3), dtype=np.uint8)
    imgs[2, 0, 0] = [42, 0, 0]

    await client.get_actions({"0@ur10e": _state((0.0,) * 6)}, schema, images={"wrist": imgs})

    vid = server.last_observation["video"]["wrist"]
    assert vid.shape == (1, 4, 64, 64, 3)
    assert vid[0, 2, 0, 0, 0] == 42


# ---------------------------------------------------------------------------
# Action decoding tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_action_decoding_single_arm(actions_from) -> None:
    """Action response with matching key should decode to ActionChunk."""
    # 4-step action horizon, 6 joints
    action_arr = np.random.randn(1, 4, 6).astype(np.float32)

    result = await actions_from(({"arm": action_arr}, {"dt_ms": 50.0}))

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


@pytest.mark.asyncio
async def test_action_decoding_dual_arm(server, client) -> None:
    """Both arms should be decoded from a dual-arm response."""
    server.action_response = (
        {
            "left_arm": np.random.randn(1, 4, 6).astype(np.float32),
            "right_arm": np.random.randn(1, 4, 6).astype(np.float32),
        },
        {"dt_ms": 33.0},
    )
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left_arm", source=_mg("0@left")),
            Observation.joint_positions("right_arm", source=_mg("0@right")),
        ]
    )
    result = await client.get_actions(
        {"0@left": _state((0.0,) * 6), "0@right": _state((0.0,) * 6)},
        schema,
    )

    assert "0@left" in result.joints
    assert "0@right" in result.joints
    assert len(result.joints["0@left"]) == 4
    assert len(result.joints["0@right"]) == 4


@pytest.mark.asyncio
async def test_ping(client) -> None:
    assert await client.ping() is True


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
        client = Gr00tPolicyClient(host="127.0.0.1", port=port, control_freq=30.0)
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
async def test_io_actions_come_from_the_chunks_first_step() -> None:
    """IO fires when the policy's first step is due, so it carries that step's value."""
    from novapolicy.schema import BoolMapping

    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    gripper = np.array([[[0.0], [0.0], [1.0]]], dtype=np.float32)  # (B=1, T=3, D=1)
    server.action_response = ({"arm": np.zeros((1, 3, 6), np.float32), "gripper": gripper}, {})
    server.start()
    try:
        mg = _mg()
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.io(
                    "gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=1.0)
                ),
            ]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port, control_freq=30.0)
        await client.connect([mg.id])
        result = await client.get_actions({mg.id: _state((0.0,) * 6)}, schema)
        assert result.ios == {mg.id: {"digital_out[0]": False}}
        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("entry", "field", "empty"),
    [
        (_gripper, "ios", None),
        (lambda: Observation.tcp("eef", source=_mg(), action=True), "tcp", {}),
    ],
    ids=["io", "tcp"],
)
async def test_an_action_missing_from_the_answer_stays_empty(
    actions_from, entry, field, empty
) -> None:
    """If GR00T doesn't return an IO or TCP key, that part of the chunk is empty."""
    response = ({"arm": np.zeros((1, 1, 6), dtype=np.float32)}, {"dt_ms": 33.0})

    result = await actions_from(response, entry(), pose=Pose((0.0, 0.0, 0.0, 0.0, 0.0, 0.0)))

    assert getattr(result, field) == empty


@pytest.mark.asyncio
async def test_tcp_action_decoding(actions_from) -> None:
    """TCP actions from GR00T response should be decoded into ActionChunk.tcp."""
    # GR00T returns joint action + TCP action as numpy arrays
    joint_arr = np.zeros((1, 4, 6), dtype=np.float32)
    tcp_arr = np.array(
        [[[100.0, 200.0, 300.0, 0.1, 0.2, 0.3], [101.0, 201.0, 301.0, 0.11, 0.21, 0.31]]],
        dtype=np.float32,
    )  # (1, 2, 6)

    # Need a state with a pose for the observation
    result = await actions_from(
        ({"arm": joint_arr, "eef": tcp_arr}, {"dt_ms": 33.0}),
        Observation.tcp("eef", source=_mg(), action=True),
        pose=Pose((100.0, 200.0, 300.0, 0.1, 0.2, 0.3)),
    )

    assert isinstance(result, ActionChunk)
    assert "0@ur10e" in result.tcp
    assert len(result.tcp["0@ur10e"]) == 2  # 2 timesteps
    np.testing.assert_allclose(
        result.tcp["0@ur10e"][0],
        [100.0, 200.0, 300.0, 0.1, 0.2, 0.3],
        atol=1e-5,
    )


@pytest.mark.asyncio
async def test_validate_schema_passes_when_keys_match(server, client) -> None:
    """validate_schema passes when the schema provides all required keys."""
    server._modality_config = {
        "state": _modality(["arm_joints"]),
        "video": _modality(["cam_left"]),
        "language": _modality(["annotation.language.language_instruction"]),
    }
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm_joints", source=_mg()),
            Observation.image("cam_left", source=MagicMock()),
            Observation.constant("language", value="Pick up the box."),
        ]
    )
    await client.validate_schema(schema)  # should not raise


@pytest.mark.asyncio
async def test_validate_schema_passes_when_state_key_is_io_observation(server, client) -> None:
    """validate_schema accepts IO observations as state keys."""
    server._modality_config = {"state": _modality(["gripper"])}
    schema = PolicySchema(
        observations=[Observation.io("gripper", source=_mg(), io="digital_out[0]")]
    )
    await client.validate_schema(schema)  # should not raise


@pytest.mark.asyncio
async def test_joint_velocity_and_pose_observations_feed_server_state_keys(server, client) -> None:
    """Observation.joint_velocities and Observation.pose count as state and are sent."""
    server._modality_config = {"state": _modality(["arm", "arm_vel", "goal"])}
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.joint_velocities("arm_vel", source=mg),
            Observation.pose("goal", source=Pose((500.0, 0.0, 300.0, 0.0, 0.0, 0.0))),
        ]
    )
    await client.validate_schema(schema)  # every server key is fed
    await client.get_actions({mg.id: _state((0.0,) * 6)}, schema)
    state = server.last_observation["state"]
    assert state["arm_vel"].shape == (1, 1, 6)
    np.testing.assert_allclose(state["goal"][0, 0], [500.0, 0.0, 300.0, 0.0, 0.0, 0.0])


@pytest.mark.asyncio
async def test_joint_observations_apply_the_schema_operators() -> None:
    """The executor inverts ops on the actions, so GR00T has to apply them on the observations:
    with Rad2Deg the policy sees degrees in and its degrees are read back as degrees."""
    from novapolicy.ops import Rad2Deg

    port = _find_free_port()
    server = _RecordingGr00tServer(port)
    server._modality_config = {
        "state": {"__ModalityConfig_class__": True, "as_json": {"modality_keys": ["arm"]}},
    }
    server.start()
    try:
        mg = _mg()
        schema = PolicySchema(
            observations=[Observation.joint_positions("arm", source=mg, ops=[Rad2Deg()])]
        )
        client = Gr00tPolicyClient(host="127.0.0.1", port=port, control_freq=30.0)
        await client.connect([mg.id])
        await client.get_actions({mg.id: _state((math.pi / 2,) * 6)}, schema)
        np.testing.assert_allclose(server.last_observation["state"]["arm"][0, 0], [90.0] * 6)
        await client.close()
    finally:
        server.close()


@pytest.mark.asyncio
async def test_a_tcp_coordinate_system_without_orientation_type_is_refused(server, client) -> None:
    """GR00T's tcp_format only applies to world poses; a frame alone would change the format."""
    server._modality_config = {"state": _modality(["eef"])}
    schema = PolicySchema(
        observations=[Observation.tcp("eef", source=_mg(), coordinate_system="table")]
    )
    with pytest.raises(ValueError, match="coordinate_system but no orientation_type"):
        await client.validate_schema(schema)


@pytest.mark.asyncio
async def test_validate_schema_rejects_malformed_modality_metadata(server, client) -> None:
    """Remote modality entries must expose LeRobot's serialized as_json payload."""
    server._modality_config = {"state": {"modality_keys": ["arm"]}}
    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=_mg())])
    with pytest.raises(TypeError, match="as_json"):
        await client.validate_schema(schema)


@pytest.mark.asyncio
async def test_validate_schema_fails_on_missing_state_key(server, client) -> None:
    """validate_schema raises ValueError when a state key is missing."""
    server._modality_config = {"state": _modality(["left_arm", "right_arm"])}
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left_arm", source=_mg()),
            # missing "right_arm"
        ]
    )
    with pytest.raises(ValueError, match="right_arm"):
        await client.validate_schema(schema)


@pytest.mark.asyncio
async def test_validate_schema_fails_on_missing_image_key(server, client) -> None:
    """validate_schema raises ValueError when a video key is missing."""
    server._modality_config = {
        "state": _modality(["arm"]),
        "video": _modality(["cam_left", "cam_right"]),
    }
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=_mg()),
            Observation.image("cam_left", source=MagicMock()),
            # missing "cam_right"
        ]
    )
    with pytest.raises(ValueError, match="cam_right"):
        await client.validate_schema(schema)


def _msgpack_numpy(value: np.ndarray | np.generic, *, bytes_keys: bool = True) -> dict:
    """``value`` the way msgpack-numpy packs it, as newer GR00T servers answer."""
    array = np.asarray(value)
    packed: dict = {
        "nd": isinstance(value, np.ndarray),
        "type": array.dtype.str,
        "data": array.tobytes(),
    }
    if packed["nd"]:
        packed |= {"kind": b"", "shape": list(array.shape)}
    return {key.encode(): item for key, item in packed.items()} if bytes_keys else packed


@pytest.mark.asyncio
async def test_an_action_this_client_cannot_decode_raises(actions_from) -> None:
    """A server that changed its array serialization must not leave the robot silently idle."""
    undecodable = {"format": "unknown", "values": [0.0] * 6}

    with pytest.raises(TypeError, match="'arm' must be an array"):
        await actions_from(({"arm": undecodable}, {"dt_ms": 33.0}))


@pytest.mark.asyncio
async def test_an_answer_without_any_expected_action_raises(actions_from) -> None:
    with pytest.raises(ValueError, match="none of the expected actions"):
        await actions_from(({"other": np.zeros((1, 1, 6), dtype=np.float32)}, {"dt_ms": 33.0}))


@pytest.mark.asyncio
async def test_an_n1_5_style_answer_names_the_protocol_it_needs(actions_from) -> None:
    """N1/N1.5 servers answer get_action with the action dict alone."""
    with pytest.raises(TypeError, match=r"N1\.6"):
        await actions_from({"arm": np.zeros((1, 1, 6), dtype=np.float32)})


@pytest.mark.asyncio
async def test_an_io_only_answer_is_a_valid_one(actions_from) -> None:
    """A model that only drives a gripper answers without the joint action."""
    response = ({"gripper": np.array([[[1.0]]], dtype=np.float32)}, {})

    result = await actions_from(response, _gripper())

    assert result.joints == {}
    assert result.ios == {"0@ur10e": {"digital_out[0]": True}}


@pytest.mark.asyncio
async def test_an_action_without_the_batch_axis_decodes(actions_from) -> None:
    steps = np.arange(12, dtype=np.float32).reshape(2, 6)  # (horizon, dof)

    result = await actions_from(({"arm": steps}, {"dt_ms": 33.0}))

    assert result.joints == {"0@ur10e": steps.tolist()}


@pytest.mark.asyncio
async def test_a_key_shared_by_two_arms_gives_each_arm_its_own_joints(client) -> None:
    """The server echoes the concatenated state; each arm must get its own half back."""
    schema = PolicySchema(
        observations=[Observation.joint_positions("arms", source=[_mg("0@left"), _mg("0@right")])]
    )
    left_joints = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    right_joints = (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0)
    result = await client.get_actions(
        {"0@left": _state(left_joints), "0@right": _state(right_joints)}, schema
    )

    np.testing.assert_allclose(result.joints["0@left"], [left_joints] * 4, atol=1e-6)
    np.testing.assert_allclose(result.joints["0@right"], [right_joints] * 4, atol=1e-6)


@pytest.mark.asyncio
async def test_the_client_takes_its_keys_from_the_server(server, client) -> None:
    """After validate_schema, the prompt goes under the server's language key, only the
    server's state keys are sent, a computed state counts, and hook actions land in values."""
    server._modality_config = {
        "state": _modality(["arm_joints", "gripper"]),
        "action": _modality(["eef_delta", "gripper"]),
        "language": _modality(["annotation.human.task_description"]),
    }
    mg = _mg()

    async def gripper(_obs: object) -> dict[str, float]:
        return {"gripper": 0.0}

    async def gripper_command(_chunk: ActionChunk) -> None:
        pass

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm_joints", source=mg, action=False),
            Observation.tcp("tcp_pose", source=mg, tcp="flange"),  # not a model input
            Observation.computed(gripper, keys=["gripper"]),
            Observation.constant("language", value="pick up the cube"),
        ],
        actions=[
            Action.tcp("eef_delta", target=mg, mode="relative"),
            Action.computed(gripper_command, keys=["gripper"]),
        ],
    )
    await client.validate_schema(schema)
    state = _state((0.0,) * 6, pose=Pose((0.0, 0.0, 0.0, 0.0, 0.0, 0.0)))
    server.action_response = (
        {
            "eef_delta": np.zeros((1, 16, 6), np.float32),
            "gripper": -np.ones((1, 16, 1), np.float32),
        },
        {},
    )
    chunk = await client.get_actions({mg.id: state}, schema)
    obs = server.last_observation
    assert obs["language"] == {"annotation.human.task_description": [["pick up the cube"]]}
    assert sorted(obs["state"]) == ["arm_joints", "gripper"]
    assert len(chunk.values["gripper"]) == 16  # for the Action.computed hook
    assert chunk.values["gripper"][0] == [-1.0]


_STATISTICS = Path(__file__).parents[1] / "unit" / "data" / "gr00t_statistics.json"


async def _first_call_with(
    server: _RecordingGr00tServer, joints: tuple[float, ...], **client_kwargs: object
) -> None:
    server._modality_config = {
        "state": _modality(["arm_joints", "gripper"]),
        "action": _modality(["eef_delta", "gripper"]),
    }
    client = Gr00tPolicyClient(
        host="127.0.0.1",
        port=server.port,
        control_freq=30.0,
        statistics=_STATISTICS,
        **client_kwargs,
    )
    try:
        mg = _mg()

        async def gripper(_obs: object) -> dict[str, float]:
            return {"gripper": 0.0}

        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm_joints", source=mg, action=False),
                Observation.computed(gripper, keys=["gripper"]),
            ],
            actions=[Action.joint_positions("arm_joints", target=mg)],
        )
        await client.connect([mg.id])
        await client.validate_schema(schema)
        server.action_response = ({"arm_joints": np.zeros((1, 4, 6), np.float32)}, {})
        await client.get_actions({mg.id: _state(joints)}, schema)
    finally:
        await client.close()


_TRAINED = (1.0, -1.8, 2.0, -1.9, -1.4, -0.5)  # radians, inside the checkpoint's range


@pytest.mark.asyncio
async def test_a_pose_in_the_trained_units_passes_the_statistics_check(server) -> None:
    await _first_call_with(server, _TRAINED)


@pytest.mark.asyncio
async def test_degrees_against_a_radian_checkpoint_are_refused_before_moving(server) -> None:
    degrees = tuple(math.degrees(q) for q in _TRAINED)
    with pytest.raises(ValueError, match="implausibly far outside"):
        await _first_call_with(server, degrees)


@pytest.mark.asyncio
async def test_an_unknown_embodiment_is_named(server) -> None:
    with pytest.raises(ValueError, match="no embodiment"):
        await _first_call_with(server, _TRAINED, embodiment="other_robot")


def test_the_step_rate_defaults_to_30_hz() -> None:
    """GR00T servers do not report their rate; without control_freq it is 30 Hz."""
    assert Gr00tPolicyClient(host="127.0.0.1", port=1)._dt_ms == pytest.approx(1000.0 / 30.0)
    assert Gr00tPolicyClient(host="127.0.0.1", port=1, control_freq=20)._dt_ms == 50.0
