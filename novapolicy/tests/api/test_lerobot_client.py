"""API/protocol tests for LeRobotPolicyClient."""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import math
import pickle
import threading
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from novapolicy.mapping.ops import Rad2Deg
from novapolicy.mapping.schema import BoolMapping, Observation, PolicySchema
from novapolicy.tests.doubles import mg as _mg, state as _state, tcp_state as _tcp_state

client_module = pytest.importorskip("novapolicy.providers.lerobot.client")
grpc = pytest.importorskip("grpc")
helpers_module = pytest.importorskip("lerobot.async_inference.helpers")
services_pb2 = pytest.importorskip("lerobot.transport.services_pb2")
services_pb2_grpc = pytest.importorskip("lerobot.transport.services_pb2_grpc")
torch = pytest.importorskip("torch")
LeRobotPolicyClient = client_module.LeRobotPolicyClient
RemotePolicyConfig = helpers_module.RemotePolicyConfig
TimedAction = helpers_module.TimedAction
TimedObservation = helpers_module.TimedObservation


class _RecordingLeRobotService(services_pb2_grpc.AsyncInferenceServicer):
    """Real LeRobot gRPC service that records wire payloads and returns scripted actions."""

    def __init__(self) -> None:
        self.ready_calls = 0
        self.policy_setup_calls = 0
        self.get_actions_calls = 0
        self.policy_setup: RemotePolicyConfig | None = None
        self.observations: list[TimedObservation] = []
        self.action_values = [
            [10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 1.0],
            [20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 0.0],
        ]
        self.block_get_actions_call: int | None = None
        self.get_actions_started = threading.Event()
        self.get_actions_finished = threading.Event()
        self.release_get_actions = threading.Event()
        self.release_get_actions.set()

    def Ready(self, _request: object, _context: object) -> Any:  # ruff: ignore[invalid-function-name]
        self.ready_calls += 1
        return services_pb2.Empty()

    def SendPolicyInstructions(self, request: Any, _context: object) -> Any:  # ruff: ignore[invalid-function-name]
        self.policy_setup_calls += 1
        self.policy_setup = pickle.loads(request.data)  # ruff: ignore[suspicious-pickle-usage] - trusted local fixture.
        return services_pb2.Empty()

    def SendObservations(  # ruff: ignore[invalid-function-name]
        self,
        request_iterator: Iterable[Any],
        _context: object,
    ) -> Any:
        payload = b"".join(message.data for message in request_iterator)
        self.observations.append(pickle.loads(payload))  # ruff: ignore[suspicious-pickle-usage] - trusted local fixture.
        return services_pb2.Empty()

    def GetActions(self, _request: object, context: Any) -> Any:  # ruff: ignore[invalid-function-name]
        self.get_actions_calls += 1
        context.add_callback(self.get_actions_finished.set)
        if self.get_actions_calls == self.block_get_actions_call:
            self.get_actions_started.set()
            if not self.release_get_actions.wait(timeout=2.0):
                raise TimeoutError("test did not release the scripted LeRobot response")

        start = self.observations[-1].timestep
        actions = [
            TimedAction(
                timestamp=0.0,
                timestep=start + index,
                action=torch.tensor(values, dtype=torch.float32),
            )
            for index, values in enumerate(self.action_values)
        ]
        return services_pb2.Actions(data=pickle.dumps(actions))


@dataclass
class _FakeLeRobot:
    stub: _RecordingLeRobotService
    address: str


@pytest.fixture
def fake_lerobot() -> Iterator[_FakeLeRobot]:
    executor = ThreadPoolExecutor(max_workers=4)
    server = grpc.server(executor)
    stub = _RecordingLeRobotService()
    services_pb2_grpc.add_AsyncInferenceServicer_to_server(stub, server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    fake = _FakeLeRobot(
        stub=stub,
        address=f"127.0.0.1:{port}",
    )
    try:
        yield fake
    finally:
        stub.release_get_actions.set()
        server.stop(0).wait(timeout=2.0)
        executor.shutdown(wait=True)


def _schema(mg: MagicMock, image_source: object | None = None) -> PolicySchema:
    return PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.io("gripper", source=mg, io="digital_out[0]", mapping=BoolMapping()),
            Observation.image("cam_scene_1", source=image_source or MagicMock()),
        ]
    )


def _tcp_schema(mg: MagicMock) -> PolicySchema:
    return PolicySchema(
        observations=[
            Observation.tcp("eef", source=mg, action=True),
            Observation.io("gripper", source=mg, io="digital_out[0]", mapping=BoolMapping()),
        ]
    )


_FRAME = np.zeros((120, 160, 3), dtype=np.uint8)


def _inputs(
    mg: MagicMock, joints: tuple[float, ...] = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0), gripper: bool = True
) -> dict[str, Any]:
    """One observation's states, camera frame and IO, as keyword arguments."""
    return {
        "states": {mg.id: _state(joints)},
        "images": {"cam_scene_1": _FRAME},
        "io_values": {"digital_out[0]": gripper},
    }


def test_control_freq_defaults_to_30_hz_and_says_so(caplog) -> None:
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", policy_type="act", actions_per_chunk=11)
    assert client.dt_ms == pytest.approx(1000.0 / 30.0)
    assert any("got no control_freq" in r.message for r in caplog.records)


@pytest.mark.asyncio
async def test_get_actions_sends_lerobot_async_protocol_and_decodes_chunk(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg("0@cobot")
    schema = _schema(mg)
    client = LeRobotPolicyClient(
        fake_lerobot.address,
        "/server-only/checkpoint",
        policy_type="act",
        control_freq=15,
        actions_per_chunk=8,
        device="cuda",
    )

    await client.connect([mg.id])
    await client.validate_schema(schema)
    chunk = await client.get_actions(schema=schema, **_inputs(mg))

    assert chunk.dt_ms == pytest.approx(1000.0 / 15.0)
    assert chunk.joints == {
        mg.id: [
            [10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
            [20.0, 21.0, 22.0, 23.0, 24.0, 25.0],
        ]
    }
    assert chunk.ios == {mg.id: {"digital_out[0]": True}}

    assert fake_lerobot.stub.ready_calls == 1

    setup = fake_lerobot.stub.policy_setup
    assert setup == RemotePolicyConfig(
        policy_type="act",
        pretrained_name_or_path="/server-only/checkpoint",
        actions_per_chunk=8,
        device="cuda",
        lerobot_features={
            "observation.state": {
                "dtype": "float32",
                "shape": (7,),
                "names": ["arm_1", "arm_2", "arm_3", "arm_4", "arm_5", "arm_6", "gripper"],
            },
            "observation.images.cam_scene_1": {
                "dtype": "image",
                "shape": (120, 160, 3),
                "names": ["height", "width", "channels"],
            },
        },
    )

    assert len(fake_lerobot.stub.observations) == 1
    observation = fake_lerobot.stub.observations[0]
    assert observation.timestep == 0
    assert observation.must_go is True
    assert observation.observation["gripper"] == 1.0
    arm = [observation.observation[f"arm_{idx}"] for idx in range(1, 7)]
    assert arm == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    np.testing.assert_array_equal(observation.observation["cam_scene_1"], _FRAME)
    await client.close()


@pytest.mark.asyncio
async def test_get_actions_decodes_tcp_targets_and_tcp_state_features(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg("0@cobot")
    schema = _tcp_schema(mg)
    client = LeRobotPolicyClient(
        fake_lerobot.address,
        "model",
        policy_type="act",
        control_freq=20,
        actions_per_chunk=2,
    )
    state = _tcp_state((100.0, 200.0, 300.0, 0.1, 0.2, 0.3))

    await client.connect([mg.id])
    await client.validate_schema(schema)
    chunk = await client.get_actions(
        {mg.id: state},
        schema,
        io_values={"digital_out[0]": False},
    )

    assert chunk.joints == {}
    assert chunk.tcp == {
        mg.id: [
            [10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
            [20.0, 21.0, 22.0, 23.0, 24.0, 25.0],
        ]
    }
    assert chunk.ios == {mg.id: {"digital_out[0]": True}}
    assert chunk.dt_ms == pytest.approx(50.0)

    assert fake_lerobot.stub.policy_setup is not None
    assert fake_lerobot.stub.policy_setup.lerobot_features["observation.state"] == {
        "dtype": "float32",
        "shape": (7,),
        "names": ["eef_x", "eef_y", "eef_z", "eef_rx", "eef_ry", "eef_rz", "gripper"],
    }
    observation = fake_lerobot.stub.observations[0].observation
    eef = [observation[f"eef_{suffix}"] for suffix in ("x", "y", "z", "rx", "ry", "rz")]
    assert eef == [100.0, 200.0, 300.0, 0.1, 0.2, 0.3]
    await client.close()


@pytest.mark.asyncio
async def test_prepare_sends_policy_instructions_before_first_inference(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg("0@cobot")
    schema = _schema(mg)
    client = LeRobotPolicyClient(
        fake_lerobot.address, "model", policy_type="act", control_freq=15, actions_per_chunk=8
    )

    await client.prepare(schema=schema, **_inputs(mg))

    assert fake_lerobot.stub.policy_setup_calls == 1
    assert fake_lerobot.stub.observations == []

    await client.get_actions(schema=schema, **_inputs(mg))

    assert fake_lerobot.stub.policy_setup_calls == 1
    assert len(fake_lerobot.stub.observations) == 1
    await client.close()


@pytest.mark.asyncio
async def test_get_actions_requires_actual_image_frame_for_feature_metadata(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg("0@cobot")
    client = LeRobotPolicyClient(
        fake_lerobot.address, "model", policy_type="act", control_freq=15, actions_per_chunk=8
    )

    with pytest.raises(ValueError, match="needs the first camera frame"):
        await client.get_actions(
            {mg.id: _state((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))},
            _schema(mg),
            images=None,
            io_values={"digital_out[0]": False},
        )
    assert fake_lerobot.stub.ready_calls == 1
    await client.close()


@pytest.mark.asyncio
async def test_validate_schema_rejects_schema_without_joint_action() -> None:
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", control_freq=15, actions_per_chunk=8)
    schema = PolicySchema(observations=[Observation.image("cam_scene_1", source=MagicMock())])

    with pytest.raises(ValueError, match="requires at least one joint or TCP action"):
        await client.validate_schema(schema)


@pytest.mark.asyncio
async def test_validate_schema_rejects_joint_and_tcp_control_for_the_same_group() -> None:
    mg = _mg("0@cobot")
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", control_freq=15, actions_per_chunk=8)
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.tcp("eef", source=mg, action=True),
        ]
    )

    with pytest.raises(ValueError, match="both joint and TCP actions"):
        await client.validate_schema(schema)


@pytest.mark.asyncio
async def test_close_allows_protocol_reconnection(fake_lerobot: _FakeLeRobot) -> None:
    client = LeRobotPolicyClient(
        fake_lerobot.address, "model", policy_type="act", control_freq=15, actions_per_chunk=8
    )

    await client.connect([])
    await client.close()
    await client.connect([])
    await client.close()

    assert fake_lerobot.stub.ready_calls == 2


# ---------------------------------------------------------------------------
# Deriving execution settings from the checkpoint
# ---------------------------------------------------------------------------


def _write_checkpoint(tmp_path, **overrides):
    config = {
        "type": "act",
        "chunk_size": 16,
        "n_action_steps": 8,
        "input_features": {
            "observation.state": {"type": "STATE", "shape": [7]},
            "observation.images.cam_scene_1": {"type": "VISUAL", "shape": [3, 120, 160]},
        },
        "output_features": {"action": {"type": "ACTION", "shape": [7]}},
    }
    config.update(overrides)
    (tmp_path / "config.json").write_text(json.dumps(config), encoding="utf-8")
    return tmp_path


def _write_instruction_preprocessor(tmp_path) -> None:
    """Give the checkpoint the pipeline an instruction-conditioned policy ships."""
    steps = [{"registry_name": "tokenizer_processor"}, {"registry_name": "device_processor"}]
    (tmp_path / "policy_preprocessor.json").write_text(
        json.dumps({"steps": steps}), encoding="utf-8"
    )


class _FakeCamera:
    """Camera source that can report and change its read-time size."""

    def __init__(self, resize: tuple[int, int] | None = None) -> None:
        self._resize = resize

    @property
    def resize(self) -> tuple[int, int] | None:
        return self._resize

    def set_resize(self, size: tuple[int, int] | None) -> None:
        self._resize = size

    async def connect(self) -> None: ...

    def read(self, max_age_s: float = 5.0):
        return np.zeros((120, 160, 3), dtype=np.uint8)

    async def disconnect(self) -> None: ...


_SERVER_ONLY = "/server-only/checkpoint"


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("path", "settings", "expected", "warning"),
    [
        pytest.param(
            None,
            {},
            {"_policy_type": "act", "_actions_per_chunk": 16, "n_action_steps": 8},
            None,
            id="derived-from-the-checkpoint",
        ),
        pytest.param(
            _SERVER_ONLY,
            {"policy_type": "act", "config_path": None},
            {"_actions_per_chunk": 16, "_pretrained_name_or_path": _SERVER_ONLY},
            None,
            id="config-path-derives-while-the-server-loads-its-own",
        ),
        pytest.param(
            None,
            {"policy_type": "act", "actions_per_chunk": 16, "n_action_steps": 8},
            {"_actions_per_chunk": 16},
            None,
            id="an-override-that-agrees",
        ),
        pytest.param(
            None,
            {"policy_type": "act", "n_action_steps": 4},
            {"n_action_steps": 4},
            "differs from the checkpoint",
            id="a-shortened-horizon-warns",
        ),
        pytest.param(
            _SERVER_ONLY,
            {"policy_type": "act", "actions_per_chunk": 11},
            {"_actions_per_chunk": 11},
            "not readable by the client",
            id="a-server-only-checkpoint-falls-back-to-explicit-settings",
        ),
    ],
)
async def test_execution_settings_are_resolved_against_the_checkpoint(
    fake_lerobot, tmp_path, caplog, path, settings, expected, warning
) -> None:
    """``None`` as the path or ``config_path`` stands for the checkpoint in ``tmp_path``."""
    checkpoint = str(_write_checkpoint(tmp_path))
    settings = {key: value or checkpoint for key, value in settings.items()}
    client = LeRobotPolicyClient(
        fake_lerobot.address, path or checkpoint, control_freq=15.0, **settings
    )

    with caplog.at_level("WARNING"):
        await client.connect([])

    assert {name: getattr(client, name) for name in expected} == expected
    if warning is not None:
        assert warning in caplog.text
    await client.close()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("path", "settings", "error"),
    [
        pytest.param(
            None, {"policy_type": "diffusion"}, "contradicts the checkpoint", id="policy-type"
        ),
        pytest.param(
            None,
            {"policy_type": "act", "actions_per_chunk": 32},
            "exceeds the checkpoint's action chunk",
            id="actions-per-chunk-above-chunk-size",
        ),
        pytest.param(
            None,
            {"policy_type": "act", "n_action_steps": 32},
            "exceeds the checkpoint's action chunk",
            id="n-action-steps-above-chunk-size",
        ),
        pytest.param(
            _SERVER_ONLY,
            {},
            "actions_per_chunk cannot be derived",
            id="a-server-only-checkpoint-without-an-explicit-chunk",
        ),
    ],
)
async def test_contradicting_or_missing_settings_keep_raising_on_reconnect(
    fake_lerobot, tmp_path, path, settings, error
) -> None:
    """A failed resolution must not leave the client running on half-applied settings."""
    checkpoint = str(_write_checkpoint(tmp_path))
    client = LeRobotPolicyClient(
        fake_lerobot.address, path or checkpoint, control_freq=15.0, **settings
    )

    for _ in range(2):
        with pytest.raises(ValueError, match=error):
            await client.connect([])


@pytest.mark.asyncio
async def test_a_camera_frame_size_mismatch_warns_and_keeps_running(
    fake_lerobot, tmp_path, caplog
) -> None:
    """Frame size is fixed at the camera stream, so a mismatch must not block a run."""
    mg = _mg("0@cobot")
    schema = _schema(mg)
    client = LeRobotPolicyClient(
        fake_lerobot.address, str(_write_checkpoint(tmp_path)), control_freq=15.0
    )

    await client.connect([])
    with caplog.at_level("WARNING"):
        await client.prepare(
            {mg.id: _state((1.0,) * 6)},
            schema,
            images={"cam_scene_1": np.zeros((480, 640, 3), dtype=np.uint8)},
            io_values={"digital_out[0]": True},
        )

    assert "aspect ratio matches" in caplog.text
    assert fake_lerobot.stub.policy_setup_calls == 1
    await client.close()


@pytest.mark.asyncio
async def test_schema_mismatch_fails_before_the_setup_message_is_sent(
    fake_lerobot, tmp_path
) -> None:
    """A 6-DOF schema against a 7-wide checkpoint must not reach the server."""
    mg = _mg("0@cobot")
    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=mg)])
    client = LeRobotPolicyClient(
        fake_lerobot.address, str(_write_checkpoint(tmp_path)), control_freq=15.0
    )

    await client.connect([])
    with pytest.raises(ValueError, match="does not match the LeRobot checkpoint"):
        await client.prepare({mg.id: _state((0.0,) * 6)}, schema)

    assert fake_lerobot.stub.policy_setup_calls == 0
    await client.close()


@pytest.mark.asyncio
async def test_the_checkpoint_is_read_once_across_reconnects(
    fake_lerobot, tmp_path, monkeypatch
) -> None:
    """Reconnecting must not re-read (or re-download) the checkpoint."""
    checkpoint = _write_checkpoint(tmp_path)
    calls = 0
    real = client_module.try_load_execution_settings

    def counting_load(path):
        nonlocal calls
        calls += 1
        return real(path)

    monkeypatch.setattr(client_module, "try_load_execution_settings", counting_load)
    client = LeRobotPolicyClient(fake_lerobot.address, str(checkpoint), control_freq=15.0)

    await client.connect([])
    await client.connect([])

    assert calls == 1
    await client.close()


def _degrees_schema(mg: MagicMock) -> PolicySchema:
    """A six-joint arm the policy sees in degrees."""
    return PolicySchema(
        observations=[Observation.joint_positions("arm", source=mg, ops=[Rad2Deg()])]
    )


def _six_dof_checkpoint(tmp_path):
    return _write_checkpoint(
        tmp_path,
        input_features={"observation.state": {"type": "STATE", "shape": [6]}},
        output_features={"action": {"type": "ACTION", "shape": [6]}},
    )


@pytest.mark.asyncio
async def test_a_degrees_dataset_observes_in_degrees_and_commands_in_radians(
    fake_lerobot, tmp_path
) -> None:
    """End to end: ops convert on the way in and are inverted on the way out."""
    mg = _mg("0@cobot")
    schema, checkpoint = _degrees_schema(mg), _six_dof_checkpoint(tmp_path)
    client = LeRobotPolicyClient(fake_lerobot.address, str(checkpoint), control_freq=15.0)
    fake_lerobot.stub.action_values = [[180.0] * 6, [-90.0] * 6]

    await client.connect([])
    chunk = await client.get_actions({mg.id: _state((math.pi,) * 6)}, schema)

    # The server saw degrees...
    sent = fake_lerobot.stub.observations[-1].observation
    assert [sent[f"arm_{i}"] for i in range(1, 7)] == pytest.approx([180.0] * 6)
    # ...and the chunk comes back in the policy's units, which the executor
    # inverts through the schema before anything reaches the robot.
    assert chunk.joints[mg.id][0] == pytest.approx([180.0] * 6)
    commanded = schema.apply_inverse_ops(chunk)
    assert commanded.joints[mg.id][0] == pytest.approx([math.pi] * 6)
    assert commanded.joints[mg.id][1] == pytest.approx([-math.pi / 2] * 6)
    await client.close()


@pytest.mark.asyncio
async def test_a_units_mismatch_is_caught_against_the_checkpoints_statistics(
    fake_lerobot, tmp_path
) -> None:
    """A stray Rad2Deg against a radian-trained checkpoint is caught before moving.

    Note the asymmetry: bounds catch values that are too *large* for the
    training range — degrees where radians were expected. The reverse, radians
    where degrees were expected, sits comfortably inside a degree range and
    cannot be caught this way.
    """
    save_file = pytest.importorskip("safetensors.numpy").save_file
    mg = _mg("0@cobot")
    schema, checkpoint = _degrees_schema(mg), _six_dof_checkpoint(tmp_path)
    (checkpoint / "policy_preprocessor.json").write_text(
        json.dumps({
            "name": "policy_preprocessor",
            "steps": [
                {
                    "registry_name": "normalizer_processor",
                    "config": {},
                    "state_file": "normalizer_processor.safetensors",
                }
            ],
        }),
        encoding="utf-8",
    )
    save_file(
        {
            "observation.state.min": np.full(6, -math.pi, dtype=np.float32),
            "observation.state.max": np.full(6, math.pi, dtype=np.float32),
        },
        checkpoint / "normalizer_processor.safetensors",
    )

    client = LeRobotPolicyClient(fake_lerobot.address, str(checkpoint), control_freq=15.0)
    await client.connect([])

    with pytest.raises(ValueError, match="implausibly far outside"):
        await client.get_actions({mg.id: _state((math.pi,) * 6)}, schema)

    await client.close()


# ---------------------------------------------------------------------------
# Language instruction
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_an_instruction_conditioned_checkpoint_refuses_to_start_without_a_task(
    fake_lerobot, tmp_path
) -> None:
    """Without the task LeRobot's server fails every inference and says so only in
    its own log, so the client stops at connect instead.
    """
    checkpoint = _write_checkpoint(tmp_path)
    _write_instruction_preprocessor(checkpoint)
    client = LeRobotPolicyClient(fake_lerobot.address, str(checkpoint), control_freq=15.0)

    with pytest.raises(ValueError, match="language instruction"):
        await client.connect([])


@pytest.mark.asyncio
async def test_the_task_travels_with_every_observation(fake_lerobot, tmp_path) -> None:
    checkpoint = _write_checkpoint(tmp_path)
    _write_instruction_preprocessor(checkpoint)
    mg = _mg("0@cobot")
    schema = _schema(mg, image_source=_FakeCamera())
    client = LeRobotPolicyClient(
        fake_lerobot.address, str(checkpoint), task="pick up the cube", control_freq=15
    )

    await client.connect([mg.id])
    await client.get_actions(schema=schema, **_inputs(mg))

    assert fake_lerobot.stub.observations[-1].observation["task"] == "pick up the cube"
    await client.close()


@pytest.mark.asyncio
async def test_a_checkpoint_without_an_instruction_sends_no_task(fake_lerobot) -> None:
    mg = _mg("0@cobot")
    schema = _schema(mg)
    client = LeRobotPolicyClient(
        fake_lerobot.address,
        "/server-only/checkpoint",
        policy_type="act",
        actions_per_chunk=8,
        control_freq=15.0,
    )

    await client.connect([mg.id])
    await client.get_actions(schema=schema, **_inputs(mg))

    assert "task" not in fake_lerobot.stub.observations[-1].observation
    await client.close()


def test_a_blank_task_is_rejected() -> None:
    with pytest.raises(ValueError, match="non-empty"):
        LeRobotPolicyClient("127.0.0.1:8080", "model", task="  ", control_freq=15.0)


@pytest.mark.asyncio
async def test_a_queued_lerobot_client_merges_its_chunks_on_one_timeline(
    fake_lerobot: _FakeLeRobot,
) -> None:
    """For continuous execution LeRobot is wrapped like any provider: the queue numbers the
    steps, resolves them to NOVA units and keeps the steps already sent."""
    from novapolicy import QueuedPolicyClient

    mg = _mg("0@cobot")
    fake_lerobot.stub.action_values = [[float(i)] * 6 + [0.0] for i in range(6)]
    client = QueuedPolicyClient(
        LeRobotPolicyClient(
            fake_lerobot.address, "model", policy_type="act", actions_per_chunk=6, control_freq=15
        )
    )
    await client.connect([mg.id])
    first = await client.get_actions(schema=_schema(mg), **_inputs(mg, (0.0,) * 6, gripper=False))
    client.synchronize_action_timestep(2)
    second = await client.get_actions(schema=_schema(mg), **_inputs(mg, (0.0,) * 6, gripper=False))
    await client.close()
    assert first.action_timestep == 0 and first.resolved
    assert second.action_timestep == 1  # the step before the replaceable one leads the plan
    assert second.retained_prefix_steps >= 1
