"""API/protocol tests for LeRobotPolicyClient."""

from __future__ import annotations

import asyncio
from collections.abc import Iterable, Iterator
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import pickle
import threading
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from novapolicy.schema import BoolMapping, Observation, PolicySchema

client_module = pytest.importorskip("novapolicy.lerobot.client")
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

    def Ready(self, _request: object, _context: object) -> Any:  # noqa: N802
        self.ready_calls += 1
        return services_pb2.Empty()

    def SendPolicyInstructions(self, request: Any, _context: object) -> Any:  # noqa: N802
        self.policy_setup_calls += 1
        self.policy_setup = pickle.loads(request.data)  # noqa: S301 - trusted local fixture.
        return services_pb2.Empty()

    def SendObservations(  # noqa: N802
        self,
        request_iterator: Iterable[Any],
        _context: object,
    ) -> Any:
        payload = b"".join(message.data for message in request_iterator)
        self.observations.append(pickle.loads(payload))  # noqa: S301 - trusted local fixture.
        return services_pb2.Empty()

    def GetActions(self, _request: object, context: Any) -> Any:  # noqa: N802
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


def _mg(mg_id: str = "0@cobot", controller_id: str = "cobot") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = controller_id
    mg._cell = "cell"
    return mg


def _state(joints: tuple[float, ...]) -> MagicMock:
    state = MagicMock()
    state.joints = joints
    state.pose = None
    state.tcp = None
    return state


def _tcp_state(values: tuple[float, float, float, float, float, float]) -> MagicMock:
    state = _state((0.0,) * 6)
    state.pose = SimpleNamespace(position=values[:3], orientation=values[3:])
    return state


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


def test_playback_speed_scales_physical_action_timing() -> None:
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        fps=15,
        playback_speed=0.75,
        actions_per_chunk=11,
    )

    assert client.dt_ms == pytest.approx(1000.0 / 15.0 / 0.75)


def test_playback_speed_must_be_positive() -> None:
    with pytest.raises(ValueError, match="playback_speed must be positive"):
        LeRobotPolicyClient(
            "127.0.0.1:8080",
            "model",
            fps=15,
            playback_speed=0.0,
            actions_per_chunk=11,
        )


@pytest.mark.parametrize("threshold", [0.0, 1.1])
def test_async_queue_refill_threshold_must_be_a_fraction(threshold: float) -> None:
    with pytest.raises(ValueError, match="async_queue_refill_threshold must be in"):
        LeRobotPolicyClient(
            "127.0.0.1:8080",
            "model",
            actions_per_chunk=11,
            async_queue_refill_threshold=threshold,
        )


@pytest.mark.asyncio
async def test_get_actions_sends_lerobot_async_protocol_and_decodes_chunk(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _schema(mg)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    client = LeRobotPolicyClient(
        fake_lerobot.address,
        "/server-only/checkpoint",
        policy_type="act",
        fps=15,
        actions_per_chunk=8,
        device="cuda",
    )

    await client.connect([mg.id])
    await client.validate_schema(schema)
    chunk = await client.get_actions(
        {mg.id: _state((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": True},
    )

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
    assert [observation.observation[f"arm_{idx}"] for idx in range(1, 7)] == [
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
    ]
    np.testing.assert_array_equal(observation.observation["cam_scene_1"], image)
    await client.close()


@pytest.mark.asyncio
async def test_get_actions_decodes_tcp_targets_and_tcp_state_features(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _tcp_schema(mg)
    client = LeRobotPolicyClient(
        fake_lerobot.address,
        "model",
        fps=20,
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
    assert [observation[f"eef_{suffix}"] for suffix in ("x", "y", "z", "rx", "ry", "rz")] == [
        100.0,
        200.0,
        300.0,
        0.1,
        0.2,
        0.3,
    ]
    await client.close()


@pytest.mark.asyncio
async def test_async_queue_refills_and_blends_overlapping_timesteps(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _schema(mg)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    fake_lerobot.stub.action_values = [
        [float(index)] * 6 + [float(index % 2)] for index in range(8)
    ]
    fake_lerobot.stub.block_get_actions_call = 2
    fake_lerobot.stub.release_get_actions.clear()
    client = LeRobotPolicyClient(
        fake_lerobot.address,
        "model",
        fps=15,
        actions_per_chunk=8,
        use_async_queue=True,
        async_queue_refill_threshold=1.0,
    )

    await client.connect([mg.id])
    first = await client.get_actions(
        {mg.id: _state((0.0,) * 6)},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": False},
    )
    assert [step[0] for step in first.joints[mg.id]] == [float(index) for index in range(8)]
    assert first.action_timestep == 0
    assert fake_lerobot.stub.observations[0].timestep == 0
    assert fake_lerobot.stub.observations[0].must_go is True

    started = await asyncio.to_thread(fake_lerobot.stub.get_actions_started.wait, 0.5)
    assert started
    fake_lerobot.stub.get_actions_finished.clear()
    fake_lerobot.stub.release_get_actions.set()
    finished = await asyncio.to_thread(fake_lerobot.stub.get_actions_finished.wait, 0.5)
    assert finished

    replacement = None
    for state_value in (1.0, 2.0, 3.0):
        await asyncio.sleep(0)
        candidate = await client.get_actions(
            {mg.id: _state((state_value,) * 6)},
            schema,
            images={"cam_scene_1": image},
            io_values={"digital_out[0]": False},
        )
        if candidate.joints:
            replacement = candidate
            break

    assert replacement is not None
    assert replacement.action_timestep >= 0
    assert replacement.ios is not None
    assert isinstance(replacement.ios[mg.id]["digital_out[0]"], bool)

    await client.close()


@pytest.mark.asyncio
async def test_async_queue_applies_action_chunk_smoothing_after_aggregation(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _schema(mg)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    client = LeRobotPolicyClient(
        fake_lerobot.address,
        "model",
        actions_per_chunk=3,
        use_async_queue=True,
        async_queue_smoothing=True,
    )
    await client.connect([mg.id])
    fake_lerobot.stub.action_values = [
        [value] * 6 + [io] for value, io in [(0.0, 1.0), (4.0, 0.0), (0.0, 1.0)]
    ]

    chunk = await client.get_actions(
        {mg.id: _state((0.0,) * 6)},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": False},
    )

    assert [step[0] for step in chunk.joints[mg.id]] == pytest.approx([1.25, 1.5, 1.25])
    assert chunk.ios == {mg.id: {"digital_out[0]": True}}

    await client.close()


@pytest.mark.asyncio
async def test_async_queue_keeps_sending_observations_while_refill_is_pending(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _schema(mg)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    fake_lerobot.stub.action_values = [
        [float(index)] * 6 + [float(index % 2)] for index in range(5)
    ]
    fake_lerobot.stub.block_get_actions_call = 2
    fake_lerobot.stub.release_get_actions.clear()
    client = LeRobotPolicyClient(
        fake_lerobot.address,
        "model",
        fps=15,
        actions_per_chunk=5,
        use_async_queue=True,
    )
    await client.connect([mg.id])

    await client.get_actions(
        {mg.id: _state((0.0,) * 6)},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": False},
    )
    second = await client.get_actions(
        {mg.id: _state((1.0,) * 6)},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": False},
    )
    assert second.joints == {}

    started = await asyncio.to_thread(fake_lerobot.stub.get_actions_started.wait, 0.5)
    assert started
    third = await client.get_actions(
        {mg.id: _state((2.0,) * 6)},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": False},
    )

    assert third.joints == {}
    assert fake_lerobot.stub.observations[-1].timestep == 2
    assert fake_lerobot.stub.observations[-1].must_go is False

    fake_lerobot.stub.release_get_actions.set()
    await client.close()


@pytest.mark.asyncio
async def test_prepare_sends_policy_instructions_before_first_inference(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _schema(mg)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    client = LeRobotPolicyClient(fake_lerobot.address, "model", fps=15, actions_per_chunk=8)

    await client.prepare(
        {mg.id: _state((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": True},
    )

    assert fake_lerobot.stub.policy_setup_calls == 1
    assert fake_lerobot.stub.observations == []

    await client.get_actions(
        {mg.id: _state((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": True},
    )

    assert fake_lerobot.stub.policy_setup_calls == 1
    assert len(fake_lerobot.stub.observations) == 1
    await client.close()


@pytest.mark.asyncio
async def test_get_actions_requires_actual_image_frame_for_feature_metadata(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    client = LeRobotPolicyClient(fake_lerobot.address, "model", fps=15, actions_per_chunk=8)

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
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", fps=15, actions_per_chunk=8)
    schema = PolicySchema(observations=[Observation.image("cam_scene_1", source=MagicMock())])

    with pytest.raises(ValueError, match="requires at least one joint or TCP action"):
        await client.validate_schema(schema)


@pytest.mark.asyncio
async def test_validate_schema_rejects_joint_and_tcp_control_for_the_same_group() -> None:
    mg = _mg()
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", fps=15, actions_per_chunk=8)
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
    client = LeRobotPolicyClient(fake_lerobot.address, "model", fps=15, actions_per_chunk=8)

    await client.connect([])
    await client.close()
    await client.connect([])
    await client.close()

    assert fake_lerobot.stub.ready_calls == 2
