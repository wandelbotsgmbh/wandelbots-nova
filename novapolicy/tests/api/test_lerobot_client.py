"""API/protocol tests for LeRobotPolicyClient."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import pickle
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from novapolicy.schema import BoolMapping, Observation, PolicySchema

client_module = pytest.importorskip("novapolicy.lerobot.client")
AsyncQueueAggregation = client_module.AsyncQueueAggregation
LeRobotPolicyClient = client_module.LeRobotPolicyClient


@dataclass
class _RemotePolicyConfig:
    policy_type: str
    pretrained_name_or_path: str
    lerobot_features: dict[str, dict[str, Any]]
    actions_per_chunk: int
    device: str = "cpu"


@dataclass
class _TimedObservation:
    timestamp: float
    observation: dict[str, Any]
    timestep: int
    must_go: bool = False


class _TimedAction:
    def __init__(self, values: list[float], timestep: int = 0) -> None:
        self._values = np.asarray(values, dtype=np.float32)
        self._timestep = timestep

    def get_action(self) -> np.ndarray:
        return self._values

    def get_timestep(self) -> int:
        return self._timestep


class _Message:
    def __init__(self, data: bytes = b"") -> None:
        self.data = data


class _FakeChannel:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _FakeStub:
    def __init__(self, _channel: _FakeChannel) -> None:
        self.ready_calls = 0
        self.policy_setup_calls = 0
        self.policy_setup: _RemotePolicyConfig | None = None
        self.observations: list[_TimedObservation] = []
        self.action_values = [
            [10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 1.0],
            [20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 0.0],
        ]

    def Ready(self, _request: object, *, timeout: float) -> _Message:  # noqa: N802
        self.ready_calls += 1
        return _Message()

    def SendPolicyInstructions(self, request: _Message, *, timeout: float) -> _Message:  # noqa: N802
        self.policy_setup_calls += 1
        self.policy_setup = pickle.loads(request.data)  # noqa: S301 - trusted fake protocol payload.
        return _Message()

    def SendObservations(self, request_iterator: list[_Message], *, timeout: float) -> _Message:  # noqa: N802
        payload = b"".join(message.data for message in request_iterator)
        self.observations.append(pickle.loads(payload))  # noqa: S301 - trusted fake protocol payload.
        return _Message()

    def GetActions(self, _request: object, *, timeout: float) -> _Message:  # noqa: N802
        start = self.observations[-1].timestep
        actions = [
            _TimedAction(values, timestep=start + index)
            for index, values in enumerate(self.action_values)
        ]
        return _Message(pickle.dumps(actions))


class _FakeGrpc:
    def __init__(self) -> None:
        self.channel = _FakeChannel()

    def insecure_channel(self, _server_address: str) -> _FakeChannel:
        return self.channel


@dataclass
class _FakeLeRobot:
    grpc: _FakeGrpc
    stub: _FakeStub | None = None


@pytest.fixture
def fake_lerobot(monkeypatch: pytest.MonkeyPatch) -> _FakeLeRobot:
    fake = _FakeLeRobot(grpc=_FakeGrpc())

    class _AsyncInferenceStub:
        def __new__(cls, channel: _FakeChannel) -> _FakeStub:
            fake.stub = _FakeStub(channel)
            return fake.stub

    def send_bytes_in_chunks(
        data: bytes, message_cls: type[_Message], *, silent: bool
    ) -> list[_Message]:
        return [message_cls(data=data)]

    monkeypatch.setattr(client_module, "grpc", fake.grpc)
    monkeypatch.setattr(client_module, "RemotePolicyConfig", _RemotePolicyConfig)
    monkeypatch.setattr(client_module, "TimedObservation", _TimedObservation)
    monkeypatch.setattr(
        client_module,
        "services_pb2",
        SimpleNamespace(Empty=_Message, PolicySetup=_Message, Observation=_Message),
    )
    monkeypatch.setattr(
        client_module,
        "services_pb2_grpc",
        SimpleNamespace(AsyncInferenceStub=_AsyncInferenceStub),
    )
    monkeypatch.setattr(client_module, "send_bytes_in_chunks", send_bytes_in_chunks)
    return fake


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


def _schema(mg: MagicMock, image_source: object | None = None) -> PolicySchema:
    return PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.io("gripper", source=mg, io="digital_out[0]", mapping=BoolMapping()),
            Observation.image("cam_scene_1", source=image_source or MagicMock()),
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


@pytest.mark.parametrize("playback_speed", [0.0, -0.5])
def test_playback_speed_must_be_positive(playback_speed: float) -> None:
    with pytest.raises(ValueError, match="playback_speed must be positive"):
        LeRobotPolicyClient(
            "127.0.0.1:8080",
            "model",
            fps=15,
            playback_speed=playback_speed,
            actions_per_chunk=11,
        )


@pytest.mark.parametrize("threshold", [0.0, -0.1, 1.1])
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
        "127.0.0.1:8080",
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

    assert fake_lerobot.stub is not None
    assert fake_lerobot.stub.ready_calls == 1

    setup = fake_lerobot.stub.policy_setup
    assert setup == _RemotePolicyConfig(
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


@pytest.mark.parametrize(
    ("aggregation", "expected"),
    [
        (AsyncQueueAggregation.WEIGHTED_AVERAGE, 7.6),
        (AsyncQueueAggregation.LATEST_ONLY, 10.0),
        (AsyncQueueAggregation.AVERAGE, 6.0),
        (AsyncQueueAggregation.CONSERVATIVE, 4.4),
    ],
)
def test_async_queue_aggregation_modes(
    aggregation: object,
    expected: float,
) -> None:
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        actions_per_chunk=4,
        use_async_queue=True,
        async_queue_aggregation=aggregation,
    )
    client._latest_action_timestep = 0
    client._action_queue = [(4, np.asarray([2.0], dtype=np.float32))]

    assert client._merge_timed_actions([_TimedAction([10.0], timestep=4)])
    assert client._action_queue[0][1][0] == pytest.approx(expected)


@pytest.mark.asyncio
async def test_async_queue_refills_and_blends_overlapping_timesteps(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _schema(mg)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        fps=15,
        actions_per_chunk=4,
        use_async_queue=True,
    )

    client.enable_trajectory_trace()
    await client.connect([mg.id])
    assert fake_lerobot.stub is not None
    fake_lerobot.stub.action_values = [
        [float(index)] * 6 + [float(index % 2)] for index in range(4)
    ]

    first = await client.get_actions(
        {mg.id: _state((0.0,) * 6)},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": False},
    )
    assert [step[0] for step in first.joints[mg.id]] == [0.0, 1.0, 2.0, 3.0]
    assert first.action_timestep == 0
    assert fake_lerobot.stub.observations[-1].timestep == 0
    assert fake_lerobot.stub.observations[-1].must_go is True

    second = await client.get_actions(
        {mg.id: _state((1.0,) * 6)},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": False},
    )

    # Refill runs asynchronously while NOVA executes its published lookahead.
    # Waiting here would age the controller timestep selected before inference.
    assert second.joints == {}
    assert second.ios == {mg.id: {"digital_out[0]": True}}

    while client._pending_actions_task is not None and not client._pending_actions_task.done():
        await asyncio.sleep(0)
    assert fake_lerobot.stub.observations[-1].timestep == 1
    assert fake_lerobot.stub.observations[-1].must_go is True
    third = await client.get_actions(
        {mg.id: _state((2.0,) * 6)},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": False},
    )

    # The next tick publishes the completed refill. Published timestep 1 is
    # prepended, selected timestep 2 and successor timestep 3 stay frozen, and
    # timestep 4 extends the queue. IO comes from selected timestep 2.
    assert third.action_timestep == 1
    assert [step[0] for step in third.joints[mg.id]] == pytest.approx([1.0, 2.0, 3.0, 3.0])
    assert third.ios == {mg.id: {"digital_out[0]": False}}
    assert [timestep for timestep, _action in client._action_queue] == [3, 4]
    assert client._action_queue[0][1][0] == pytest.approx(3.0)
    assert client._action_queue[1][1][0] == pytest.approx(3.0)
    raw_chunks = client.trajectory_trace["raw_action_chunks"]
    assert len(raw_chunks) == 2
    assert raw_chunks[0]["first_timestep"] == 0
    assert [action["timestep"] for action in raw_chunks[0]["actions"]] == [0, 1, 2, 3]

    await client.close()


@pytest.mark.asyncio
async def test_async_queue_applies_action_chunk_smoothing_after_aggregation(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _schema(mg)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        actions_per_chunk=3,
        use_async_queue=True,
        async_queue_smoothing=True,
    )
    client.enable_trajectory_trace()
    await client.connect([mg.id])
    assert fake_lerobot.stub is not None
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
    assert [action[0] for action in client._published_actions.values()] == pytest.approx(
        [1.25, 1.5, 1.25]
    )
    assert [action[-1] for action in client._published_actions.values()] == [1.0, 0.0, 1.0]
    assert chunk.joints[mg.id] == [
        action[:6].tolist() for action in client._published_actions.values()
    ]
    assert client.trajectory_trace["raw_action_chunks"][0]["actions"][1]["values"][0] == 4.0

    await client.close()


def test_async_queue_freezes_current_and_two_successors() -> None:
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        actions_per_chunk=5,
        use_async_queue=True,
    )
    client._latest_action_timestep = 0
    client._action_queue = [
        (timestep, np.asarray([float(timestep)], dtype=np.float32)) for timestep in range(1, 5)
    ]
    client._published_actions = {
        timestep: np.asarray([float(timestep + 10)], dtype=np.float32) for timestep in range(1, 4)
    }

    assert client._merge_timed_actions(
        [_TimedAction([10.0], timestep=timestep) for timestep in range(1, 5)]
    )

    assert [action[0] for _timestep, action in client._action_queue] == pytest.approx(
        [11.0, 12.0, 13.0, 8.2]
    )


def test_average_aggregation_is_a_true_running_mean_per_timestep() -> None:
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        actions_per_chunk=4,
        use_async_queue=True,
        async_queue_aggregation=AsyncQueueAggregation.AVERAGE,
    )
    client._latest_action_timestep = 0
    client._action_queue = [(4, np.asarray([2.0], dtype=np.float32))]

    assert client._merge_timed_actions([_TimedAction([10.0], timestep=4)])
    assert client._merge_timed_actions([_TimedAction([12.0], timestep=4)])

    assert client._action_queue[0][1][0] == pytest.approx((2.0 + 10.0 + 12.0) / 3.0)
    assert client._action_prediction_counts == {4: 3}


def test_async_queue_synchronization_drops_actions_elapsed_on_nova() -> None:
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        actions_per_chunk=4,
        use_async_queue=True,
    )
    client._latest_action_timestep = 1
    client._action_queue = [
        (2, np.asarray([2.0], dtype=np.float32)),
        (3, np.asarray([3.0], dtype=np.float32)),
        (4, np.asarray([4.0], dtype=np.float32)),
    ]

    client.synchronize_action_timestep(4)

    assert client._latest_action_timestep == 3
    assert [timestep for timestep, _action in client._action_queue] == [4]


@pytest.mark.asyncio
async def test_async_queue_keeps_sending_observations_while_refill_is_pending(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        fps=15,
        actions_per_chunk=4,
        use_async_queue=True,
    )
    await client.connect([mg.id])

    action_1 = np.asarray([1.0] * 7, dtype=np.float32)
    action_2 = np.asarray([2.0] * 7, dtype=np.float32)
    client._action_chunk_size = 4
    client._action_queue = [(1, action_1), (2, action_2)]
    client._pending_actions_task = asyncio.create_task(asyncio.sleep(10, result=[]))

    chunk = await client._get_async_queue_actions(
        {"arm_1": 1.0},
        [(mg.id, slice(0, 6))],
        [],
    )

    assert chunk.joints == {}
    assert fake_lerobot.stub is not None
    assert fake_lerobot.stub.observations[-1].timestep == 1
    assert fake_lerobot.stub.observations[-1].must_go is False

    await client.close()


@pytest.mark.asyncio
async def test_prepare_sends_policy_instructions_before_first_inference(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    schema = _schema(mg)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", fps=15, actions_per_chunk=8)

    await client.prepare(
        {mg.id: _state((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))},
        schema,
        images={"cam_scene_1": image},
        io_values={"digital_out[0]": True},
    )

    assert fake_lerobot.stub is not None
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


@pytest.mark.asyncio
async def test_get_actions_applies_state_overrides_to_sent_observation(
    fake_lerobot: _FakeLeRobot,
) -> None:
    mg = _mg()
    client = LeRobotPolicyClient(
        "127.0.0.1:8080",
        "model",
        fps=15,
        actions_per_chunk=8,
        state_overrides={f"arm_{idx}": 0.0 for idx in range(1, 7)},
    )

    await client.get_actions(
        {mg.id: _state((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))},
        _schema(mg),
        images={"cam_scene_1": np.zeros((240, 320, 3), dtype=np.uint8)},
        io_values={"digital_out[0]": False},
    )

    assert fake_lerobot.stub is not None
    observation = fake_lerobot.stub.observations[0].observation
    assert [observation[f"arm_{idx}"] for idx in range(1, 7)] == [0.0] * 6
    assert observation["gripper"] == 0.0


@pytest.mark.asyncio
async def test_get_actions_requires_actual_image_frame_for_feature_metadata(
    fake_lerobot: _FakeLeRobot,
) -> None:
    assert fake_lerobot.stub is None
    mg = _mg()
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", fps=15, actions_per_chunk=8)

    with pytest.raises(ValueError, match="needs the first camera frame"):
        await client.get_actions(
            {mg.id: _state((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))},
            _schema(mg),
            images=None,
            io_values={"digital_out[0]": False},
        )


@pytest.mark.asyncio
async def test_validate_schema_rejects_schema_without_joint_action(
    fake_lerobot: _FakeLeRobot,
) -> None:
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", fps=15, actions_per_chunk=8)
    schema = PolicySchema(observations=[Observation.image("cam_scene_1", source=MagicMock())])

    with pytest.raises(ValueError, match="requires at least one joint action"):
        await client.validate_schema(schema)


@pytest.mark.asyncio
async def test_close_closes_protocol_channel(fake_lerobot: _FakeLeRobot) -> None:
    client = LeRobotPolicyClient("127.0.0.1:8080", "model", fps=15, actions_per_chunk=8)

    await client.connect([])
    await client.close()

    assert fake_lerobot.grpc.channel.closed is True
