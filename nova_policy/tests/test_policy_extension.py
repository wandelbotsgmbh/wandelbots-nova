from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

from nova.cell.motion_group import MotionGroup
import pytest

from nova_policy import ACTAdapter, ACTPolicy, PolicyExecutionOptions, enable_motion_group_policy_extension
from nova_policy.models import ActionChunk, ActionStep, PolicyRun


class FakeRealtimeSession:
    def __init__(self):
        self.predict_calls: list[dict[str, object]] = []
        self.closed = False

    async def predict(self, *, run, seq, state_point, task=None, timeout_s=None):
        self.predict_calls.append(
            {
                "run": run,
                "seq": seq,
                "observation": state_point.to_observation(),
                "task": task,
                "timeout_s": timeout_s,
            }
        )
        return ActionChunk(
            run=run,
            policy="org/policy",
            policy_kind="act",
            chunk_id=f"chunk_{seq}",
            observation_seq=seq,
            n_action_steps=1,
            control_dt_s=0.01,
            inference_latency_ms=1.0,
            steps=[ActionStep(joints={"joint_1.pos": 1.0})],
        )

    async def close(self):
        self.closed = True


class FakePolicyClient:
    def __init__(self):
        self.started_payload = None
        self.stop_calls: list[tuple[str, str | None]] = []
        self.realtime_session = FakeRealtimeSession()
        self.status_poll_interval_s = 0.0

    async def start_run(self, policy: str, payload: dict[str, object]):
        self.started_payload = (policy, payload)
        return PolicyRun(run="run_1", policy=policy, state="PREPARING", timeout_s=120.0)

    async def stop_run(self, policy: str, run: str | None = None):
        self.stop_calls.append((policy, run))

    async def get_run(self, policy: str, run: str):
        if self.stop_calls:
            return PolicyRun(run=run, policy=policy, state="STOPPED", elapsed_s=1.0)
        return PolicyRun(
            run=run,
            policy=policy,
            state="RUNNING",
            elapsed_s=1.0,
            metadata={"control_dt_s": 0.0},
        )

    async def stream_run(self, policy: str, run: str):
        yield PolicyRun(run=run, policy=policy, state="RUNNING", elapsed_s=1.0)
        yield PolicyRun(run=run, policy=policy, state="TIMED_OUT", elapsed_s=120.0)

    def open_realtime_session(self):
        return self.realtime_session


@pytest.fixture
def mock_motion_group():
    enable_motion_group_policy_extension()
    mock_api_client = MagicMock()
    mock_api_client.config = MagicMock(access_token="token")

    motion_group = MotionGroup(
        api_client=mock_api_client,
        cell="cell",
        controller_id="controller",
        motion_group_id="0",
    )
    motion_group.active_tcp_name = AsyncMock(return_value="flange")
    mock_setup = MagicMock()
    mock_setup.model_dump.return_value = {"motion_group_model": "test-model"}
    motion_group.get_setup = AsyncMock(return_value=mock_setup)
    return motion_group


def test_act_adapter_builds_service_policy_payload():
    payload = ACTAdapter(ACTPolicy(path="org/policy", n_action_steps=4)).service_policy_payload(
        device="cuda"
    )

    assert payload == {
        "kind": "act",
        "path": "org/policy",
        "n_action_steps": 4,
        "device": "cuda",
    }


@pytest.mark.asyncio
async def test_execute_policy_returns_terminal_state(mock_motion_group, monkeypatch):
    fake_client = FakePolicyClient()
    monkeypatch.setattr(
        "nova_policy.motion_group_extensions._resolve_policy_client",
        lambda *_: fake_client,
    )

    result = await mock_motion_group.execute_policy(
        policy_path="org/policy",
        task="pick",
        timeout_s=120.0,
    )

    assert result.state == "TIMED_OUT"
    assert fake_client.started_payload is not None


@pytest.mark.asyncio
async def test_stream_policy_stop_calls_api(mock_motion_group, monkeypatch):
    fake_client = FakePolicyClient()
    monkeypatch.setattr(
        "nova_policy.motion_group_extensions._resolve_policy_client",
        lambda *_: fake_client,
    )

    async for state in mock_motion_group.stream_policy(
        policy_path="org/policy",
        task="pick",
        timeout_s=120.0,
    ):
        await state.stop()
        break

    assert fake_client.stop_calls == [("org/policy", "run_1")]


@pytest.mark.asyncio
async def test_stream_policy_accepts_typed_policy(mock_motion_group, monkeypatch):
    fake_client = FakePolicyClient()
    monkeypatch.setattr(
        "nova_policy.motion_group_extensions._resolve_policy_client",
        lambda *_: fake_client,
    )

    async for _state in mock_motion_group.stream_policy(
        policy=ACTPolicy(path="org/typed-policy", n_action_steps=16),
        task="pick",
        timeout_s=120.0,
    ):
        break

    assert fake_client.started_payload is not None
    started_policy, started_payload = fake_client.started_payload
    assert started_policy == "org/typed-policy"
    policy_payload = started_payload["policy"]
    assert isinstance(policy_payload, dict)
    assert policy_payload["kind"] == "act"
    assert policy_payload["n_action_steps"] == 16


@pytest.mark.asyncio
async def test_stream_policy_realtime_pushes_robot_state(mock_motion_group, monkeypatch):
    fake_client = FakePolicyClient()
    monkeypatch.setattr(
        "nova_policy.motion_group_extensions._resolve_policy_client",
        lambda *_: fake_client,
    )
    mock_motion_group.get_state = AsyncMock(
        return_value=SimpleNamespace(joints=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    )

    seen_states = []
    async for state in mock_motion_group.stream_policy(
        policy_path="org/policy",
        task="pick",
        timeout_s=120.0,
        options=PolicyExecutionOptions(
            realtime=True,
            max_observations=2,
            allow_mock_images=True,
            use_gripper=True,
        ),
    ):
        seen_states.append(state.state)

    assert seen_states == ["PREPARING", "RUNNING", "RUNNING", "STOPPED"]
    assert fake_client.stop_calls == [("org/policy", "run_1")]
    assert fake_client.realtime_session.closed is True
    assert [call["seq"] for call in fake_client.realtime_session.predict_calls] == [0, 1]
    assert fake_client.realtime_session.predict_calls[0]["observation"] == {
        "joint_1.pos": 0.1,
        "joint_2.pos": 0.2,
        "joint_3.pos": 0.3,
        "joint_4.pos": 0.4,
        "joint_5.pos": 0.5,
        "joint_6.pos": 0.6,
        "gripper.pos": 0.0,
    }
    assert fake_client.started_payload is not None
    _, payload = fake_client.started_payload
    assert payload["allow_mock_images"] is True
