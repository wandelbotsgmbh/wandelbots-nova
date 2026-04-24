from unittest.mock import AsyncMock, MagicMock

from nova.cell.motion_group import MotionGroup
import pytest

from nova_policy import ACTPolicy, enable_motion_group_policy_extension
from nova_policy.models import PolicyRun


class FakePolicyClient:
    def __init__(self):
        self.started_payload = None
        self.stop_calls: list[tuple[str, str | None]] = []

    async def start_run(self, policy: str, payload: dict[str, object]):
        self.started_payload = (policy, payload)
        return PolicyRun(run="run_1", policy=policy, state="PREPARING", timeout_s=120.0)

    async def stop_run(self, policy: str, run: str | None = None):
        self.stop_calls.append((policy, run))

    async def stream_run(self, policy: str, run: str):
        yield PolicyRun(run=run, policy=policy, state="RUNNING", elapsed_s=1.0)
        yield PolicyRun(run=run, policy=policy, state="TIMED_OUT", elapsed_s=120.0)


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
