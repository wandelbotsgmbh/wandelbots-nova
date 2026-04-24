from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from nova_policy import ACTPolicy, PolicyServiceClient, RobotStatePoint


@pytest.mark.asyncio
async def test_get_policy_returns_typed_act_policy(monkeypatch):
    client = PolicyServiceClient(base_url="http://policy-service")
    fake_request = AsyncMock(return_value={"kind": "act", "path": "org/act-policy"})
    monkeypatch.setattr(client, "_request_json", fake_request)

    policy = await client.get_policy()

    assert isinstance(policy, ACTPolicy)
    assert policy.path == "org/act-policy"
    fake_request.assert_awaited_once()


@pytest.mark.asyncio
async def test_get_policy_rejects_unknown_kind(monkeypatch):
    client = PolicyServiceClient(base_url="http://policy-service")
    fake_request = AsyncMock(return_value={"kind": "pi0", "path": "org/pi0-policy"})
    monkeypatch.setattr(client, "_request_json", fake_request)

    with pytest.raises(ValueError, match="Unsupported policy kind"):
        await client.get_policy()


def test_parse_action_chunk_payload_returns_typed_chunk():
    chunk = PolicyServiceClient._parse_action_chunk_payload(
        {
            "run": "run_1",
            "policy": "org/policy",
            "policy_kind": "act",
            "chunk_id": "chunk_1",
            "observation_seq": 7,
            "n_action_steps": 2,
            "control_dt_s": 0.1,
            "inference_latency_ms": 12.5,
            "steps": [
                {
                    "joints": {"joint_1.pos": 1.0, "joint_2.pos": 2.0},
                    "gripper": {"gripper.pos": 0.0},
                },
                {
                    "joints": {"joint_1.pos": 3.0, "joint_2.pos": 4.0},
                },
            ],
        }
    )

    assert chunk.run == "run_1"
    assert chunk.observation_seq == 7
    assert chunk.steps[0].joints["joint_1.pos"] == 1.0
    assert chunk.steps[0].gripper == {"gripper.pos": 0.0}


@pytest.mark.asyncio
async def test_realtime_session_predict_uses_socketio(monkeypatch):
    class FakeAsyncClient:
        def __init__(self):
            self.connected = False
            self.handlers: dict[str, object] = {}

        def on(self, event: str):
            def decorator(handler):
                self.handlers[event] = handler
                return handler

            return decorator

        async def connect(self, *_args, **_kwargs):
            self.connected = True

        async def disconnect(self):
            self.connected = False

        async def call(self, event: str, payload: dict[str, object], timeout: float):
            assert event == "observation.push"
            assert payload["run"] == "run_1"
            assert payload["seq"] == 3
            assert timeout == 1.5
            action_handler = self.handlers["action.chunk"]
            action_handler(
                {
                    "run": "run_1",
                    "policy": "org/policy",
                    "policy_kind": "act",
                    "chunk_id": "chunk_1",
                    "observation_seq": 3,
                    "n_action_steps": 1,
                    "control_dt_s": 0.05,
                    "inference_latency_ms": 9.1,
                    "steps": [{"joints": {"joint_1.pos": 1.23}}],
                }
            )
            return {"accepted": True}

    fake_client = FakeAsyncClient()
    monkeypatch.setattr(
        "nova_policy.client._load_socketio_module",
        lambda: SimpleNamespace(AsyncClient=lambda reconnection=True: fake_client),
    )

    client = PolicyServiceClient(base_url="http://policy-service", timeout_s=1.5)
    session = client.open_realtime_session()
    chunk = await session.predict(
        run="run_1",
        seq=3,
        state_point=RobotStatePoint(joints={"joint_1.pos": 0.5}),
    )

    assert chunk.chunk_id == "chunk_1"
    assert chunk.steps[0].joints == {"joint_1.pos": 1.23}
