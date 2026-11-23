"""Tests for CallbackPolicyClient action-chunk handling."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest

from novapolicy.policy_client import CallbackPolicyClient
from novapolicy.schema import Observation, PolicySchema
from novapolicy.tests import doubles
from novapolicy.types import ActionChunk


@pytest.mark.asyncio
async def test_returns_action_chunk_unchanged():
    """The callback returns an ActionChunk, which is passed through as-is."""
    mg = doubles.mg()
    schema = PolicySchema(observations=[Observation.joint_positions("joints", source=mg)])
    expected = ActionChunk(joints={"0@ur10e": [[0.5, -1.0], [0.6, -0.9]]}, dt_ms=50.0)

    async def fn(_obs: dict) -> ActionChunk:
        return expected

    client = CallbackPolicyClient(fn)
    chunk = await client.get_actions(states={}, schema=schema)

    assert chunk is expected


@pytest.mark.asyncio
async def test_callback_client_uses_explicit_no_op_capabilities() -> None:
    async def fn(_obs: dict) -> ActionChunk:
        return ActionChunk()

    client = CallbackPolicyClient(fn)
    await client.connect([])
    await client.validate_schema(PolicySchema(observations=[]))
    await client.prepare({}, PolicySchema(observations=[]))
    client.synchronize_action_timestep(4)
    await client.close()

    assert client.default_modules == []


@pytest.mark.asyncio
async def test_a_callback_control_freq_fills_chunks_without_step_spacing() -> None:
    from types import SimpleNamespace

    async def policy(_obs: object) -> ActionChunk:
        return ActionChunk(joints={"mg": [[0.0]]})

    schema = SimpleNamespace(build_observation=AsyncMock(return_value={}))
    chunk = await CallbackPolicyClient(policy, control_freq=20.0).get_actions({}, schema)  # type: ignore[arg-type]
    assert chunk.dt_ms == pytest.approx(50.0)
    chunk = await CallbackPolicyClient(policy).get_actions({}, schema)  # type: ignore[arg-type]
    assert chunk.dt_ms == 0.0  # a single step stays one


class _EchoWire:
    """A provider's wire, faked: answers every observation with a two-step plan for 'arm'."""

    def __init__(self) -> None:
        self.opened = self.closed = False
        self.seen: dict[str, object] = {}

    async def open(self) -> None:
        self.opened = True

    async def infer(self, observation: dict[str, list[float]], images: object) -> dict[str, object]:
        self.seen = observation
        first = [v + 0.1 for v in observation["arm"]]
        return {"arm": [first, [v + 0.1 for v in first]]}

    async def close(self) -> None:
        self.closed = True


@pytest.mark.asyncio
async def test_a_provider_is_a_transport_and_the_schema_does_the_mapping() -> None:
    """The provider from docs/architecture.md, as written there, on a fake wire."""
    from novapolicy import PolicyClient, resolve_control_freq

    class MyPolicyClient(PolicyClient):
        def __init__(self, wire: _EchoWire, *, control_freq: float | None = None) -> None:
            self._wire = wire  # the provider's own protocol
            self._dt_ms = 1000.0 / resolve_control_freq(control_freq, owner="MyPolicyClient")

        async def connect(self, motion_group_ids: list[str]) -> None:
            await self._wire.open()

        async def get_actions(self, states, schema, images=None, io_values=None):
            observation = await schema.build_observation(states, io_values)
            groups = schema.observation_groups(observation)  # {"arm": [6 values], ...}
            outputs = await self._wire.infer(groups, images)  # {"arm": [[6 values], ...]}
            return schema.decode_actions(outputs, dt_ms=self._dt_ms, source="MyPolicy")

        async def close(self) -> None:
            await self._wire.close()

    mg = MagicMock()
    mg.id = "0@arm"
    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=mg, action=True)])
    wire = _EchoWire()
    client = MyPolicyClient(wire, control_freq=20)
    await client.connect([mg.id])
    chunk = await client.get_actions({mg.id: SimpleNamespace(joints=(0.0,) * 6)}, schema)
    await client.close()
    assert wire.opened and wire.closed
    assert chunk.dt_ms == 50.0
    assert chunk.joints[mg.id] == [[0.1] * 6, [0.2] * 6]


def test_a_flat_wire_round_trips_through_flatten_and_unflatten() -> None:
    """One state vector out, one flat row per step back: joints, then IO, in schema order."""
    mg = MagicMock()
    mg.id = "0@arm"
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg, action=True),
            Observation.io("gripper", source=mg, io="digital_out[0]", action=True),
        ]
    )
    groups = {"arm": [float(i) for i in range(6)], "gripper": [1.0]}
    assert schema.flatten(groups) == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 1.0]
    outputs = schema.unflatten([[0.1] * 6 + [0.0], [0.2] * 6 + [1.0]], dof={mg.id: 6})
    assert outputs == {"arm": [[0.1] * 6, [0.2] * 6], "gripper": [[0.0], [1.0]]}
    with pytest.raises(ValueError, match="expected 7"):
        schema.unflatten([[0.1] * 6], dof={mg.id: 6})


def test_decoding_takes_each_groups_joints_and_refuses_too_few() -> None:
    left, right = MagicMock(), MagicMock()
    left.id, right.id = "1@l", "2@r"
    schema = PolicySchema(
        observations=[Observation.joint_positions("arms", source=[left, right], action=True)]
    )
    dof = {"1@l": 2, "2@r": 3}
    chunk = schema.decode_actions({"arms": [[1, 2, 3, 4, 5, 6]]}, dt_ms=10.0, dof=dof)
    assert chunk.joints == {"1@l": [[1.0, 2.0]], "2@r": [[3.0, 4.0, 5.0]]}  # an extra column left
    with pytest.raises(ValueError, match="do not split"):
        schema.decode_actions({"arms": [[1, 2, 3, 4]]}, dt_ms=10.0, dof=dof)


def _http_example() -> Any:
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[2] / "examples" / "http_policy.py"
    spec = importlib.util.spec_from_file_location("http_policy", path)
    assert spec is not None and spec.loader is not None
    example = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(example)
    return example


@pytest.mark.asyncio
async def test_the_http_example_provider_round_trips_through_its_mock_server() -> None:
    """examples/http_policy.py: observation out as JSON, a decoded chunk back."""
    example = _http_example()

    mg = MagicMock()
    mg.id = "0@arm"
    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=mg, action=True)])
    server = example.serve_mock_policy([0.0] * 6)
    host, port = server.server_address[:2]
    client = example.HttpPolicyClient(f"http://{host}:{port}/act", control_freq=30)
    try:
        await client.connect([mg.id])
        chunk = await client.get_actions({mg.id: SimpleNamespace(joints=(0.0,) * 6)}, schema)
    finally:
        await client.close()
        server.shutdown()
    assert chunk.dt_ms == pytest.approx(1000 / 30)
    assert len(chunk.joints[mg.id]) == 15 and len(chunk.joints[mg.id][0]) == 6
