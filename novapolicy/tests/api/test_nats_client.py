"""Tests for ``NatsPolicyClient`` — the observation/action translation layer.

No broker: the client's transport is one method (``_request``), so stubbing it
exercises everything that actually decides what the robot is told to do — proprio
assembly, DOF guards, block binding, and chunk conversion.
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from novapolicy.nats.client import ACTION_KEY, PROPRIO_KEY, NatsPolicyClient
from novapolicy.nats.protocol import (
    ActionSpec,
    FieldSpec,
    PolicyInfo,
    PolicyProtocolError,
    PredictResponse,
    decode,
)
from novapolicy.schema import Observation, PolicySchema

DOF = 6
MG_ID = "0@ur10e"
LEFT_ID, RIGHT_ID = "1@dualarm", "2@dualarm"
HOME = [0.0, -1.571, 1.571, 0.0, 1.42, 3.142]


def _mg(mg_id: str) -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.split("@")[1]
    mg._cell = "cell"
    return mg


class _FakeState:
    """Stands in for a Nova ``RobotState``; the client only reads ``joints``."""

    def __init__(self, joints: tuple[float, ...]) -> None:
        self.joints = joints


def _info(*, dof: int = DOF, horizon: int = 1) -> PolicyInfo:
    """Single-block ('legacy') policy info."""
    return PolicyInfo(
        policy_id="dummy",
        name="Dummy",
        version="0.1.0",
        embodiment="generic-6dof",
        observations=[FieldSpec(name=PROPRIO_KEY, shape=[3 * DOF])],
        actions=[ActionSpec(name=ACTION_KEY, dof=dof, horizon=horizon, dt_ms=20.0)],
    )


def _dual_info(dof: int = 7) -> PolicyInfo:
    """Named-block policy info: one block per arm, as a dual-arm policy declares."""
    return PolicyInfo(
        policy_id="dual",
        name="Dual",
        observations=[
            FieldSpec(name=f"{PROPRIO_KEY}.left", shape=[3 * dof]),
            FieldSpec(name=f"{PROPRIO_KEY}.right", shape=[3 * dof]),
        ],
        actions=[
            ActionSpec(name=f"{ACTION_KEY}.left", dof=dof, horizon=1, dt_ms=20.0),
            ActionSpec(name=f"{ACTION_KEY}.right", dof=dof, horizon=1, dt_ms=20.0),
        ],
    )


def _client(info: PolicyInfo | None = None, motion_groups: list[str] | None = None):
    """A client wired up as if ``connect()`` had already succeeded."""
    client = NatsPolicyClient(policy_id="dummy", servers="nats://localhost:4222")
    client._info = info if info is not None else _info()
    client._bind_blocks(motion_groups or [MG_ID])
    client._session_id = "session-1"
    client._nc = object()  # non-None so _request is the only thing left to stub
    return client


def _reply(targets: np.ndarray, *, name: str = ACTION_KEY, dt_ms: float = 20.0) -> dict[str, Any]:
    return PredictResponse(session_id="session-1", actions={name: targets}, dt_ms=dt_ms).to_wire()


def _stub_request(client: NatsPolicyClient, body: dict[str, Any]) -> list[dict[str, Any]]:
    """Replace the transport; return the list that captures outgoing observations."""
    sent: list[dict[str, Any]] = []

    async def fake_request(subject: str, payload: bytes, *, expect: str) -> dict[str, Any]:
        sent.append(decode(payload, expect=expect))
        return body

    client._request = fake_request  # ty: ignore[invalid-assignment]
    return sent


def _schema(*mgs: MagicMock) -> PolicySchema:
    """A schema whose inferred joint actions bind one motion group each."""
    return PolicySchema(
        observations=[Observation.joint_positions(f"arm{i}", source=mg) for i, mg in enumerate(mgs)]
    )


def _hold_chunk(dof: int = DOF, steps: int = 1) -> np.ndarray:
    return np.tile(HOME[:dof], (steps, 1)).astype(np.float32)


# -- proprio assembly ------------------------------------------------------


@pytest.mark.asyncio
async def test_first_observation_reports_zero_velocity_and_hold_as_last_action() -> None:
    """Step 0 has no previous sample, so velocity must be zero and `last_action`
    must read as "stay where you are" — not zeros, which would look like a command
    to fold the arm to its origin."""
    client = _client()
    sent = _stub_request(client, _reply(_hold_chunk()))

    await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))

    proprio = sent[0]["observation"][PROPRIO_KEY]
    assert proprio.shape == (3 * DOF,)
    np.testing.assert_allclose(proprio[:DOF], HOME, atol=1e-5)
    np.testing.assert_allclose(proprio[DOF : 2 * DOF], np.zeros(DOF))
    np.testing.assert_allclose(proprio[2 * DOF :], HOME, atol=1e-5)


@pytest.mark.asyncio
async def test_velocity_is_finite_differenced_between_steps(monkeypatch) -> None:
    ticks = [100.0, 100.5]  # 0.5 s apart; the last value repeats if read again
    monkeypatch.setattr(
        "novapolicy.nats.client.time.monotonic",
        lambda: ticks.pop(0) if len(ticks) > 1 else ticks[0],
    )

    client = _client()
    schema = _schema(_mg(MG_ID))
    moved = [q + 0.25 for q in HOME]
    sent = _stub_request(client, _reply(_hold_chunk()))

    await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, schema)
    await client.get_actions({MG_ID: _FakeState(tuple(moved))}, schema)

    velocity = sent[1]["observation"][PROPRIO_KEY][DOF : 2 * DOF]
    np.testing.assert_allclose(velocity, np.full(DOF, 0.5), atol=1e-4)  # 0.25 rad / 0.5 s


@pytest.mark.asyncio
async def test_simultaneous_samples_report_zero_velocity_not_a_spike(monkeypatch) -> None:
    """A zero dt must not divide observation noise into a huge velocity."""
    monkeypatch.setattr("novapolicy.nats.client.time.monotonic", lambda: 100.0)

    client = _client()
    schema = _schema(_mg(MG_ID))
    sent = _stub_request(client, _reply(_hold_chunk()))

    await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, schema)
    await client.get_actions({MG_ID: _FakeState(tuple(q + 0.1 for q in HOME))}, schema)

    velocity = sent[1]["observation"][PROPRIO_KEY][DOF : 2 * DOF]
    np.testing.assert_allclose(velocity, np.zeros(DOF))


@pytest.mark.asyncio
async def test_last_action_reports_what_was_actually_commanded() -> None:
    """The next observation's `last_action` block must be the final target of the
    previous chunk, so the policy sees its own effect rather than a guess."""
    client = _client()
    schema = _schema(_mg(MG_ID))
    chunk = _hold_chunk(steps=4)
    chunk[-1] = np.array(HOME) + 0.05
    sent = _stub_request(client, _reply(chunk))

    await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, schema)
    await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, schema)

    np.testing.assert_allclose(sent[1]["observation"][PROPRIO_KEY][2 * DOF :], chunk[-1], atol=1e-5)


@pytest.mark.asyncio
async def test_step_counter_and_elapsed_time_advance() -> None:
    client = _client()
    schema = _schema(_mg(MG_ID))
    sent = _stub_request(client, _reply(_hold_chunk()))

    for _ in range(3):
        await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, schema)

    assert [msg["step"] for msg in sent] == [0, 1, 2]
    assert all(msg["t_ms"] >= 0.0 for msg in sent)


# -- chunk conversion ------------------------------------------------------


@pytest.mark.asyncio
async def test_horizon_one_reply_becomes_a_single_step_chunk() -> None:
    """A horizon-1 policy predicts one target per call; it must reach the executor
    as a 1-step chunk carrying the policy's own control period, because that is
    what the session's braking-horizon padding is sized against."""
    client = _client()
    target = np.array([HOME], dtype=np.float32)
    _stub_request(client, _reply(target, dt_ms=20.0))

    chunk = await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))

    assert chunk.dt_ms == 20.0
    assert list(chunk.joints) == [MG_ID]
    assert len(chunk.joints[MG_ID]) == 1
    np.testing.assert_allclose(chunk.joints[MG_ID][0], HOME, atol=1e-5)


@pytest.mark.asyncio
async def test_a_reply_without_dt_falls_back_to_the_declared_control_period() -> None:
    client = _client()
    _stub_request(client, _reply(_hold_chunk(), dt_ms=0.0))

    chunk = await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))

    assert chunk.dt_ms == 20.0  # ActionSpec.dt_ms


# -- named-block (dual-arm) binding ---------------------------------------


def test_named_blocks_bind_motion_groups_in_declared_order() -> None:
    """The declared order *is* the binding contract: the i-th declared block
    drives the i-th motion group the executor passes in."""
    client = _client(_dual_info(), [LEFT_ID, RIGHT_ID])

    assert client.bound_blocks == [
        (LEFT_ID, client.info.actions[0]),
        (RIGHT_ID, client.info.actions[1]),
    ]


@pytest.mark.asyncio
async def test_named_blocks_key_each_arm_separately_in_both_directions() -> None:
    """Each arm gets its own proprio field on the way out and its own entry in
    the chunk on the way back; crossing them would drive the wrong arm."""
    dof = 7
    client = _client(_dual_info(dof), [LEFT_ID, RIGHT_ID])
    left_target = np.full((1, dof), 0.1, dtype=np.float32)
    right_target = np.full((1, dof), 0.2, dtype=np.float32)
    body = PredictResponse(
        session_id="session-1",
        actions={f"{ACTION_KEY}.left": left_target, f"{ACTION_KEY}.right": right_target},
        dt_ms=20.0,
    ).to_wire()
    sent = _stub_request(client, body)

    chunk = await client.get_actions(
        {LEFT_ID: _FakeState((0.0,) * dof), RIGHT_ID: _FakeState((1.0,) * dof)},
        _schema(_mg(LEFT_ID), _mg(RIGHT_ID)),
    )

    observation = sent[0]["observation"]
    assert set(observation) == {f"{PROPRIO_KEY}.left", f"{PROPRIO_KEY}.right"}
    np.testing.assert_allclose(observation[f"{PROPRIO_KEY}.left"][:dof], np.zeros(dof))
    np.testing.assert_allclose(observation[f"{PROPRIO_KEY}.right"][:dof], np.ones(dof))
    np.testing.assert_allclose(chunk.joints[LEFT_ID][0], left_target[0], atol=1e-6)
    np.testing.assert_allclose(chunk.joints[RIGHT_ID][0], right_target[0], atol=1e-6)


def test_a_named_block_without_a_matching_observation_is_rejected() -> None:
    info = _dual_info()
    info.observations = info.observations[:1]  # drop proprio.right

    with pytest.raises(PolicyProtocolError, match="no matching observation"):
        _client(info, [LEFT_ID, RIGHT_ID])


def test_action_heads_disagreeing_on_dt_are_rejected() -> None:
    info = _dual_info()
    info.actions[1].dt_ms = 50.0

    with pytest.raises(PolicyProtocolError, match="disagree on dt_ms"):
        _client(info, [LEFT_ID, RIGHT_ID])


# -- guards ----------------------------------------------------------------


@pytest.mark.asyncio
async def test_robot_dof_mismatch_fails_before_commanding_anything() -> None:
    client = _client()
    _stub_request(client, _reply(np.zeros((1, DOF), dtype=np.float32)))

    with pytest.raises(PolicyProtocolError, match="7 axes but policy") as exc:
        await client.get_actions({MG_ID: _FakeState((0.0,) * 7)}, _schema(_mg(MG_ID)))
    assert exc.value.code == "dof_mismatch"


@pytest.mark.asyncio
async def test_declared_proprio_width_is_enforced() -> None:
    """Catches a policy trained on a different proprio layout (e.g. no velocities)."""
    info = _info()
    info.observations = [FieldSpec(name=PROPRIO_KEY, kind="state", shape=[2 * DOF])]
    client = _client(info)
    _stub_request(client, _reply(np.zeros((1, DOF), dtype=np.float32)))

    with pytest.raises(PolicyProtocolError, match="width mismatch") as exc:
        await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))
    assert exc.value.code == "obs_mismatch"


@pytest.mark.asyncio
async def test_missing_state_for_the_bound_motion_group_is_rejected() -> None:
    client = _client()
    _stub_request(client, _reply(np.zeros((1, DOF), dtype=np.float32)))

    with pytest.raises(PolicyProtocolError, match="No robot state"):
        await client.get_actions({"0@other": _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))


@pytest.mark.asyncio
async def test_reply_missing_the_declared_action_is_rejected() -> None:
    client = _client()
    _stub_request(client, _reply(np.zeros((1, DOF), dtype=np.float32), name="wrong_key"))

    with pytest.raises(PolicyProtocolError, match="missing action 'joint_target'"):
        await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))


@pytest.mark.asyncio
async def test_reply_with_the_wrong_dof_per_step_is_rejected() -> None:
    client = _client()
    _stub_request(client, _reply(np.zeros((1, 4), dtype=np.float32)))

    with pytest.raises(PolicyProtocolError, match="4 values per step"):
        await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))


def test_binding_more_motion_groups_than_declared_blocks_is_rejected() -> None:
    client = NatsPolicyClient(policy_id="dummy", servers="nats://localhost:4222")
    client._info = _info()  # single-block policy

    with pytest.raises(ValueError, match="declares 1 motion-group block"):
        client._bind_blocks([MG_ID, "0@other"])


@pytest.mark.asyncio
async def test_no_subscriber_is_reported_as_such_not_as_a_raw_nats_error() -> None:
    """`NoRespondersError` is the common misconfiguration (service down, or a
    mismatched policy id / subject prefix) and NATS raises it *instantly* rather
    than timing out — so it needs its own branch, naming both things to check.
    """
    from nats.errors import NoRespondersError

    client = _client()

    async def no_responders(*_args, **_kwargs):
        raise NoRespondersError

    client._nc = type("NC", (), {"request": staticmethod(no_responders)})()

    with pytest.raises(PolicyProtocolError, match="Nothing is subscribed") as exc:
        await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))
    assert exc.value.code == "no_responders"
    assert "policy id" in str(exc.value)


@pytest.mark.asyncio
async def test_a_slow_but_present_policy_is_reported_as_a_timeout() -> None:
    client = _client()

    async def too_slow(*_args, **_kwargs):
        raise TimeoutError

    client._nc = type("NC", (), {"request": staticmethod(too_slow)})()

    with pytest.raises(PolicyProtocolError, match="did not answer in time") as exc:
        await client.get_actions({MG_ID: _FakeState(tuple(HOME))}, _schema(_mg(MG_ID)))
    assert exc.value.code == "timeout"


def test_info_before_connect_is_a_clear_error() -> None:
    client = NatsPolicyClient(policy_id="dummy", servers="nats://localhost:4222")

    with pytest.raises(RuntimeError, match="connect"):
        _ = client.info


# -- schema validation -----------------------------------------------------


@pytest.mark.asyncio
async def test_validate_schema_accepts_one_joint_target_per_block() -> None:
    client = _client(_dual_info(), [LEFT_ID, RIGHT_ID])

    await client.validate_schema(_schema(_mg(LEFT_ID), _mg(RIGHT_ID)))


@pytest.mark.asyncio
async def test_validate_schema_rejects_the_wrong_number_of_joint_targets() -> None:
    client = _client()  # single-block policy

    with pytest.raises(ValueError, match="declares 1 joint action block"):
        await client.validate_schema(_schema(_mg(MG_ID), _mg("0@other")))


@pytest.mark.asyncio
async def test_validate_schema_rejects_a_joint_target_bound_to_two_arms() -> None:
    client = _client()
    schema = PolicySchema(
        observations=[Observation.joint_positions("both", source=[_mg(MG_ID), _mg("0@other")])]
    )

    with pytest.raises(ValueError, match="bind exactly one motion group"):
        await client.validate_schema(schema)


@pytest.mark.asyncio
async def test_validate_schema_rejects_a_wrongly_bound_motion_group() -> None:
    client = _client()

    with pytest.raises(ValueError, match="Schema binds motion groups"):
        await client.validate_schema(_schema(_mg("0@other")))
