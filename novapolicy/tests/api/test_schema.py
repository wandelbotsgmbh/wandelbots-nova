"""Tests for PolicySchema: observation building, action parsing, mappings, validation."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from novapolicy.schema import (
    Action,
    BoolMapping,
    Observation,
    PolicySchema,
)
from novapolicy.types import ActionChunk


def _mg(mg_id: str = "0@ur10e", controller_id: str = "ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = controller_id
    mg._cell = "cell"
    return mg


def _state(joints: tuple[float, ...]) -> MagicMock:
    s = MagicMock()
    s.joints = joints
    s.pose = None
    s.tcp = None
    return s


# ---------------------------------------------------------------------------
# Observation building
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_dual_arm_observation():
    left = _mg("0@left", "left")
    right = _mg("0@right", "right")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left_joints", source=left),
            Observation.joint_positions("right_joints", source=right),
        ]
    )
    obs = await schema.build_observation({
        "0@left": _state((1.0, 2.0)),
        "0@right": _state((3.0,)),
    })
    assert obs == {"left_joints_1": 1.0, "left_joints_2": 2.0, "right_joints_1": 3.0}


@pytest.mark.asyncio
async def test_concatenated_observation():
    left = _mg("0@left", "left")
    right = _mg("0@right", "right")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("state", source=[left, right], action=False),
        ],
        actions=[
            Action.joint_positions("action", target=[left, right]),
        ],
    )
    obs = await schema.build_observation({
        "0@left": _state((1.0, 2.0)),
        "0@right": _state((3.0, 4.0)),
    })
    assert obs == {"state_1": 1.0, "state_2": 2.0, "state_3": 3.0, "state_4": 4.0}


@pytest.mark.asyncio
async def test_constant_and_io_observation():
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.constant("task", value="pick"),
            Observation.io(
                "gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=100.0)
            ),
        ]
    )
    obs = await schema.build_observation(
        {"0@ur10e": _state((0.0,))},
        io_values={"digital_out[0]": True},
    )
    assert obs["task"] == "pick"
    assert obs["joints_1"] == 0.0
    assert obs["gripper"] == 100.0


# ---------------------------------------------------------------------------
# Computed hooks
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_observation_computed_is_evaluated():
    """Observation.computed runs during build_observation and is merged into obs."""
    mg = _mg()
    calls: list[dict] = []

    async def read_sensor(obs: dict) -> dict:
        calls.append(dict(obs))
        return {"sensor": 42.0}

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.computed(read_sensor),
        ]
    )
    obs = await schema.build_observation({"0@ur10e": _state((0.1, 0.2))})

    assert len(calls) == 1
    assert obs["sensor"] == 42.0


@pytest.mark.asyncio
async def test_action_computed_is_evaluated():
    """Action.computed fires once with the policy's ActionChunk."""
    mg = _mg()
    received: list[ActionChunk] = []

    async def journal(chunk: ActionChunk) -> None:
        received.append(chunk)

    schema = PolicySchema(
        observations=[Observation.joint_positions("joints", source=mg)],
        actions=[Action.computed(journal)],
    )
    chunk = ActionChunk(joints={"0@ur10e": [[0.1, 0.2]]})
    await schema.run_computed_actions(chunk)

    assert received == [chunk]


# ---------------------------------------------------------------------------
# Mappings
# ---------------------------------------------------------------------------


def test_bool_mapping():
    m = BoolMapping(on=100.0)
    assert m.to_policy(True) == 100.0
    assert m.to_policy(False) == 0.0
    assert m.threshold == 50.0
    assert m.to_hardware(80.0) is True
    assert m.to_hardware(20.0) is False


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


def test_duplicate_observation_key_raises():
    mg = _mg()
    with pytest.raises(ValueError, match="Duplicate observation key"):
        PolicySchema(
            observations=[
                Observation.joint_positions("joints", source=mg),
                Observation.joint_positions("joints", source=mg),
            ]
        )


def test_duplicate_action_key_raises():
    mg = _mg()
    with pytest.raises(ValueError, match="Duplicate action key"):
        PolicySchema(
            observations=[Observation.joint_positions("joints", source=mg)],
            actions=[
                Action.io("out", target=mg, io="digital_out[0]"),
                Action.io("out", target=mg, io="digital_out[1]"),
            ],
        )


# ---------------------------------------------------------------------------
# Computed observations and actions
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_computed_observation():
    mg = _mg()

    async def read_sensor(obs: dict) -> dict:
        joint_sum = sum(v for k, v in obs.items() if k.startswith("joints_"))
        return {"temperature": 42.5, "force_z": joint_sum * 10.0}

    async def read_opcua(obs: dict) -> dict:
        return {"plc_temp": 25.0}

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.computed(read_sensor),
            Observation.computed(read_opcua),
        ]
    )
    obs = await schema.build_observation({"0@ur10e": _state((0.1, 0.2))})
    assert obs["temperature"] == 42.5
    assert obs["force_z"] == pytest.approx(3.0)
    assert obs["plc_temp"] == 25.0


# ---------------------------------------------------------------------------
# Relative mode
# ---------------------------------------------------------------------------


def test_relative_motion_groups():
    mg1 = _mg()
    mg2 = _mg("0@ur10e-2")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left", source=mg1, mode="relative"),
            Observation.joint_positions("right", source=mg2, mode="absolute"),
        ]
    )
    assert schema.relative_motion_groups() == {"0@ur10e"}
