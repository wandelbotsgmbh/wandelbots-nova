"""Tests for PolicySchema: observation building, action parsing, mappings, validation."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from policy.schema import (
    Action,
    BoolMapping,
    Observation,
    PolicySchema,
)


def _mg(mg_id: str = "0@ur10e", controller_id: str = "ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = controller_id
    mg._cell = "cell"
    return mg


def _state(joints: tuple[float, ...], torques: tuple[float, ...] | None = None) -> MagicMock:
    s = MagicMock()
    s.joints = joints
    s.pose = None
    s.tcp = None
    s.joint_torques = torques
    s.joint_currents = None
    return s


# ---------------------------------------------------------------------------
# Observation building
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_dual_arm_observation():
    left = _mg("0@left", "left")
    right = _mg("0@right", "right")
    schema = PolicySchema(observations=[
        Observation.joint_positions("left_joints", source=left),
        Observation.joint_positions("right_joints", source=right),
    ])
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
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.constant("task", value="pick"),
        Observation.io("gripper", source=mg, io="digital_out[0]",
                       mapping=BoolMapping(on=100.0)),
    ])
    obs = await schema.build_observation(
        {"0@ur10e": _state((0.0,))},
        io_values={"digital_out[0]": True},
    )
    assert obs["task"] == "pick"
    assert obs["joints_1"] == 0.0
    assert obs["gripper"] == 100.0


@pytest.mark.asyncio
async def test_joint_torques_with_and_without_data():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.joint_torques("torques", source=mg, default=[0.0, 0.0]),
    ])
    # With torque data
    obs = await schema.build_observation({
        "0@ur10e": _state((0.0, 0.1), torques=(1.5, 2.5)),
    })
    assert obs["torques_1"] == 1.5
    assert obs["torques_2"] == 2.5

    # Without torque data — falls back to default
    obs = await schema.build_observation({"0@ur10e": _state((0.0, 0.1))})
    assert obs["torques_1"] == 0.0
    assert obs["torques_2"] == 0.0


# ---------------------------------------------------------------------------
# Action parsing
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_inferred_and_non_inferred_actions():
    """action=True infers joint action; action=False does not."""
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
    ])
    joints, _tcp, ios = await schema.parse_action({"joints_1": 0.5, "joints_2": -1.0})
    assert joints == {"0@ur10e": [[0.5, -1.0]]}
    assert ios is None

    # Non-action
    schema2 = PolicySchema(observations=[
        Observation.joint_positions("state", source=mg, action=False),
    ])
    joints2, _, _ = await schema2.parse_action({"state_1": 0.5})
    assert joints2 == {}


@pytest.mark.asyncio
async def test_explicit_action_different_key():
    mg = _mg()
    schema = PolicySchema(
        observations=[Observation.joint_positions("obs", source=mg, action=False)],
        actions=[Action.joint_positions("action", target=mg)],
    )
    joints, _tcp, _ios = await schema.parse_action({"action_1": 0.5, "action_2": -1.0})
    assert joints == {"0@ur10e": [[0.5, -1.0]]}


@pytest.mark.asyncio
async def test_io_action_with_bool_mapping():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.io("gripper", source=mg, io="digital_out[0]",
                       mapping=BoolMapping(on=100.0)),
    ])
    _, _, ios_open = await schema.parse_action({"joints_1": 0.0, "gripper": 80.0})
    _, _, ios_closed = await schema.parse_action({"joints_1": 0.0, "gripper": 20.0})
    assert ios_open == {"0@ur10e": {"digital_out[0]": True}}
    assert ios_closed == {"0@ur10e": {"digital_out[0]": False}}


@pytest.mark.asyncio
async def test_concatenated_action():
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
    joints, _tcp, _ = await schema.parse_action({
        "action_1": 1.0, "action_2": 2.0,
        "action_3": 3.0, "action_4": 4.0,
    })
    assert joints["0@left"] == [[1.0, 2.0]]
    assert joints["0@right"] == [[3.0, 4.0]]


@pytest.mark.asyncio
async def test_no_matching_action_keys():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
    ])
    joints, _tcp, ios = await schema.parse_action({"unrelated": 1.0})
    assert joints == {}
    assert ios is None


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
        PolicySchema(observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.joint_positions("joints", source=mg),
        ])


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

    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.computed(read_sensor),
        Observation.computed(read_opcua),
    ])
    obs = await schema.build_observation({"0@ur10e": _state((0.1, 0.2))})
    assert obs["temperature"] == 42.5
    assert obs["force_z"] == pytest.approx(3.0)
    assert obs["plc_temp"] == 25.0


@pytest.mark.asyncio
async def test_computed_action():
    mg = _mg()
    triggered = {}

    async def write_plc(action: dict) -> None:
        triggered["conveyor"] = action.get("conveyor_speed", 0.0)

    schema = PolicySchema(
        observations=[Observation.joint_positions("joints", source=mg)],
        actions=[Action.computed(write_plc)],
    )
    joints, _tcp, _ = await schema.parse_action({"joints_1": 0.5, "conveyor_speed": 42.0})
    assert joints == {"0@ur10e": [[0.5]]}
    assert triggered["conveyor"] == 42.0


# ---------------------------------------------------------------------------
# Relative mode
# ---------------------------------------------------------------------------


def test_relative_motion_groups():
    mg1 = _mg()
    mg2 = _mg("0@ur10e-2")
    schema = PolicySchema(observations=[
        Observation.joint_positions("left", source=mg1, mode="relative"),
        Observation.joint_positions("right", source=mg2, mode="absolute"),
    ])
    assert schema.relative_motion_groups() == {"0@ur10e"}


@pytest.mark.asyncio
async def test_parse_action_returns_raw_values_for_relative():
    """parse_action returns raw values regardless of mode — executor handles conversion."""
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("arm", source=mg, mode="relative"),
    ])
    joints, _tcp, _ = await schema.parse_action({"arm_1": 0.1, "arm_2": -0.2})
    assert joints["0@ur10e"][0] == [0.1, -0.2]
