"""Tests for PolicySchema: observation building, action parsing, mappings, validation."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from policy.schema import (
    Action,
    BoolMapping,
    IdentityMapping,
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
async def test_single_arm_observation():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("arm_joints", source=mg),
    ])
    obs = await schema.build_observation({"0@ur10e": _state((0.1, -1.5, 0.0))})
    assert obs == {"arm_joints_1": 0.1, "arm_joints_2": -1.5, "arm_joints_3": 0.0}


@pytest.mark.asyncio
async def test_dual_arm_observation():
    left = _mg("0@left", "left")
    right = _mg("0@right", "right")
    schema = PolicySchema(observations=[
        Observation.joint_positions("left_joints", source=left),
        Observation.joint_positions("right_joints", source=right),
    ])
    obs = await schema.build_observation({
        "0@left": _state((1.0,)),
        "0@right": _state((2.0,)),
    })
    assert obs["left_joints_1"] == 1.0
    assert obs["right_joints_1"] == 2.0


@pytest.mark.asyncio
async def test_concatenated_observation():
    left = _mg("0@left", "left")
    right = _mg("0@right", "right")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("state", source=[left, right], writable=False),
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
async def test_constant_observation():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.constant("task", value="pick"),
    ])
    obs = await schema.build_observation({"0@ur10e": _state((0.0,))})
    assert obs["task"] == "pick"
    assert obs["joints_1"] == 0.0


@pytest.mark.asyncio
async def test_io_observation_with_bool_mapping():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.io("gripper", source=mg, io="digital_out[0]",
                       mapping=BoolMapping(on=100.0)),
    ])
    obs = await schema.build_observation(
        {"0@ur10e": _state((0.0,))},
        io_values={"digital_out[0]": True},
    )
    assert obs["gripper"] == 100.0


@pytest.mark.asyncio
async def test_joint_torques_observation():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.joint_torques("torques", source=mg),
    ])
    obs = await schema.build_observation({
        "0@ur10e": _state((0.0, 0.1), torques=(1.5, 2.5)),
    })
    assert obs["torques_1"] == 1.5
    assert obs["torques_2"] == 2.5


@pytest.mark.asyncio
async def test_joint_torques_default():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.joint_torques("torques", source=mg, default=[0.0, 0.0]),
    ])
    obs = await schema.build_observation({"0@ur10e": _state((0.0, 0.1))})
    assert obs["torques_1"] == 0.0
    assert obs["torques_2"] == 0.0


# ---------------------------------------------------------------------------
# Action parsing
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_inferred_joint_action():
    """Joint action should be automatically inferred from writable joint observation."""
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
    ])
    joints, ios = await schema.parse_action({"joints_1": 0.5, "joints_2": -1.0})
    assert joints == {"0@ur10e": [[0.5, -1.0]]}
    assert ios is None


@pytest.mark.asyncio
async def test_non_writable_no_inferred_action():
    """writable=False should not infer an action."""
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("state", source=mg, writable=False),
    ])
    joints, _ios = await schema.parse_action({"state_1": 0.5})
    assert joints == {}


@pytest.mark.asyncio
async def test_explicit_action_different_key():
    """Explicit Action.joint_positions with a different key than the observation."""
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("obs_state", source=mg, writable=False),
        ],
        actions=[
            Action.joint_positions("action", target=mg),
        ],
    )
    joints, _ios = await schema.parse_action({"action_1": 0.5, "action_2": -1.0})
    assert joints == {"0@ur10e": [[0.5, -1.0]]}


@pytest.mark.asyncio
async def test_io_action_with_bool_mapping():
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.io("gripper", source=mg, io="digital_out[0]",
                           mapping=BoolMapping(on=100.0)),
        ],
    )
    _, ios_open = await schema.parse_action({"joints_1": 0.0, "gripper": 80.0})
    _, ios_closed = await schema.parse_action({"joints_1": 0.0, "gripper": 20.0})
    assert ios_open == {"0@ur10e": {"digital_out[0]": True}}
    assert ios_closed == {"0@ur10e": {"digital_out[0]": False}}


@pytest.mark.asyncio
async def test_concatenated_action():
    left = _mg("0@left", "left")
    right = _mg("0@right", "right")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("state", source=[left, right], writable=False),
        ],
        actions=[
            Action.joint_positions("action", target=[left, right]),
        ],
    )
    joints, _ = await schema.parse_action({
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
    joints, ios = await schema.parse_action({"unrelated": 1.0})
    assert joints == {}
    assert ios is None


# ---------------------------------------------------------------------------
# Mappings
# ---------------------------------------------------------------------------


def test_identity_mapping():
    m = IdentityMapping()
    assert m.to_policy(3.14) == 3.14
    assert m.to_hardware(3.14) == 3.14


def test_bool_mapping():
    m = BoolMapping(on=100.0)
    assert m.to_policy(True) == 100.0
    assert m.to_policy(False) == 0.0
    assert m.threshold == 50.0  # midpoint
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
# Grouped observation (GR00T)
# ---------------------------------------------------------------------------


def test_grouped_observation():
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("arm", source=mg),
    ])
    grouped = schema.build_grouped_observation({"0@ur10e": _state((0.1, -1.5))})
    assert len(grouped) == 1
    assert grouped[0].key == "arm"
    assert grouped[0].joints == [0.1, -1.5]
    assert grouped[0].motion_group_id == "0@ur10e"


# ---------------------------------------------------------------------------
# Schema introspection
# ---------------------------------------------------------------------------


def test_get_motion_groups():
    left = _mg("0@left", "left")
    right = _mg("0@right", "right")
    schema = PolicySchema(observations=[
        Observation.joint_positions("left", source=left),
        Observation.joint_positions("right", source=right),
    ])
    mgs = schema.get_motion_groups()
    assert len(mgs) == 2
    assert mgs[0].id == "0@left"
    assert mgs[1].id == "0@right"


def test_io_keys_by_controller():
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.io("sensor", source=mg, io="digital_in[0]", writable=False),
            Observation.io("gripper", source=mg, io="digital_out[0]"),
        ],
    )
    io_keys = schema.io_keys_by_controller()
    assert io_keys == {"ur10e": ["digital_in[0]", "digital_out[0]"]}


@pytest.mark.asyncio
async def test_writable_io_infers_action():
    """Writable IO observation auto-infers a matching action."""
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.io("gripper", source=mg, io="digital_out[0]",
                       mapping=BoolMapping(on=100.0)),
    ])
    _, ios = await schema.parse_action({"joints_1": 0.0, "gripper": 80.0})
    assert ios == {"0@ur10e": {"digital_out[0]": True}}


@pytest.mark.asyncio
async def test_non_writable_io_no_action():
    """Non-writable IO observation does not infer an action."""
    mg = _mg()
    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.io("sensor", source=mg, io="digital_in[0]", writable=False),
    ])
    _, ios = await schema.parse_action({"joints_1": 0.0, "sensor": 1.0})
    assert ios is None


@pytest.mark.asyncio
async def test_explicit_action_overrides_inferred_io():
    """Explicit Action.io() overrides the inferred one from observation."""
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.io("gripper", source=mg, io="digital_in[2]", writable=False),
        ],
        actions=[
            Action.io("gripper", target=mg, io="digital_out[0]",
                      mapping=BoolMapping(on=1.0)),
        ],
    )
    # Reads from digital_in[2], writes to digital_out[0]
    _, ios = await schema.parse_action({"joints_1": 0.0, "gripper": 0.8})
    assert ios == {"0@ur10e": {"digital_out[0]": True}}


# ---------------------------------------------------------------------------
# Computed observations
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_computed_observation():
    mg = _mg()

    async def read_sensor(obs: dict) -> dict:
        # Can access already-built obs (joints are filled first)
        joint_sum = sum(v for k, v in obs.items() if k.startswith("joints_"))
        return {"temperature": 42.5, "force_z": joint_sum * 10.0}

    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.computed(read_sensor),
    ])
    obs = await schema.build_observation({"0@ur10e": _state((0.1, 0.2))})
    assert obs["joints_1"] == 0.1
    assert obs["joints_2"] == 0.2
    assert obs["temperature"] == 42.5
    assert obs["force_z"] == pytest.approx(3.0)  # (0.1 + 0.2) * 10


@pytest.mark.asyncio
async def test_multiple_computed_observations():
    mg = _mg()

    async def read_opcua(obs: dict) -> dict:
        return {"plc_temp": 25.0}

    async def read_force_sensor(obs: dict) -> dict:
        return {"force_x": 1.0, "force_y": 2.0}

    schema = PolicySchema(observations=[
        Observation.joint_positions("joints", source=mg),
        Observation.computed(read_opcua),
        Observation.computed(read_force_sensor),
    ])
    obs = await schema.build_observation({"0@ur10e": _state((0.0,))})
    assert obs["plc_temp"] == 25.0
    assert obs["force_x"] == 1.0
    assert obs["force_y"] == 2.0


# ---------------------------------------------------------------------------
# Computed actions
# ---------------------------------------------------------------------------


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
    joints, _ = await schema.parse_action({"joints_1": 0.5, "conveyor_speed": 42.0})
    assert joints == {"0@ur10e": [[0.5]]}
    assert triggered["conveyor"] == 42.0


@pytest.mark.asyncio
async def test_computed_action_with_io():
    mg = _mg()
    log = []

    async def log_action(action: dict) -> None:
        log.append(dict(action))

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.io("gripper", source=mg, io="digital_out[0]",
                           mapping=BoolMapping(on=100.0)),
        ],
        actions=[Action.computed(log_action)],
    )
    _, ios = await schema.parse_action({"joints_1": 0.0, "gripper": 80.0})
    assert ios == {"0@ur10e": {"digital_out[0]": True}}
    assert len(log) == 1
    assert log[0]["gripper"] == 80.0
