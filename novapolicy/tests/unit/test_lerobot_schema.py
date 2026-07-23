"""Unit tests for LeRobot schema validation and flat action layout."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from novapolicy.schema import Observation, PolicySchema

schema_module = pytest.importorskip("novapolicy.lerobot.schema")
FlatActionLayout = schema_module.FlatActionLayout
LeRobotSchema = schema_module.LeRobotSchema


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


def test_flat_action_layout_orders_joints_then_tcp_then_ios() -> None:
    joint_mg = _mg("0@joint", "joint")
    tcp_mg = _mg("0@tcp", "tcp")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=joint_mg),
            Observation.tcp("eef", source=tcp_mg, action=True),
            Observation.io("gripper", source=joint_mg, io="digital_out[0]"),
        ]
    )

    LeRobotSchema.validate_schema(schema)
    layout = LeRobotSchema(dt_ms=50.0).action_layout(
        {
            joint_mg.id: _state((0.0,) * 6),
            tcp_mg.id: _tcp_state((0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        },
        schema,
    )

    assert layout.joints == [(joint_mg.id, slice(0, 6))]
    assert layout.tcp == [(tcp_mg.id, slice(6, 12))]
    assert [
        (group_id, io, action_slice) for group_id, io, _mapping, action_slice in layout.ios
    ] == [(joint_mg.id, "digital_out[0]", slice(12, 13))]


def test_decode_tcp_action_requires_six_values() -> None:
    schema = LeRobotSchema(dt_ms=50.0)
    layout = FlatActionLayout(joints=[], tcp=[("0@tcp", slice(0, 6))], ios=[])

    with pytest.raises(ValueError, match="expected 6 values, got 5"):
        schema.decode_arrays(
            [np.asarray([1.0, 2.0, 3.0, 0.1, 0.2], dtype=np.float32)],
            layout,
        )


def test_validate_schema_accepts_joint_targets_for_different_groups() -> None:
    left = _mg("0@left", "left")
    right = _mg("0@right", "right")
    schema = PolicySchema(observations=[Observation.joint_positions("arms", source=[left, right])])

    LeRobotSchema.validate_schema(schema)


def test_validate_schema_rejects_duplicate_joint_targets_for_one_group() -> None:
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("first", source=mg),
            Observation.joint_positions("second", source=mg),
        ]
    )

    with pytest.raises(ValueError, match="multiple joint action targets"):
        LeRobotSchema.validate_schema(schema)


def test_validate_schema_rejects_duplicate_tcp_targets_for_one_group() -> None:
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.tcp("first", source=mg, action=True),
            Observation.tcp("second", source=mg, action=True),
        ]
    )

    with pytest.raises(ValueError, match="multiple TCP action targets"):
        LeRobotSchema.validate_schema(schema)


def test_validate_schema_rejects_duplicate_io_targets_for_one_group() -> None:
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.io("first", source=mg, io="digital_out[0]"),
            Observation.io("second", source=mg, io="digital_out[0]"),
        ]
    )

    with pytest.raises(ValueError, match="multiple actions for the same IO target"):
        LeRobotSchema.validate_schema(schema)
