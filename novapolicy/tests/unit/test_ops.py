"""Unit and schema-level tests for invertible value operators."""

from __future__ import annotations

import math
from unittest.mock import MagicMock

import pytest

from novapolicy.ops import (
    Clamp,
    OpDirection,
    Rad2Deg,
    Scale,
    ValueOp,
    apply_ops,
    apply_ops_inverse,
)
from novapolicy.schema import Action, Observation, PolicySchema
from novapolicy.types import ActionChunk

MG_ID = "0@cobot"


def _mg(mg_id: str = MG_ID) -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = mg_id.split("@")[1]
    mg._cell = "cell"
    return mg


def _state(joints: tuple[float, ...]) -> MagicMock:
    state = MagicMock()
    state.joints = joints
    state.pose = None
    return state


# ---------------------------------------------------------------------------
# The operators themselves
# ---------------------------------------------------------------------------


def test_rad2deg_round_trips_exactly() -> None:
    assert Rad2Deg().forward(math.pi) == pytest.approx(180.0)
    assert Rad2Deg().inverse(180.0) == pytest.approx(math.pi)
    assert Rad2Deg().inverse(Rad2Deg().forward(0.7)) == pytest.approx(0.7)


def test_scale_round_trips_exactly() -> None:
    """mm -> m and back, the TCP position case."""
    assert Scale(0.001).forward(1500.0) == pytest.approx(1.5)
    assert Scale(0.001).inverse(1.5) == pytest.approx(1500.0)


@pytest.mark.parametrize("factor", [0.0, math.inf, math.nan])
def test_a_non_invertible_scale_is_rejected_at_construction(factor: float) -> None:
    with pytest.raises(ValueError, match="finite and non-zero"):
        Scale(factor)


def test_clamp_bounds_both_directions() -> None:
    """Limiting a command is the safety-relevant direction, so it applies on inverse too."""
    clamp = Clamp(-1.0, 1.0)

    assert clamp.forward(5.0) == 1.0
    assert clamp.inverse(5.0) == 1.0
    assert clamp.inverse(-5.0) == -1.0
    assert clamp.inverse(0.5) == 0.5


def test_clamp_rejects_inverted_bounds() -> None:
    with pytest.raises(ValueError, match="low must not exceed high"):
        Clamp(1.0, -1.0)


def test_operator_lists_reverse_on_the_way_back() -> None:
    """Clamp runs in the scaled space forward, so it must un-scale after clamping."""
    ops = [Scale(0.001), Clamp(-1.0, 1.0)]

    # 2000 mm -> 2.0 m -> clamped to 1.0 m
    assert apply_ops([2000.0], ops) == pytest.approx([1.0])
    # 2.0 m -> clamped to 1.0 m -> 1000 mm. Reversing the order would clamp
    # millimetres and return 1 mm instead.
    assert apply_ops_inverse([2.0], ops) == pytest.approx([1000.0])


def test_an_empty_operator_list_passes_values_through() -> None:
    assert apply_ops([1.0, 2.0], []) == [1.0, 2.0]
    assert apply_ops_inverse([1.0, 2.0], []) == [1.0, 2.0]


# ---------------------------------------------------------------------------
# Declaring them on a schema
# ---------------------------------------------------------------------------


class _ForwardOnly(ValueOp):
    direction = OpDirection.FORWARD_ONLY

    def forward(self, value: float) -> float:
        return value

    def inverse(self, value: float) -> float:
        raise NotImplementedError


@pytest.mark.asyncio
async def test_joint_observations_are_converted_on_the_way_in() -> None:
    mg = _mg()
    schema = PolicySchema(
        observations=[Observation.joint_positions("arm", source=mg, ops=[Rad2Deg()])]
    )

    obs = await schema.build_observation({MG_ID: _state((math.pi, 0.0, -math.pi / 2))})

    assert obs["arm_1"] == pytest.approx(180.0)
    assert obs["arm_3"] == pytest.approx(-90.0)


def test_joint_actions_are_converted_on_the_way_out() -> None:
    mg = _mg()
    schema = PolicySchema(
        observations=[Observation.joint_positions("arm", source=mg, ops=[Rad2Deg()])]
    )

    chunk = schema.apply_inverse_ops(ActionChunk(joints={MG_ID: [[180.0, -90.0]]}))

    assert chunk.joints[MG_ID][0] == pytest.approx([math.pi, -math.pi / 2])


def test_a_forward_only_operator_is_refused_on_a_writable_channel() -> None:
    with pytest.raises(ValueError, match="must be reversible"):
        PolicySchema(
            observations=[Observation.joint_positions("arm", source=_mg(), ops=[_ForwardOnly()])]
        )


def test_a_forward_only_operator_is_allowed_on_a_read_only_channel() -> None:
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=_mg(), action=False, ops=[_ForwardOnly()])
        ]
    )

    assert schema.get_motion_groups()


def test_tcp_position_and_orientation_convert_independently() -> None:
    """Position is millimetres and orientation is radians — one list cannot serve both."""
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.tcp(
                "eef",
                source=mg,
                action=True,
                position_ops=[Scale(0.001)],
                orientation_ops=[Rad2Deg()],
            )
        ]
    )

    chunk = schema.apply_inverse_ops(
        ActionChunk(tcp={MG_ID: [[1.5, 0.5, 0.25, 180.0, 0.0, -90.0]]})
    )

    assert chunk.tcp[MG_ID][0][:3] == pytest.approx([1500.0, 500.0, 250.0])
    assert chunk.tcp[MG_ID][0][3:] == pytest.approx([math.pi, 0.0, -math.pi / 2])


def test_an_explicit_action_overrides_the_observations_operators() -> None:
    """Explicit-wins, matching how joint_action_keys already resolves duplicates."""
    mg = _mg()
    schema = PolicySchema(
        observations=[Observation.joint_positions("arm", source=mg, action=False, ops=[Rad2Deg()])],
        actions=[Action.joint_positions("arm", target=mg, ops=[Scale(2.0)])],
    )

    chunk = schema.apply_inverse_ops(ActionChunk(joints={MG_ID: [[4.0]]}))

    assert chunk.joints[MG_ID][0] == pytest.approx([2.0])


def test_a_group_without_operators_is_left_alone() -> None:
    left, right = _mg("0@left"), _mg("0@right")
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left", source=left, ops=[Rad2Deg()]),
            Observation.joint_positions("right", source=right),
        ]
    )

    chunk = schema.apply_inverse_ops(
        ActionChunk(joints={"0@left": [[180.0]], "0@right": [[180.0]]})
    )

    assert chunk.joints["0@left"][0] == pytest.approx([math.pi])
    assert chunk.joints["0@right"][0] == [180.0]


def test_a_schema_without_operators_returns_the_chunk_unchanged() -> None:
    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=_mg())])
    chunk = ActionChunk(joints={MG_ID: [[1.0]]})

    assert schema.apply_inverse_ops(chunk) is chunk
