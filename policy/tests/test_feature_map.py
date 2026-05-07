"""Tests for FeatureMap: observation building and action parsing."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from policy.feature_map import FeatureGroup, FeatureMap


def _mg(mg_id: str = "0@ur10e", controller_id: str = "ur10e") -> MagicMock:
    mg = MagicMock()
    mg.id = mg_id
    mg._controller_id = controller_id
    mg._cell = "cell"
    return mg


def _state(joints: tuple[float, ...]) -> MagicMock:
    s = MagicMock()
    s.joints = joints
    return s


# --- build_observation ---


@pytest.mark.asyncio
async def test_build_observation_single_arm() -> None:
    fm = FeatureMap(groups=[FeatureGroup(motion_group=_mg(), name="left")])
    obs = fm.build_observation({"0@ur10e": _state((0.1, -1.5, 0.0))})

    assert obs == {
        "left_joint_position_1": 0.1,
        "left_joint_position_2": -1.5,
        "left_joint_position_3": 0.0,
    }


@pytest.mark.asyncio
async def test_build_observation_dual_arm() -> None:
    fm = FeatureMap(groups=[
        FeatureGroup(motion_group=_mg("0@left", "left"), name="left"),
        FeatureGroup(motion_group=_mg("0@right", "right"), name="right"),
    ])
    obs = fm.build_observation({
        "0@left": _state((1.0,)),
        "0@right": _state((2.0,)),
    })

    assert obs["left_joint_position_1"] == 1.0
    assert obs["right_joint_position_1"] == 2.0


@pytest.mark.asyncio
async def test_build_observation_custom_joint_key() -> None:
    fm = FeatureMap(groups=[FeatureGroup(motion_group=_mg(), name="arm", joint_key="state")])
    obs = fm.build_observation({"0@ur10e": _state((0.5,))})

    assert obs == {"state_1": 0.5}


# --- parse_action ---


def test_parse_action_single_arm() -> None:
    fm = FeatureMap(groups=[FeatureGroup(motion_group=_mg(), name="left")])

    joints, ios = fm.parse_action({
        "left_joint_position_1": 0.1,
        "left_joint_position_2": -1.5,
    })

    assert joints == {"0@ur10e": [[0.1, -1.5]]}
    assert ios is None


def test_parse_action_dual_arm() -> None:
    fm = FeatureMap(groups=[
        FeatureGroup(motion_group=_mg("0@left"), name="left"),
        FeatureGroup(motion_group=_mg("0@right"), name="right"),
    ])

    joints, _ = fm.parse_action({
        "left_joint_position_1": 0.1,
        "right_joint_position_1": 0.2,
    })

    assert joints["0@left"] == [[0.1]]
    assert joints["0@right"] == [[0.2]]


def test_parse_action_io_threshold() -> None:
    fm = FeatureMap(groups=[FeatureGroup(
        motion_group=_mg(), name="arm",
        ios={"gripper": "digital_out[0]"}, io_threshold=0.5,
    )])

    _, ios_open = fm.parse_action({"arm_joint_position_1": 0.0, "gripper": 0.8})
    _, ios_closed = fm.parse_action({"arm_joint_position_1": 0.0, "gripper": 0.2})

    assert ios_open == {"0@ur10e": {"digital_out[0]": True}}
    assert ios_closed == {"0@ur10e": {"digital_out[0]": False}}


def test_parse_action_no_matching_keys() -> None:
    fm = FeatureMap(groups=[FeatureGroup(motion_group=_mg(), name="left")])
    joints, ios = fm.parse_action({"unrelated": 1.0})

    assert joints == {}
    assert ios is None
