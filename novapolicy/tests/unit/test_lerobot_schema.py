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


# ---------------------------------------------------------------------------
# assert_matches: schema against the checkpoint's feature contract
# ---------------------------------------------------------------------------

LeRobotExecutionSettings = pytest.importorskip("novapolicy.lerobot.config").LeRobotExecutionSettings
_types = pytest.importorskip("lerobot.configs.types")
FeatureType = _types.FeatureType
PolicyFeature = _types.PolicyFeature


def _settings(
    *,
    inputs: dict | None = None,
    outputs: dict | None = None,
) -> object:
    """Checkpoint settings for a 6-DOF arm with one 320x240 camera."""
    if inputs is None:
        inputs = {
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(6,)),
            "observation.images.scene": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 240, 320)),
        }
    if outputs is None:
        outputs = {"action": PolicyFeature(type=FeatureType.ACTION, shape=(6,))}
    return LeRobotExecutionSettings(
        policy_type="act",
        chunk_size=16,
        n_action_steps=8,
        input_features=inputs,
        output_features=outputs,
    )


def _matching_schema(mg) -> PolicySchema:
    camera = MagicMock()
    return PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.image("scene", source=camera),
        ]
    )


def _check(schema, settings, *, state_names, images, layout=None) -> None:
    lerobot_schema = LeRobotSchema(dt_ms=50.0)
    if layout is None:
        layout = FlatActionLayout(joints=[("0@cobot", slice(0, 6))], tcp=[], ios=[])
    lerobot_schema.assert_matches(settings, schema, state_names, images, layout)


def _frame(height: int = 240, width: int = 320):
    return np.zeros((height, width, 3), dtype=np.uint8)


def test_assert_matches_accepts_a_schema_that_matches_the_checkpoint() -> None:
    mg = _mg()
    _check(
        _matching_schema(mg),
        _settings(),
        state_names=[f"arm_{i}" for i in range(1, 7)],
        images={"scene": _frame()},
    )


def test_assert_matches_rejects_a_wrong_state_width() -> None:
    mg = _mg()

    with pytest.raises(ValueError, match=r"observation\.state.*shape \(7,\).*expects \(6,\)"):
        _check(
            _matching_schema(mg),
            _settings(),
            state_names=[f"arm_{i}" for i in range(1, 8)],
            images={"scene": _frame()},
        )


def test_assert_matches_reports_a_camera_the_checkpoint_does_not_declare() -> None:
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.image("scene", source=MagicMock()),
            Observation.image("wrist", source=MagicMock()),
        ]
    )

    with pytest.raises(ValueError, match=r"observation\.images\.wrist.*not declared"):
        _check(
            schema,
            _settings(),
            state_names=[f"arm_{i}" for i in range(1, 7)],
            images={"scene": _frame(), "wrist": _frame()},
        )


def test_assert_matches_reports_a_camera_missing_from_the_schema() -> None:
    mg = _mg()
    schema = PolicySchema(observations=[Observation.joint_positions("arm", source=mg)])

    with pytest.raises(ValueError, match=r"observation\.images\.scene.*is missing"):
        _check(
            schema,
            _settings(),
            state_names=[f"arm_{i}" for i in range(1, 7)],
            images=None,
        )


def test_a_higher_resolution_at_the_same_aspect_warns_about_wasted_pixels(caplog) -> None:
    """640x480 against a 320x240 checkpoint: both 4:3, so rescaling is clean.

    The comparison is channel-first: our HWC frame against the checkpoint's CHW
    declaration.
    """
    mg = _mg()

    with caplog.at_level("WARNING"):
        _check(
            _matching_schema(mg),
            _settings(),
            state_names=[f"arm_{i}" for i in range(1, 7)],
            images={"scene": _frame(height=480, width=640)},
        )

    assert "aspect ratio matches" in caplog.text
    assert "640x480" in caplog.text
    assert "320x240" in caplog.text
    assert "stretched" not in caplog.text


def test_a_different_aspect_ratio_warns_that_rescaling_cannot_fix_it(caplog) -> None:
    """1920x1080 against a 320x240 checkpoint: 16:9 into 4:3 distorts."""
    mg = _mg()

    with caplog.at_level("WARNING"):
        _check(
            _matching_schema(mg),
            _settings(),
            state_names=[f"arm_{i}" for i in range(1, 7)],
            images={"scene": _frame(height=1080, width=1920)},
        )

    assert "different aspect ratio" in caplog.text
    assert "stretched" in caplog.text
    assert "1920x1080" in caplog.text


def test_a_matching_frame_size_says_nothing(caplog) -> None:
    mg = _mg()

    with caplog.at_level("WARNING"):
        _check(
            _matching_schema(mg),
            _settings(),
            state_names=[f"arm_{i}" for i in range(1, 7)],
            images={"scene": _frame()},
        )

    assert not caplog.text


def test_assert_matches_rejects_a_wrong_action_width() -> None:
    mg = _mg()

    with pytest.raises(ValueError, match="action width is 7, checkpoint expects 6"):
        _check(
            _matching_schema(mg),
            _settings(),
            state_names=[f"arm_{i}" for i in range(1, 7)],
            images={"scene": _frame()},
            layout=FlatActionLayout(
                joints=[("0@cobot", slice(0, 6))],
                tcp=[],
                ios=[("0@cobot", "digital_out[0]", None, slice(6, 7))],
            ),
        )


def test_assert_matches_reports_every_structural_problem_at_once() -> None:
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            Observation.image("scene", source=MagicMock()),
            Observation.image("wrist", source=MagicMock()),
        ]
    )

    with pytest.raises(ValueError) as excinfo:
        _check(
            schema,
            _settings(),
            state_names=[f"arm_{i}" for i in range(1, 8)],
            images={"scene": _frame(), "wrist": _frame()},
        )

    message = str(excinfo.value)
    assert "observation.state" in message
    assert "observation.images.wrist" in message


def test_assert_matches_ignores_language_features_it_cannot_produce() -> None:
    """A task-conditioned checkpoint must not fail on its LANGUAGE feature."""
    mg = _mg()
    settings = _settings(
        inputs={
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(6,)),
            "observation.images.scene": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 240, 320)),
            "task": PolicyFeature(type=FeatureType.LANGUAGE, shape=(1,)),
        }
    )

    _check(
        _matching_schema(mg),
        settings,
        state_names=[f"arm_{i}" for i in range(1, 7)],
        images={"scene": _frame()},
    )


def test_assert_matches_warns_and_skips_when_the_checkpoint_declares_no_features(caplog) -> None:
    mg = _mg()

    with caplog.at_level("WARNING"):
        _check(
            _matching_schema(mg),
            _settings(inputs={}, outputs={}),
            state_names=[f"arm_{i}" for i in range(1, 9)],
            images={"scene": _frame()},
        )

    assert "declares no input features" in caplog.text


def test_flat_action_layout_width_spans_every_slice() -> None:
    layout = FlatActionLayout(
        joints=[("0@left", slice(0, 6)), ("0@right", slice(6, 12))],
        tcp=[],
        ios=[("0@left", "digital_out[0]", None, slice(12, 13))],
    )

    assert layout.width == 13
    assert FlatActionLayout(joints=[], tcp=[], ios=[]).width == 0


# ---------------------------------------------------------------------------
# The first live observation against the checkpoint's own statistics
# ---------------------------------------------------------------------------

check_observation_range = schema_module.check_observation_range
FeatureStats = pytest.importorskip("novapolicy.lerobot.config").FeatureStats


def _stats_settings(**stats) -> object:
    return LeRobotExecutionSettings(
        policy_type="act",
        chunk_size=16,
        n_action_steps=8,
        stats={"observation.state": FeatureStats(**stats)},
    )


_NAMES = ["arm_1", "arm_2"]


def test_an_in_range_observation_passes_quietly(caplog) -> None:
    settings = _stats_settings(minimum=(-3.14, -3.14), maximum=(3.14, 3.14))

    with caplog.at_level("WARNING"):
        check_observation_range(settings, {"arm_1": 1.0, "arm_2": -2.0}, _NAMES)

    assert not caplog.text


def test_a_pose_just_outside_the_demonstrations_only_warns(caplog) -> None:
    """A start pose the demonstrations never visited is normal, not a unit error."""
    settings = _stats_settings(minimum=(-1.0, -1.0), maximum=(1.0, 1.0))

    with caplog.at_level("WARNING"):
        check_observation_range(settings, {"arm_1": 5.0, "arm_2": 0.0}, _NAMES)

    assert "outside the training distribution" in caplog.text


def test_degrees_against_a_radian_checkpoint_is_rejected() -> None:
    """The case this exists for: a ~57x error lands far outside any pose."""
    settings = _stats_settings(minimum=(-3.14, -3.14), maximum=(3.14, 3.14))

    with pytest.raises(ValueError, match="implausibly far outside"):
        check_observation_range(settings, {"arm_1": 180.0, "arm_2": -90.0}, _NAMES)


def test_the_rejection_names_the_dimension_and_both_ranges() -> None:
    settings = _stats_settings(minimum=(-3.14, -3.14), maximum=(3.14, 3.14))

    with pytest.raises(ValueError) as excinfo:
        check_observation_range(settings, {"arm_1": 0.0, "arm_2": 180.0}, _NAMES)

    message = str(excinfo.value)
    assert "'arm_2'" in message
    assert "180" in message
    assert "arm_1" not in message


def test_no_statistics_means_no_check() -> None:
    settings = LeRobotExecutionSettings(policy_type="act")

    check_observation_range(settings, {"arm_1": 999.0}, _NAMES)


def test_a_state_width_mismatch_skips_the_check() -> None:
    """Width disagreement is the feature contract's failure to report, not this one."""
    settings = _stats_settings(minimum=(-1.0,), maximum=(1.0,))

    check_observation_range(settings, {"arm_1": 500.0, "arm_2": 500.0}, _NAMES)


def test_dimensions_without_usable_bounds_are_skipped() -> None:
    settings = _stats_settings(mean=(0.0, 0.0), std=(0.0, 1.0))

    with pytest.raises(ValueError, match="'arm_2'"):
        check_observation_range(settings, {"arm_1": 999.0, "arm_2": 999.0}, _NAMES)


def test_a_near_constant_dimension_does_not_fail_a_legitimate_pose(caplog) -> None:
    """Regression, measured against a live UR10e.

    Wrist joint 5 barely moved across the choreo3 demonstrations — a trained
    span of 0.022 rad. Scaling the failure threshold by that span gave a
    +/-0.09 rad window, so the arm's own park pose was reported as a unit
    mismatch. A units error is an error of scale, so magnitude decides.
    """
    settings = _stats_settings(minimum=(-1.577, 2.005), maximum=(-1.555, 2.662))

    with caplog.at_level("WARNING"):
        check_observation_range(settings, {"arm_1": 1.571, "arm_2": -1.571}, _NAMES)

    assert "outside the training distribution" in caplog.text


def test_the_same_pose_in_the_wrong_units_is_still_rejected() -> None:
    """The park pose above, reported in degrees against a radian checkpoint."""
    settings = _stats_settings(minimum=(-1.577, 2.005), maximum=(-1.555, 2.662))

    with pytest.raises(ValueError, match="implausibly far outside"):
        check_observation_range(settings, {"arm_1": 90.0, "arm_2": -90.0}, _NAMES)
