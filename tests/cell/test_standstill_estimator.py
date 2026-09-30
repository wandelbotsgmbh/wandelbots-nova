"""Tests for StandstillEstimator: debouncing the controller's ``standstill`` flag.

The flag is an observed threshold on measured joint velocities and currently drops
to ``false`` for single frames while the robot is at rest. The estimator must ignore
such flickers, still see real motion at once when the commanded location or the
measured joints corroborate it, and — configured as ``passthrough`` — return the raw
flag on the same frame, for when upstream fixes it.
"""

import random
from datetime import datetime, timezone

import pytest

from nova import api
from nova.cell.movement_controller.policy import ExecutionPolicy
from nova.cell.movement_controller.standstill import StandstillConfig, StandstillEstimator


def _frame(
    standstill: bool,
    location: float | None = None,
    *,
    joints: list[float] | None = None,
    trajectory: str = "traj-1",
) -> api.models.MotionGroupState:
    execute = None
    if location is not None:
        execute = api.models.Execute(
            joint_position=[0.0] * 6,
            details=api.models.TrajectoryDetails(
                trajectory=trajectory, location=location, state=api.models.TrajectoryPausedByUser()
            ),
        )
    return api.models.MotionGroupState(
        timestamp=datetime.now(timezone.utc),
        sequence_number=1,
        description_revision=0,
        motion_group="mg-0",
        controller="ctrl-0",
        joint_position=joints if joints is not None else [0.0] * 6,
        joint_limit_reached=api.models.MotionGroupStateJointLimitReached(limit_reached=[False] * 6),
        standstill=standstill,
        execute=execute,
    )


def _decisions(estimator: StandstillEstimator, frames) -> list[bool]:
    return [estimator.update(frame).at_rest for frame in frames]


def test_passthrough_returns_the_raw_flag_on_the_same_frame():
    rng = random.Random(7)
    flags = [rng.random() < 0.5 for _ in range(500)]
    estimator = StandstillEstimator(StandstillConfig.passthrough())

    readings = [
        estimator.update(_frame(flag, rng.random(), joints=[rng.random()] * 6)) for flag in flags
    ]

    assert [reading.at_rest for reading in readings] == flags
    assert all(reading.jitter is None for reading in readings)


def test_passthrough_is_the_default():
    assert StandstillEstimator().config.is_passthrough
    assert not StandstillConfig.robust().is_passthrough


def test_a_single_flicker_at_rest_is_ignored_and_reported():
    estimator = StandstillEstimator(StandstillConfig.robust())
    frames = [_frame(True, 0.0), _frame(False, 0.0), _frame(True, 0.0), _frame(True, 0.0)]

    readings = [estimator.update(frame) for frame in frames]

    assert [reading.at_rest for reading in readings] == [True, True, True, True]
    assert readings[1].disagrees
    assert [reading.jitter for reading in readings] == [None, None, "motion", None]


def test_uncorroborated_motion_needs_motion_votes_frames():
    estimator = StandstillEstimator(StandstillConfig(motion_votes=3))

    decisions = _decisions(estimator, [_frame(True, 0.0)] + [_frame(False, 0.0)] * 3)

    assert decisions == [True, True, True, False]


def test_a_moving_commanded_location_proves_motion_at_once():
    estimator = StandstillEstimator(StandstillConfig.robust())

    readings = [estimator.update(frame) for frame in (_frame(True, 0.0), _frame(False, 0.01))]

    assert readings[1].at_rest is False
    assert readings[1].evidence == "location"


def test_a_location_jump_between_trajectories_is_not_motion():
    estimator = StandstillEstimator(StandstillConfig.robust())

    decisions = _decisions(
        estimator, [_frame(True, 2.0, trajectory="old"), _frame(False, 0.0, trajectory="new")]
    )

    assert decisions == [True, True]


def test_a_standstill_frame_at_a_new_location_still_counts_as_rest():
    """The first frame at the end of a trajectory has a new location and a settled robot."""
    estimator = StandstillEstimator(StandstillConfig(rest_votes=1, location_epsilon=1e-6))

    decisions = _decisions(estimator, [_frame(False, 0.5), _frame(True, 2.0)])

    assert decisions == [False, True]


def test_moving_measured_joints_prove_motion_and_veto_rest():
    estimator = StandstillEstimator(StandstillConfig.robust())

    readings = [
        estimator.update(frame)
        for frame in (
            _frame(True, joints=[0.0] * 6),
            _frame(False, joints=[0.01] + [0.0] * 5),
            _frame(True, joints=[0.02] + [0.0] * 5),  # flag says rest, joints still move
            _frame(True, joints=[0.02] + [0.0] * 5),
            _frame(True, joints=[0.02] + [0.0] * 5),
        )
    ]

    assert [reading.at_rest for reading in readings] == [True, False, False, False, True]
    assert readings[1].evidence == "joints"
    assert readings[2].evidence == "joints"


def test_joint_noise_below_epsilon_is_rest():
    estimator = StandstillEstimator(StandstillConfig.robust())

    decisions = _decisions(
        estimator, [_frame(True, joints=[0.0] * 6), _frame(False, joints=[1e-5] * 6), _frame(True)]
    )

    assert decisions == [True, True, True]


def test_rest_needs_rest_votes_frames_and_a_flicker_while_moving_is_reported():
    estimator = StandstillEstimator(StandstillConfig(rest_votes=2))
    frames = [_frame(False), _frame(True), _frame(False), _frame(True), _frame(True)]

    readings = [estimator.update(frame) for frame in frames]

    assert [reading.at_rest for reading in readings] == [False, False, False, False, True]
    assert readings[2].jitter == "rest"


def test_votes_must_be_positive():
    with pytest.raises(ValueError):
        StandstillConfig(motion_votes=0)


class TestPolicyFromEnvironment:
    def test_default_is_robust(self):
        policy = ExecutionPolicy.from_env({})
        assert policy == ExecutionPolicy.robust()
        assert policy.strict is False
        assert policy.standstill == StandstillConfig.robust()

    def test_strict_is_the_raw_flag_with_failing_contradictions(self):
        policy = ExecutionPolicy.from_env({"NOVA_EXECUTION_POLICY": "strict"})
        assert policy.strict is True
        assert policy.standstill.is_passthrough
        assert policy.standstill.strict is True

    def test_diagnose_debounces_but_fails_on_every_finding(self):
        policy = ExecutionPolicy.from_env({"NOVA_EXECUTION_POLICY": "diagnose"})
        assert policy.strict is True
        assert not policy.standstill.is_passthrough
        assert policy.standstill.strict is True

    def test_knobs_override_the_preset(self):
        policy = ExecutionPolicy.from_env(
            {
                "NOVA_STANDSTILL_MOTION_VOTES": "5",
                "NOVA_STANDSTILL_REST_VOTES": "1",
                "NOVA_STANDSTILL_LOCATION_EPSILON": "none",
                "NOVA_STANDSTILL_JOINT_EPSILON": "0.002",
                "NOVA_RESUME_DETECT_MS": "250",
                "NOVA_RESUME_WINDOW_MS": "2000",
            }
        )
        assert policy.standstill == StandstillConfig(
            motion_votes=5, rest_votes=1, location_epsilon=None, joint_epsilon=0.002
        )
        assert policy.resume_detect_s == 0.25
        assert policy.resume_window_s == 2.0

    def test_robust_tuned_to_one_vote_and_no_evidence_is_passthrough(self):
        policy = ExecutionPolicy.from_env(
            {
                "NOVA_STANDSTILL_MOTION_VOTES": "1",
                "NOVA_STANDSTILL_REST_VOTES": "1",
                "NOVA_STANDSTILL_LOCATION_EPSILON": "none",
                "NOVA_STANDSTILL_JOINT_EPSILON": "none",
            }
        )
        assert policy.standstill.is_passthrough

    def test_an_unknown_preset_is_rejected(self):
        with pytest.raises(ValueError, match="NOVA_EXECUTION_POLICY"):
            ExecutionPolicy.from_env({"NOVA_EXECUTION_POLICY": "lenient"})
