import pytest
from pydantic import ValidationError

from nova import api
from nova.command_routines import (
    at_path_fraction,
    command_routine,
    io_value,
    marker,
    move_joint_ptp,
    move_linear,
    set_io,
    wait_for_time,
    with_settings,
)


def test_builds_mixed_routine():
    routine = command_routine(
        "pick-and-place",
        motion_group_setup=None,
        tcp="flange",
        commands=[
            move_linear("approach"),
            set_io("digital_out[0]", True),
            wait_for_time(500),
            marker("grasp"),
        ],
    )
    assert isinstance(routine, api.models.CommandRoutine)
    assert routine.command_routine == "pick-and-place"
    assert routine.tcp == "flange"
    assert routine.motion_group_setup is None
    assert [cmd.type for cmd in routine.commands] == [
        "motion_command",
        "set_io",
        "wait_for_time",
        "marker",
    ]


def test_move_linear_coerces_pose_id():
    command = move_linear("approach")
    assert command.target == api.models.LocalPoseReference(pose_id="approach")
    assert isinstance(command.path_type, api.models.PathTypeLine)


def test_move_joint_ptp_coerces_joint_list():
    command = move_joint_ptp([0.0] * 6)
    assert command.target == api.models.JointPositionReference(joints=[0.0] * 6)
    assert isinstance(command.path_type, api.models.PathTypeJointPTP)


def test_set_io_discriminates_by_python_type():
    assert isinstance(set_io("io", True).io_value, api.models.IOBooleanValue)
    assert isinstance(set_io("io", 3).io_value, api.models.IOIntegerValue)
    assert isinstance(set_io("io", 1.5).io_value, api.models.IOFloatValue)
    # int is transmitted as a string to avoid precision loss.
    assert set_io("io", 42).io_value.value == "42"


def test_io_value_discriminates_by_python_type():
    assert isinstance(io_value("io", True), api.models.IOBooleanValue)
    assert isinstance(io_value("io", 3), api.models.IOIntegerValue)
    assert isinstance(io_value("io", 1.5), api.models.IOFloatValue)
    # int is transmitted as a string to avoid precision loss.
    assert io_value("io", 42).value == "42"


def test_with_settings_applies_only_to_settingless_motions():
    shared = api.models.MotionSettings(
        limits_override=api.models.LimitsOverride(tcp_velocity_limit=100.0)
    )
    explicit = api.models.MotionSettings(
        limits_override=api.models.LimitsOverride(tcp_velocity_limit=250.0)
    )
    group = with_settings(
        shared, [move_linear("a"), move_linear("b", settings=explicit), set_io("out", True)]
    )
    assert group[0].motion_settings == shared
    assert group[1].motion_settings == explicit
    assert group[2].type == "set_io"


def test_with_settings_does_not_mutate_input():
    original = move_linear("a")
    with_settings(api.models.MotionSettings(), [original])
    assert original.motion_settings is None


def test_at_trigger_positions_set_io():
    routine = command_routine(
        "routine", commands=[move_linear("a"), set_io("out", True, at=at_path_fraction(0.5))]
    )
    assert routine.commands[1].at == api.models.PathFractionTrigger(value=0.5)


def test_motion_group_id_is_wrapped():
    routine = command_routine(
        "routine", motion_group="0@controller", commands=[set_io("out", True)]
    )
    assert routine.motion_group == api.models.MotionGroupReference(id="0@controller")


def test_requires_at_least_one_command():
    with pytest.raises(ValidationError):
        command_routine("routine", commands=[])


def test_invalid_routine_id_is_rejected():
    with pytest.raises(ValidationError):
        command_routine("1-bad-id", commands=[set_io("out", True)])


def test_routine_round_trips_through_json():
    routine = command_routine("routine", commands=[move_joint_ptp([0.0] * 6), set_io("out", True)])
    restored = api.models.CommandRoutine.model_validate_json(routine.model_dump_json())
    assert restored == routine


def test_commands_built_with_comprehension():
    routine = command_routine("routine", commands=[move_linear(p) for p in ("a", "b", "c")])
    assert len(routine.commands) == 3
