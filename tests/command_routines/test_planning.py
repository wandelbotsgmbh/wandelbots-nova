import pytest

from nova import api
from nova.actions.io import WriteAction
from nova.actions.mock import WaitAction
from nova.actions.motions import CartesianPTP, Circular, CollisionFreeMotion, JointPTP, Linear
from nova.command_routines import (
    UnsupportedCommandRoutineFeature,
    actions_from_command_routine,
    at_path_fraction,
    command_routine,
    joint_position,
    marker,
    move_joint_ptp,
    move_linear,
    set_io,
    wait_for_io,
    wait_for_time,
)
from nova.command_routines.commands import (
    generated_motion,
    motion,
    path_cartesian_ptp,
    path_circle,
    path_line,
)
from nova.types import Pose


def _routine(commands, **kwargs):
    return command_routine("routine", commands=commands, **kwargs)


def _inline(position, orientation=(0, 0, 0)) -> api.models.InlinePoseReference:
    return api.models.InlinePoseReference(
        pose=api.models.Pose(position=position, orientation=orientation)
    )


class TestExplicitMotionCommands:
    def test_line_command_converts_to_linear_action(self):
        command = motion(_inline((1, 2, 3), (4, 5, 6)), path_line())
        [action] = actions_from_command_routine(_routine([command]))
        assert isinstance(action, Linear)
        assert action.target == Pose((1, 2, 3, 4, 5, 6))

    def test_cartesian_ptp_command_converts_to_cartesian_ptp_action(self):
        command = motion(_inline((1, 2, 3)), path_cartesian_ptp())
        [action] = actions_from_command_routine(_routine([command]))
        assert isinstance(action, CartesianPTP)

    def test_move_joint_ptp_converts_to_joint_ptp_action(self):
        joints = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        [action] = actions_from_command_routine(_routine([move_joint_ptp(joints)]))
        assert isinstance(action, JointPTP)
        assert action.target == tuple(joints)

    def test_circular_command_converts_to_circular_action(self):
        command = motion(_inline((1, 2, 3)), path_circle(_inline((4, 5, 6))))
        [action] = actions_from_command_routine(_routine([command]))
        assert isinstance(action, Circular)
        assert action.target == Pose((1, 2, 3, 0, 0, 0))
        assert action.intermediate == Pose((4, 5, 6, 0, 0, 0))

    def test_line_with_joint_target_raises(self):
        command = motion(joint_position([0.0] * 6), path_line())
        with pytest.raises(UnsupportedCommandRoutineFeature, match="Cartesian pose target"):
            actions_from_command_routine(_routine([command]))

    def test_joint_ptp_with_pose_target_raises(self):
        command = api.models.ExplicitPathMotionCommand(
            target=_inline((0, 0, 0)), path_type=api.models.PathTypeJointPTP()
        )
        with pytest.raises(UnsupportedCommandRoutineFeature, match="joint-position target"):
            actions_from_command_routine(_routine([command]))


class TestPoseReferenceResolution:
    def test_local_pose_reference_raises_without_a_pose_resolver(self):
        with pytest.raises(UnsupportedCommandRoutineFeature, match="LocalPoseReference"):
            actions_from_command_routine(_routine([move_linear("approach")]))

    def test_local_pose_reference_resolves_via_mapping(self):
        [action] = actions_from_command_routine(
            _routine([move_linear("approach")]),
            pose_resolver={"approach": Pose((1, 2, 3, 4, 5, 6))},
        )
        assert isinstance(action, Linear)
        assert action.target == Pose((1, 2, 3, 4, 5, 6))

    def test_local_pose_reference_resolves_via_callable(self):
        [action] = actions_from_command_routine(
            _routine([move_linear("approach")]),
            pose_resolver=lambda pose_id: (
                Pose((1, 2, 3, 4, 5, 6)) if pose_id == "approach" else None
            ),
        )
        assert action.target == Pose((1, 2, 3, 4, 5, 6))

    def test_local_pose_reference_missing_from_mapping_raises(self):
        with pytest.raises(UnsupportedCommandRoutineFeature, match="no entry"):
            actions_from_command_routine(
                _routine([move_linear("approach")]),
                pose_resolver={"other": Pose((0, 0, 0, 0, 0, 0))},
            )

    def test_dataset_pose_reference_without_resolved_pose_raises(self):
        ref = api.models.DatasetPoseReference(cell="cell", dataset="dataset", dataset_pose="pose-1")
        command = motion(ref, path_line())
        with pytest.raises(UnsupportedCommandRoutineFeature, match="resolved_pose"):
            actions_from_command_routine(_routine([command]))

    def test_dataset_pose_reference_with_resolved_pose_converts(self):
        ref = api.models.DatasetPoseReference(
            cell="cell",
            dataset="dataset",
            dataset_pose="pose-1",
            resolved_pose=api.models.ConfiguredPose(
                pose=api.models.Pose(position=(1, 2, 3), orientation=(4, 5, 6))
            ),
        )
        command = motion(ref, path_line())
        [action] = actions_from_command_routine(_routine([command]))
        assert isinstance(action, Linear)
        assert action.target == Pose((1, 2, 3, 4, 5, 6))

    def test_dataset_pose_reference_with_coordinate_system_raises(self):
        ref = api.models.DatasetPoseReference(
            cell="cell",
            dataset="dataset",
            dataset_pose="pose-1",
            resolved_pose=api.models.ConfiguredPose(
                pose=api.models.Pose(position=(1, 2, 3), orientation=(4, 5, 6)),
                coordinate_system="table",
            ),
        )
        command = motion(ref, path_line())
        with pytest.raises(UnsupportedCommandRoutineFeature, match="coordinate_system"):
            actions_from_command_routine(_routine([command]))

    def test_inline_pose_with_coordinate_system_raises(self):
        ref = api.models.InlinePoseReference(
            pose=api.models.Pose(position=(1, 2, 3), orientation=(0, 0, 0)),
            coordinate_system="table",
        )
        command = motion(ref, path_line())
        with pytest.raises(UnsupportedCommandRoutineFeature, match="coordinate_system"):
            actions_from_command_routine(_routine([command]))

    def test_joint_position_reference_converts(self):
        joints = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        command = motion(joint_position(joints), api.models.PathTypeJointPTP())
        [action] = actions_from_command_routine(_routine([command]))
        assert action.target == tuple(joints)


class TestGeneratedMotionCommand:
    def test_generated_motion_converts_to_collision_free_motion(self):
        joints = [0.0] * 6
        command = generated_motion(
            joint_position(joints),
            api.models.MotionGenerator(algorithm=api.models.RRTConnectAlgorithm()),
        )
        [action] = actions_from_command_routine(_routine([command]))
        assert isinstance(action, CollisionFreeMotion)
        assert action.target == tuple(joints)

    def test_generated_motion_with_direction_constraint_raises(self):
        joints = [0.0] * 6
        command = generated_motion(
            joint_position(joints),
            api.models.MotionGenerator(
                algorithm=api.models.RRTConnectAlgorithm(),
                constraint=api.models.DirectionConstraint(
                    world=(0.0, 0.0, 1.0), tcp=(0.0, 0.0, 1.0), tolerance=0.1
                ),
            ),
        )
        with pytest.raises(UnsupportedCommandRoutineFeature, match="constraint"):
            actions_from_command_routine(_routine([command]))


class TestOverlayCommands:
    def test_set_io_boolean_converts_to_write_action(self):
        [action] = actions_from_command_routine(_routine([set_io("digital_out[0]", True)]))
        assert isinstance(action, WriteAction)
        assert action.key == "digital_out[0]"
        assert action.value is True

    def test_set_io_integer_round_trips_without_precision_loss(self):
        [action] = actions_from_command_routine(_routine([set_io("register[0]", 42)]))
        assert action.value == 42
        assert isinstance(action.value, int)

    def test_set_io_float_converts(self):
        [action] = actions_from_command_routine(_routine([set_io("analog_out[0]", 1.5)]))
        assert action.value == 1.5

    def test_wait_for_time_converts_to_wait_action(self):
        [action] = actions_from_command_routine(_routine([wait_for_time(500)]))
        assert isinstance(action, WaitAction)
        assert action.wait_for_in_seconds == 0.5

    def test_set_io_with_at_trigger_raises(self):
        with pytest.raises(UnsupportedCommandRoutineFeature, match="at triggers"):
            actions_from_command_routine(_routine([set_io("out", True, at=at_path_fraction(0.5))]))

    def test_wait_for_io_raises(self):
        condition = api.models.IOConditionExpression(
            io=api.models.IOBooleanValue(io="digital_in[0]", value=True),
            comparator=api.models.Comparator.COMPARATOR_EQUALS,
        )
        with pytest.raises(UnsupportedCommandRoutineFeature, match="WaitForIOCommand"):
            actions_from_command_routine(_routine([wait_for_io(condition)]))

    def test_marker_raises(self):
        with pytest.raises(UnsupportedCommandRoutineFeature, match="MarkerCommand"):
            actions_from_command_routine(_routine([marker("grasp")]))


class TestDefaultMotionSettings:
    def test_default_motion_settings_applied_when_command_has_none(self):
        default = api.models.MotionSettings(
            limits_override=api.models.LimitsOverride(tcp_velocity_limit=250.0)
        )
        command = motion(_inline((0, 0, 0)), path_line())
        [action] = actions_from_command_routine(
            _routine([command], default_motion_settings=default)
        )
        assert action.settings.tcp_velocity_limit == 250.0

    def test_own_motion_settings_take_precedence_over_default(self):
        default = api.models.MotionSettings(
            limits_override=api.models.LimitsOverride(tcp_velocity_limit=250.0)
        )
        own = api.models.MotionSettings(
            limits_override=api.models.LimitsOverride(tcp_velocity_limit=42.0)
        )
        command = motion(_inline((0, 0, 0)), path_line(), settings=own)
        [action] = actions_from_command_routine(
            _routine([command], default_motion_settings=default)
        )
        assert action.settings.tcp_velocity_limit == 42.0


class TestMotionSettingsConversion:
    def test_blending_auto_converts(self):
        settings = api.models.MotionSettings(
            blending=api.models.BlendingAuto(min_velocity_in_percent=80)
        )
        command = motion(_inline((0, 0, 0)), path_line(), settings=settings)
        [action] = actions_from_command_routine(_routine([command]))
        assert action.settings.blending_auto == 80

    def test_blending_position_radius_converts(self):
        settings = api.models.MotionSettings(
            blending=api.models.BlendingPosition(position_zone_radius=5.0)
        )
        command = motion(_inline((0, 0, 0)), path_line(), settings=settings)
        [action] = actions_from_command_routine(_routine([command]))
        assert action.settings.blending_radius == 5.0

    def test_blending_position_with_unsupported_zone_field_raises(self):
        settings = api.models.MotionSettings(
            blending=api.models.BlendingPosition(orientation_zone_radius=0.1)
        )
        command = motion(_inline((0, 0, 0)), path_line(), settings=settings)
        with pytest.raises(UnsupportedCommandRoutineFeature, match="blending-zone"):
            actions_from_command_routine(_routine([command]))

    def test_limits_override_round_trips(self):
        limits = api.models.LimitsOverride(
            tcp_velocity_limit=100.0,
            tcp_acceleration_limit=200.0,
            joint_velocity_limits=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        )
        settings = api.models.MotionSettings(limits_override=limits)
        command = motion(_inline((0, 0, 0)), path_line(), settings=settings)
        [action] = actions_from_command_routine(_routine([command]))
        assert action.settings.tcp_velocity_limit == 100.0
        assert action.settings.tcp_acceleration_limit == 200.0
        assert action.settings.joint_velocity_limits == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    def test_no_motion_settings_uses_action_defaults(self):
        from nova.types import MotionSettings

        command = motion(_inline((0, 0, 0)), path_line())
        [action] = actions_from_command_routine(_routine([command]))
        assert action.settings == MotionSettings()
