import pytest

from nova import api
from nova.types import Pose, Vector3d


@pytest.fixture
def kinematic_config():
    return api.models.KinematicConfiguration(
        kinematic_branch=api.models.KinematicBranch(
            shoulder_branch="FRONT", elbow_branch="UP", wrist_branch="NO_FLIP"
        )
    )


class TestPoseInitAllowed:
    """Constructor forms that are currently supported."""

    def test_none_arg_gives_identity(self):
        p = Pose(None)
        assert p.to_tuple() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert p.kinematic_configuration is None

    def test_native_kwargs(self):
        p = Pose(position=Vector3d(x=10, y=20, z=30), orientation=Vector3d(x=1, y=2, z=3))
        assert p.to_tuple() == (10, 20, 30, 1, 2, 3)
        assert p.kinematic_configuration is None

    def test_six_tuple(self):
        p = Pose((1, 2, 3, 4, 5, 6))
        assert p.to_tuple() == (1, 2, 3, 4, 5, 6)

    def test_three_tuple_defaults_orientation(self):
        p = Pose((1, 2, 3))
        assert p.to_tuple() == (1, 2, 3, 0, 0, 0)

    def test_six_list(self):
        p = Pose([1, 2, 3, 4, 5, 6])
        assert p.to_tuple() == (1, 2, 3, 4, 5, 6)

    def test_three_list_defaults_orientation(self):
        p = Pose([1, 2, 3])
        assert p.to_tuple() == (1, 2, 3, 0, 0, 0)

    def test_six_positional_args(self):
        p = Pose(1, 2, 3, 4, 5, 6)
        assert p.to_tuple() == (1, 2, 3, 4, 5, 6)

    def test_three_positional_args_defaults_orientation(self):
        p = Pose(1, 2, 3)
        assert p.to_tuple() == (1, 2, 3, 0, 0, 0)

    def test_from_api_model(self):
        api_pose = api.models.Pose(
            position=api.models.Vector3d([1, 2, 3]),
            orientation=api.models.RotationVector([4, 5, 6]),
        )
        p = Pose(api_pose)
        assert p.to_tuple() == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    def test_from_api_model_with_none_fields_gives_identity(self):
        p = Pose(api.models.Pose(position=None, orientation=None))
        assert p.to_tuple() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def test_from_api_model_with_none_position_gives_zero_position(self):
        p = Pose(api.models.Pose(position=None, orientation=api.models.RotationVector([1, 2, 3])))
        assert p.to_tuple() == (0.0, 0.0, 0.0, 1.0, 2.0, 3.0)

    def test_from_api_model_with_none_orientation_gives_zero_orientation(self):
        p = Pose(api.models.Pose(position=api.models.Vector3d([1, 2, 3]), orientation=None))
        assert p.to_tuple() == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)

    def test_kinematic_configuration_kwarg_preserved(self, kinematic_config):
        p = Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kinematic_config)
        assert p.to_tuple() == (1, 2, 3, 4, 5, 6)
        assert p.kinematic_configuration == kinematic_config

    def test_from_dataset_pose(self, kinematic_config):
        dataset_pose = api.models.DatasetPose(
            id="p1",
            pose=api.models.Pose(
                position=api.models.Vector3d([1, 2, 3]),
                orientation=api.models.RotationVector([4, 5, 6]),
            ),
            kinematic_configuration=kinematic_config,
        )
        p = Pose(dataset_pose)
        assert p.to_tuple() == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        assert p.kinematic_configuration == kinematic_config

    def test_from_dataset_pose_position_none(self, kinematic_config):
        dataset_pose = api.models.DatasetPose(
            id="p1",
            pose=api.models.Pose(position=None, orientation=api.models.RotationVector([4, 5, 6])),
            kinematic_configuration=kinematic_config,
        )
        p = Pose(dataset_pose)
        assert p.to_tuple() == (0.0, 0.0, 0.0, 4.0, 5.0, 6.0)
        assert p.kinematic_configuration == kinematic_config

    def test_from_dataset_pose_orientation_none(self, kinematic_config):
        dataset_pose = api.models.DatasetPose(
            id="p1",
            pose=api.models.Pose(position=api.models.Vector3d([1, 2, 3]), orientation=None),
            kinematic_configuration=kinematic_config,
        )
        p = Pose(dataset_pose)
        assert p.to_tuple() == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
        assert p.kinematic_configuration == kinematic_config

    def test_from_dataset_pose_position_and_orientation_none(self, kinematic_config):
        dataset_pose = api.models.DatasetPose(
            id="p1",
            pose=api.models.Pose(position=None, orientation=None),
            kinematic_configuration=kinematic_config,
        )
        p = Pose(dataset_pose)
        assert p.to_tuple() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert p.kinematic_configuration == kinematic_config

    def test_from_dataset_pose_without_kinematic_configuration(self):
        dataset_pose = api.models.DatasetPose(
            id="p2",
            pose=api.models.Pose(
                position=api.models.Vector3d([1, 2, 3]),
                orientation=api.models.RotationVector([4, 5, 6]),
            ),
        )
        p = Pose(dataset_pose)
        assert p.to_tuple() == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        assert p.kinematic_configuration is None

    def test_from_existing_pose_copies_values(self):
        original = Pose((1, 2, 3, 4, 5, 6))
        p = Pose(original)
        assert p == original
        assert p.kinematic_configuration is None

    def test_from_existing_pose_with_kinematic_configuration(self, kinematic_config):
        original = Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kinematic_config)
        p = Pose(original)
        assert p.to_tuple() == (1, 2, 3, 4, 5, 6)
        assert p.kinematic_configuration == kinematic_config

    def test_from_existing_pose_without_kinematic_configuration_kwarg_applied(
        self, kinematic_config
    ):
        original = Pose((1, 2, 3, 4, 5, 6))
        p = Pose(original, kinematic_configuration=kinematic_config)
        assert p.to_tuple() == (1, 2, 3, 4, 5, 6)
        assert p.kinematic_configuration == kinematic_config


class TestPoseInitForbidden:
    """Constructor forms that are currently rejected."""

    def test_no_args_raises(self):
        with pytest.raises(KeyError):
            Pose()

    def test_none_keyword_args_raises(self):
        with pytest.raises(TypeError):
            Pose(position=None, orientation=None)

    def test_single_scalar_arg_raises(self):
        with pytest.raises(ValueError):
            Pose(42)

    def test_four_positional_args_raise(self):
        with pytest.raises(ValueError):
            Pose(1, 2, 3, 4)

    def test_seven_positional_args_raise(self):
        with pytest.raises(ValueError):
            Pose(1, 2, 3, 4, 5, 6, 7)

    def test_four_element_list_raises(self):
        with pytest.raises(ValueError):
            Pose([1, 2, 3, 4])

    def test_string_arg_raises(self):
        with pytest.raises(ValueError):
            Pose("abcdef")

    def test_existing_pose_with_double_kinematic_configuration_raises(self, kinematic_config):
        original = Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kinematic_config)
        with pytest.raises(ValueError):
            Pose(original, kinematic_configuration=kinematic_config)

    def test_dataset_pose_with_double_kinematic_configuration_raises(self, kinematic_config):
        dataset_pose = api.models.DatasetPose(
            id="p1",
            pose=api.models.Pose(
                position=api.models.Vector3d([1, 2, 3]),
                orientation=api.models.RotationVector([4, 5, 6]),
            ),
            kinematic_configuration=kinematic_config,
        )
        with pytest.raises(ValueError):
            Pose(dataset_pose, kinematic_configuration=kinematic_config)
