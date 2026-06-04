import pytest

from nova import api
from nova.types import Pose


@pytest.fixture
def kinematic_config():
    return api.models.KinematicConfiguration(
        kinematic_branch=api.models.KinematicBranch(
            shoulder_branch="FRONT", elbow_branch="UP", wrist_branch="NO_FLIP"
        )
    )


@pytest.fixture
def pose_with_config(kinematic_config):
    return Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kinematic_config)


class TestPoseKinematicConfiguration:
    def test_eq_same_config(self, kinematic_config):
        p1 = Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kinematic_config)
        p2 = Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kinematic_config)
        assert p1 == p2

    def test_eq_different_config(self, kinematic_config):
        other_config = api.models.KinematicConfiguration(
            kinematic_branch=api.models.KinematicBranch(
                shoulder_branch="BACK", elbow_branch="DOWN", wrist_branch="FLIP"
            )
        )
        p1 = Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=kinematic_config)
        p2 = Pose((1, 2, 3, 4, 5, 6), kinematic_configuration=other_config)
        assert p1 != p2

    def test_eq_with_config_vs_without(self, pose_with_config):
        p_no_config = Pose((1, 2, 3, 4, 5, 6))
        assert pose_with_config != p_no_config

    def test_matmul_discards_config(self, pose_with_config):
        result = pose_with_config @ (0, 0, 0, 0, 0, 0)
        assert result.kinematic_configuration is None

    def test_to_api_model_excludes_config(self, pose_with_config):
        api_model = pose_with_config.to_api_model()
        assert not hasattr(api_model, "kinematic_configuration")
