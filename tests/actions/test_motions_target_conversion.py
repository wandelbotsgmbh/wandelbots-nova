import pytest

from nova.actions.motions import cartesian_ptp, circular, linear, spline
from nova.types import Pose


class TestMotionTargetConversion:
    """Motion factory functions convert PoseOrSequence targets via the Pose constructor."""

    def test_linear_six_tuple(self):
        assert linear((1, 2, 3, 4, 5, 6)).target == Pose((1, 2, 3, 4, 5, 6))

    def test_linear_three_tuple_defaults_orientation(self):
        assert linear((1, 2, 3)).target == Pose((1, 2, 3, 0, 0, 0))

    def test_linear_existing_pose_passed_through(self):
        original = Pose((1, 2, 3, 4, 5, 6))
        assert linear(original).target is original

    def test_linear_wrong_length_raises(self):
        with pytest.raises(ValueError):
            linear((1, 2, 3, 4))

    def test_cartesian_ptp_existing_pose_passed_through(self):
        original = Pose((1, 2, 3, 4, 5, 6))
        assert cartesian_ptp(original).target is original

    def test_cartesian_ptp_wrong_length_raises(self):
        with pytest.raises(ValueError):
            cartesian_ptp((1, 2, 3, 4))

    def test_circular_existing_poses_passed_through(self):
        target = Pose((1, 2, 3, 4, 5, 6))
        intermediate = Pose((7, 8, 9, 10, 11, 12))
        action = circular(target, intermediate)
        assert action.target is target
        assert action.intermediate is intermediate

    def test_circular_wrong_length_raises(self):
        with pytest.raises(ValueError):
            circular((1, 2, 3, 4), (7, 8, 9, 10, 11, 12))

    def test_spline_existing_pose_passed_through(self):
        original = Pose((1, 2, 3, 4, 5, 6))
        assert spline(original).target is original

    def test_spline_wrong_length_raises(self):
        with pytest.raises(ValueError):
            spline((1, 2, 3, 4))


class TestMotionFactoriesAcceptLists:
    """List inputs should behave the same as the equivalent tuple inputs.

    Note: we compare `.target`/`.intermediate` rather than full object equality,
    since the factory functions attach caller source-location metadata (`metas`)
    that differs between call sites.
    """

    def test_linear_six_list(self):
        assert linear([1, 2, 3, 4, 5, 6]).target == Pose((1, 2, 3, 4, 5, 6))

    def test_linear_three_list_defaults_orientation(self):
        assert linear([1, 2, 3]).target == Pose((1, 2, 3, 0, 0, 0))

    def test_cartesian_ptp_six_list(self):
        assert cartesian_ptp([1, 2, 3, 4, 5, 6]).target == Pose((1, 2, 3, 4, 5, 6))

    def test_cartesian_ptp_three_list_defaults_orientation(self):
        assert cartesian_ptp([1, 2, 3]).target == Pose((1, 2, 3, 0, 0, 0))

    def test_circular_six_lists(self):
        action = circular([1, 2, 3, 4, 5, 6], [7, 8, 9, 10, 11, 12])
        assert action.target == Pose((1, 2, 3, 4, 5, 6))
        assert action.intermediate == Pose((7, 8, 9, 10, 11, 12))

    def test_circular_three_lists_default_orientation(self):
        action = circular([1, 2, 3], [4, 5, 6])
        assert action.target == Pose((1, 2, 3, 0, 0, 0))
        assert action.intermediate == Pose((4, 5, 6, 0, 0, 0))

    def test_spline_six_list(self):
        assert spline([1, 2, 3, 4, 5, 6]).target == Pose((1, 2, 3, 4, 5, 6))

    def test_spline_three_list_defaults_orientation(self):
        assert spline([1, 2, 3]).target == Pose((1, 2, 3, 0, 0, 0))
