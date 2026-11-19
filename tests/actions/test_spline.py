import pytest

from nova import api
from nova.actions import CombinedActions, after_start, at_path_fraction, before_target, io_write
from nova.actions.base import Action
from nova.actions.motions import Spline, cartesian_ptp, spline
from nova.actions.path_trigger_resolver import resolve_set_outputs
from nova.types import Pose


class TestSplineFactory:
    def test_converts_vectors_to_poses(self):
        action = spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0))
        assert action.start == Pose((0, 0, 0, 0, 0, 0))
        assert action.via == (Pose((3, 4, 0, 0, 0, 0)),)
        assert action.target == Pose((3, 4, 12, 0, 0, 0))

    def test_existing_pose_passed_through(self):
        target = Pose((1, 2, 3, 4, 5, 6))
        assert spline(target, start=(0, 0, 0)).target is target

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError):
            spline((1, 2, 3, 4), start=(0, 0, 0))

    def test_dict_roundtrip(self):
        action = spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0), parameters=(0, 5, 17))
        assert Action.from_dict(action.model_dump()) == action


class TestSplineToApiModel:
    def test_poses_in_order_start_via_target(self):
        model = spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0)).to_api_model()
        assert isinstance(model, api.models.PathCubicSpline)
        assert model.path_definition_name == "PathCubicSpline"
        assert [p.pose.position for p in model.parameters] == [(0, 0, 0), (3, 4, 0), (3, 4, 12)]

    def test_default_path_parameter_is_chord_length(self):
        model = spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0)).to_api_model()
        assert [p.path_parameter for p in model.parameters] == pytest.approx([0, 5, 17])

    def test_default_path_parameter_rises_where_positions_coincide(self):
        model = spline((1, 0, 0, 0, 0, 0), via=[(0, 0, 0, 0, 0, 1)], start=(0, 0, 0)).to_api_model()
        values = [p.path_parameter for p in model.parameters]
        assert all(b > a for a, b in zip(values, values[1:]))

    def test_explicit_path_parameters_are_used(self):
        action = spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0), parameters=(0, 1, 2))
        assert [p.path_parameter for p in action.to_api_model().parameters] == [0, 1, 2]


class TestSplineValidation:
    @pytest.mark.parametrize("parameters", [(0, 1), (0, 1, 2, 3)])
    def test_one_path_parameter_per_pose(self, parameters):
        with pytest.raises(ValueError, match="one path parameter per pose"):
            spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0), parameters=parameters)

    @pytest.mark.parametrize("parameters", [(0, 1, 1), (0, 2, 1)])
    def test_path_parameters_rise_strictly(self, parameters):
        with pytest.raises(ValueError, match="strictly rising"):
            spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0), parameters=parameters)

    def test_model_validates_too(self):
        with pytest.raises(ValueError):
            Spline(target=Pose((1, 0, 0)), start=Pose((0, 0, 0)), parameters=(0, 0))


class TestSplineInActionList:
    def test_motion_command_carries_the_whole_spline(self):
        actions = CombinedActions(
            items=(cartesian_ptp((0, 0, 0)), spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0)))
        )
        commands = actions.to_motion_command()
        assert len(commands) == 2
        assert isinstance(commands[1].path, api.models.PathCubicSpline)
        assert len(commands[1].path.parameters) == 3

    def test_spline_is_one_motion_for_io_writes(self):
        actions = CombinedActions(
            items=(
                io_write("a", True),
                spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0)),
                io_write("b", True),
            )
        )
        assert len(actions.motions) == 1
        assert [a.path_parameter for a in actions.actions] == [0, 1]


class TestSplinePathTriggers:
    """A spline is one motion, so path triggers measure against the whole spline."""

    # Planned trajectory of one spline: location 0 -> 1 over 4 s and 200 mm.
    TIMES = [0.0, 1.0, 2.0, 3.0, 4.0]
    LOCATIONS = [0.0, 0.25, 0.5, 0.75, 1.0]
    POSITIONS = [(0.0, 0.0, 0.0), (50.0, 0, 0), (100.0, 0, 0), (150.0, 0, 0), (200.0, 0, 0)]

    def _resolve(self, trigger):
        actions = CombinedActions(
            items=(
                io_write("a", True, at=trigger),
                spline((3, 4, 12), via=[(3, 4, 0)], start=(0, 0, 0)),
            )
        )
        outputs = resolve_set_outputs(actions, self.TIMES, self.LOCATIONS, self.POSITIONS)
        return outputs[0].location

    def test_path_fraction(self):
        assert self._resolve(at_path_fraction(0.5)) == pytest.approx(0.5)

    def test_after_start_seconds(self):
        assert self._resolve(after_start(seconds=1.0)) == pytest.approx(0.25)

    def test_before_target_millimeters(self):
        assert self._resolve(before_target(millimeters=50)) == pytest.approx(0.75)
