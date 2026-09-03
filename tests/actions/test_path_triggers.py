"""Unit tests for path triggers ("Bahnschaltpunkte") and their resolution."""

import math

import pytest
from pydantic import ValidationError

from nova.actions import (
    AtReference,
    DistanceTrigger,
    PathFractionTrigger,
    TimeTrigger,
    after_distance,
    after_time,
    at_distance,
    at_path_fraction,
    at_time,
    before_distance,
    before_time,
    io_write,
)
from nova.actions.container import CombinedActions
from nova.actions.io import WriteAction
from nova.actions.motions import linear
from nova.actions.path_trigger_resolver import (
    has_distance_triggers,
    has_path_triggers,
    resolve_set_outputs,
)

# Synthetic planned trajectory spanning motion-index locations 0 -> 2.
# 5 samples, location 1 reached at time 2.0 / arc length 100 mm,
# location 2 reached at time 4.0 / arc length 200 mm.
TIMES = [0.0, 1.0, 2.0, 3.0, 4.0]
LOCATIONS = [0.0, 0.5, 1.0, 1.5, 2.0]
POSITIONS = [
    (0.0, 0.0, 0.0),
    (50.0, 0.0, 0.0),
    (100.0, 0.0, 0.0),
    (150.0, 0.0, 0.0),
    (200.0, 0.0, 0.0),
]


def _combined(*actions) -> CombinedActions:
    return CombinedActions(items=tuple(actions))


def _locations(set_outputs) -> list[float]:
    return [entry.location for entry in set_outputs]


class TestTriggerBuilders:
    def test_at_path_fraction(self):
        trig = at_path_fraction(0.5)
        assert isinstance(trig, PathFractionTrigger)
        assert trig.value == 0.5

    def test_after_time_is_previous(self):
        trig = after_time(0.5)
        assert isinstance(trig, TimeTrigger)
        assert trig.seconds == 0.5
        assert trig.reference is AtReference.PREVIOUS
        assert trig == at_time(0.5, AtReference.PREVIOUS)

    def test_before_time_is_next(self):
        assert before_time(0.5) == at_time(0.5, AtReference.NEXT)

    def test_after_distance_is_previous(self):
        trig = after_distance(100)
        assert isinstance(trig, DistanceTrigger)
        assert trig.millimeters == 100
        assert trig.reference is AtReference.PREVIOUS
        assert trig == at_distance(100, AtReference.PREVIOUS)

    def test_before_distance_is_next(self):
        assert before_distance(50) == at_distance(50, AtReference.NEXT)

    def test_negative_values_rejected(self):
        with pytest.raises(ValidationError):
            after_time(-1)
        with pytest.raises(ValidationError):
            after_distance(-1)
        with pytest.raises(ValidationError):
            at_path_fraction(-0.1)

    def test_path_fraction_is_half_open(self):
        # The command-routine API defines value in [0, 1): 1.0 is "the next motion", which
        # is expressed by placing the write after that motion without a trigger.
        at_path_fraction(0.0)
        with pytest.raises(ValidationError):
            at_path_fraction(1.0)


class TestIoWriteTrigger:
    def test_io_write_without_trigger(self):
        assert io_write("relay", True).at is None

    def test_io_write_with_trigger(self):
        action = io_write("relay", True, at=after_time(0.5))
        assert isinstance(action.at, TimeTrigger)

    def test_trigger_round_trips_through_serialization(self):
        action = io_write("relay", True, at=before_distance(25))
        restored = WriteAction.model_validate_json(action.model_dump_json())
        assert restored.at == action.at
        assert restored == action

    def test_trigger_uses_the_command_routine_wire_format(self):
        action = io_write("relay", True, at=after_distance(25))
        assert action.model_dump(mode="json", exclude_none=True)["at"] == {
            "type": "distance",
            "millimeters": 25.0,
            "reference": "previous",
        }


class TestTriggerPredicates:
    def test_no_triggers(self):
        actions = _combined(linear((1, 2, 3)), io_write("a", True), linear((4, 5, 6)))
        assert not has_path_triggers(actions)
        assert not has_distance_triggers(actions)

    def test_time_trigger_is_not_a_distance_trigger(self):
        actions = _combined(linear((1, 2, 3)), io_write("a", True, at=after_time(1)))
        assert has_path_triggers(actions)
        assert not has_distance_triggers(actions)

    def test_distance_trigger(self):
        actions = _combined(linear((1, 2, 3)), io_write("a", True, at=after_distance(1)))
        assert has_path_triggers(actions)
        assert has_distance_triggers(actions)


class TestResolveSetOutputs:
    def test_no_triggers_matches_to_set_io(self):
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True), linear((4, 5, 6)), io_write("b", False)
        )
        assert resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS) == actions.to_set_io()

    def test_path_fraction_trigger_is_anchored(self):
        # write action is anchored to motion segment [1, 2]; value 0.25 -> 1.25
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=at_path_fraction(0.25)), linear((4, 5, 6))
        )
        assert _locations(resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS)) == [1.25]

    def test_time_trigger_previous(self):
        # anchor = motion 1 (reached at t=2.0); +1s -> t=3.0 -> location 1.5
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=after_time(1.0)), linear((4, 5, 6))
        )
        (location,) = _locations(resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS))
        assert math.isclose(location, 1.5)

    def test_time_trigger_next(self):
        # next = motion 2 (reached at t=4.0); -1s -> t=3.0 -> location 1.5
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=before_time(1.0)), linear((4, 5, 6))
        )
        (location,) = _locations(resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS))
        assert math.isclose(location, 1.5)

    def test_time_trigger_overshoot_clamped_to_segment(self, caplog):
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=after_time(100.0)), linear((4, 5, 6))
        )
        with caplog.at_level("WARNING"):
            (location,) = _locations(resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS))
        assert math.isclose(location, 2.0)
        assert "outside its motion segment" in caplog.text

    def test_distance_trigger_previous(self):
        # anchor arc length = 100 mm; +25 mm -> 125 mm -> location 1.25
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=after_distance(25)), linear((4, 5, 6))
        )
        (location,) = _locations(resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS))
        assert math.isclose(location, 1.25)

    def test_distance_trigger_next(self):
        # next arc length = 200 mm; -25 mm -> 175 mm -> location 1.75
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=before_distance(25)), linear((4, 5, 6))
        )
        (location,) = _locations(resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS))
        assert math.isclose(location, 1.75)

    def test_distance_trigger_overshoot_clamped_to_segment(self, caplog):
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=before_distance(1000)), linear((4, 5, 6))
        )
        with caplog.at_level("WARNING"):
            (location,) = _locations(resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS))
        assert math.isclose(location, 1.0)
        assert "outside its motion segment" in caplog.text

    def test_distance_trigger_without_positions_falls_back_to_anchor(self, caplog):
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=after_distance(25)), linear((4, 5, 6))
        )
        with caplog.at_level("WARNING"):
            set_outputs = resolve_set_outputs(actions, TIMES, LOCATIONS, tcp_positions=None)
        assert _locations(set_outputs) == [1.0]
        assert "no TCP positions" in caplog.text

    def test_trigger_after_last_motion_collapses_to_end(self, caplog):
        actions = _combined(
            linear((1, 2, 3)), linear((4, 5, 6)), io_write("a", True, at=at_path_fraction(0.5))
        )
        with caplog.at_level("WARNING"):
            set_outputs = resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS)
        assert _locations(set_outputs) == [2.0]
        assert "no following segment" in caplog.text

    def test_mixed_triggered_and_untriggered_keep_order(self):
        actions = _combined(
            io_write("start", False),
            linear((1, 2, 3)),
            io_write("a", True, at=after_time(1.0)),
            io_write("b", True, at=at_path_fraction(0.75)),
            linear((4, 5, 6)),
            io_write("end", False),
        )
        set_outputs = resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS)
        assert [entry.io.io for entry in set_outputs] == ["start", "a", "b", "end"]
        assert _locations(set_outputs) == [0.0, 1.5, 1.75, 2.0]
        # Untriggered entries are identical to the plain overlay.
        plain = actions.to_set_io()
        assert set_outputs[0] == plain[0]
        assert set_outputs[3] == plain[3]

    def test_io_origin_and_value_are_preserved(self):
        from nova import api

        actions = _combined(
            linear((1, 2, 3)),
            io_write("bus", 3, origin=api.models.IOOrigin.BUS_IO, at=at_path_fraction(0.5)),
            linear((4, 5, 6)),
        )
        (entry,) = resolve_set_outputs(actions, TIMES, LOCATIONS, POSITIONS)
        assert entry.io_origin is api.models.IOOrigin.BUS_IO
        assert entry.io.value == "3"
        assert entry.location == 1.5
