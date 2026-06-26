"""Unit tests for path triggers ("Bahnschaltpunkte") and their resolution."""

import math

import pytest

from nova.actions import (
    DistanceTrigger,
    PathParameterTrigger,
    TimeTrigger,
    TriggerReference,
    after_distance,
    after_time,
    at_path,
    before_distance,
    before_time,
    io_write,
)
from nova.actions.container import CombinedActions
from nova.actions.motions import linear
from nova.actions.path_trigger_resolver import resolve_trigger_locations

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


class TestTriggerConstructors:
    def test_at_path(self):
        trig = at_path(0.5)
        assert isinstance(trig, PathParameterTrigger)
        assert trig.value == 0.5

    def test_after_time_is_previous(self):
        trig = after_time(0.5)
        assert isinstance(trig, TimeTrigger)
        assert trig.seconds == 0.5
        assert trig.reference is TriggerReference.PREVIOUS

    def test_before_time_is_next(self):
        trig = before_time(0.5)
        assert trig.reference is TriggerReference.NEXT

    def test_after_distance_is_previous(self):
        trig = after_distance(100)
        assert isinstance(trig, DistanceTrigger)
        assert trig.millimeters == 100
        assert trig.reference is TriggerReference.PREVIOUS

    def test_before_distance_is_next(self):
        trig = before_distance(50)
        assert trig.reference is TriggerReference.NEXT

    def test_negative_value_rejected(self):
        with pytest.raises(ValueError):
            after_time(-1)
        with pytest.raises(ValueError):
            after_distance(-1)


class TestIoWriteTrigger:
    def test_io_write_without_trigger(self):
        action = io_write("relay", True)
        assert action.trigger is None

    def test_io_write_with_trigger(self):
        action = io_write("relay", True, at=after_time(0.5))
        assert isinstance(action.trigger, TimeTrigger)


class TestResolveTriggerLocations:
    def test_no_triggers_returns_empty(self):
        actions = _combined(linear((1, 2, 3)), io_write("a", True), linear((4, 5, 6)))
        assert resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS) == {}

    def test_path_parameter_trigger_is_anchored_fraction(self):
        # write action is anchored to motion segment [1, 2]; value 0.25 -> 1.25
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=at_path(0.25)), linear((4, 5, 6))
        )
        overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS)
        assert overrides == {0: 1.25}

    def test_path_parameter_trigger_overshoot_clamped(self, caplog):
        # value 1.5 overshoots the anchor segment [1, 2] -> clamped to 2.0 with a warning
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=at_path(1.5)), linear((4, 5, 6))
        )
        with caplog.at_level("WARNING"):
            overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS)
        assert math.isclose(overrides[0], 2.0)
        assert "outside its motion segment" in caplog.text

    def test_time_trigger_previous(self):
        # anchor = motion 1 (reached at t=2.0); +1s -> t=3.0 -> location 1.5
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=after_time(1.0)), linear((4, 5, 6))
        )
        overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS)
        assert math.isclose(overrides[0], 1.5)

    def test_time_trigger_next(self):
        # next = motion 2 (reached at t=4.0); -1s -> t=3.0 -> location 1.5
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=before_time(1.0)), linear((4, 5, 6))
        )
        overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS)
        assert math.isclose(overrides[0], 1.5)

    def test_time_trigger_overshoot_clamped_to_segment(self, caplog):
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=after_time(100.0)), linear((4, 5, 6))
        )
        with caplog.at_level("WARNING"):
            overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS)
        assert math.isclose(overrides[0], 2.0)
        assert "outside its motion segment" in caplog.text

    def test_distance_trigger_previous(self):
        # anchor arc length = 100 mm; +25 mm -> 125 mm -> location 1.25
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=after_distance(25)), linear((4, 5, 6))
        )
        overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS)
        assert math.isclose(overrides[0], 1.25)

    def test_distance_trigger_next(self):
        # next arc length = 200 mm; -25 mm -> 175 mm -> location 1.75
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=before_distance(25)), linear((4, 5, 6))
        )
        overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS)
        assert math.isclose(overrides[0], 1.75)

    def test_distance_trigger_without_positions_falls_back(self):
        actions = _combined(
            linear((1, 2, 3)), io_write("a", True, at=after_distance(25)), linear((4, 5, 6))
        )
        overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, tcp_positions=None)
        assert overrides == {}

    def test_multiple_write_actions_indexing(self):
        actions = _combined(
            linear((1, 2, 3)),
            io_write("a", True, at=after_time(1.0)),
            linear((4, 5, 6)),
            io_write("b", True),
        )
        overrides = resolve_trigger_locations(actions, TIMES, LOCATIONS, POSITIONS)
        # only the first (index 0) write carries a trigger
        assert set(overrides.keys()) == {0}


class TestToSetIoOverrides:
    def test_overrides_applied_and_default_kept(self):
        actions = _combined(
            linear((1, 2, 3)),
            io_write("a", True, at=at_path(0.5)),
            linear((4, 5, 6)),
            io_write("b", False),
        )
        set_io = actions.to_set_io({0: 1.5})
        assert set_io[0].location == 1.5
        # second write has no override -> keeps its motion-index path parameter
        assert set_io[1].location == actions.actions[1].path_parameter

    def test_no_overrides_uses_path_parameter(self):
        actions = _combined(linear((1, 2, 3)), io_write("a", True), linear((4, 5, 6)))
        set_io = actions.to_set_io()
        assert set_io[0].location == actions.actions[0].path_parameter
