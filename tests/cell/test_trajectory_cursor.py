"""Tests for TrajectoryCursor action index and location logic."""

import pytest
from unittest.mock import MagicMock

from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor, MovementOption
from nova.actions.motions import lin
from nova.types import Pose


def create_cursor(num_actions: int, initial_location: float) -> TrajectoryCursor:
    """Helper to create a TrajectoryCursor with mock dependencies."""
    actions = [lin(Pose((i * 100, 0, 0, 0, 0, 0))) for i in range(num_actions)]

    joint_trajectory = MagicMock()
    joint_trajectory.locations = [MagicMock(root=float(i)) for i in range(num_actions + 1)]

    cursor = object.__new__(TrajectoryCursor)
    cursor.joint_trajectory = joint_trajectory
    cursor.actions = MagicMock()
    cursor.actions.__len__ = lambda _: num_actions
    cursor.actions.__getitem__ = lambda _, i: actions[i]
    cursor._current_location = initial_location
    cursor._target_location = initial_location

    return cursor


@pytest.mark.parametrize(
    "location, expected",
    [
        pytest.param(0.0, 0, id="at_trajectory_start"),
        pytest.param(0.5, 0, id="midway_through_first_action"),
        pytest.param(1.0, 1, id="at_action_boundary"),
        pytest.param(1.7, 1, id="midway_through_middle_action"),
        pytest.param(2.5, 2, id="at_last_action"),
        pytest.param(3.0, 2, id="at_trajectory_end_clamps_to_last"),
        pytest.param(5.0, 2, id="beyond_end_clamps_to_last"),
    ],
)
def test_current_action_index(location, expected):
    cursor = create_cursor(num_actions=3, initial_location=location)
    assert cursor.current_action_index == expected


@pytest.mark.parametrize(
    "num_actions, location, expected",
    [
        pytest.param(1, 0.0, 0, id="single_action_at_start"),
        pytest.param(1, 1.0, 0, id="single_action_at_end"),
    ],
)
def test_current_action_index_single_action(num_actions, location, expected):
    cursor = create_cursor(num_actions=num_actions, initial_location=location)
    assert cursor.current_action_index == expected


@pytest.mark.parametrize(
    "location, expected",
    [
        pytest.param(0.0, 0.0, id="at_trajectory_start"),
        pytest.param(0.999, 0.0, id="just_before_boundary"),
        pytest.param(1.0, 1.0, id="at_action_start"),
        pytest.param(1.7, 1.0, id="midway_through_action"),
    ],
)
def test_current_action_start(location, expected):
    cursor = create_cursor(num_actions=3, initial_location=location)
    assert cursor.current_action_start == expected


@pytest.mark.parametrize(
    "location, expected",
    [
        pytest.param(0.0, 0.0, id="at_trajectory_start"),
        pytest.param(1.0, 1.0, id="at_boundary"),
        pytest.param(1.001, 2.0, id="just_after_boundary"),
        pytest.param(1.3, 2.0, id="midway_through_action"),
    ],
)
def test_current_action_end(location, expected):
    cursor = create_cursor(num_actions=3, initial_location=location)
    assert cursor.current_action_end == expected


@pytest.mark.parametrize(
    "location, expected",
    [
        pytest.param(0.5, 1.0, id="midway_first_action"),
        pytest.param(1.0, 1.0, id="at_exact_boundary"),
        pytest.param(2.5, 3.0, id="midway_last_action"),
    ],
)
def test_next_action_start(location, expected):
    cursor = create_cursor(num_actions=3, initial_location=location)
    assert cursor.next_action_start == expected
    assert cursor.next_action_start == cursor.current_action_end


@pytest.mark.parametrize(
    "location, expected",
    [
        pytest.param(0.5, -1.0, id="from_first_action_returns_negative"),
        pytest.param(1.5, 0.0, id="from_second_action"),
        pytest.param(2.0, 1.0, id="at_exact_boundary"),
        pytest.param(2.5, 1.0, id="from_third_action"),
    ],
)
def test_previous_action_start(location, expected):
    cursor = create_cursor(num_actions=3, initial_location=location)
    assert cursor.previous_action_start == expected


@pytest.mark.parametrize(
    "num_actions, location, has_previous",
    [
        pytest.param(3, 0.0, False, id="at_trajectory_start_returns_none"),
        pytest.param(3, 0.5, False, id="in_first_action_returns_none"),
        pytest.param(3, 1.0, True, id="at_second_action_boundary"),
        pytest.param(3, 1.5, True, id="in_second_action"),
        pytest.param(3, 2.5, True, id="in_last_action"),
        pytest.param(1, 0.5, False, id="single_action_returns_none"),
    ],
)
def test_previous_action(num_actions, location, has_previous):
    cursor = create_cursor(num_actions=num_actions, initial_location=location)
    assert (cursor.previous_action is not None) == has_previous


@pytest.mark.parametrize(
    "num_actions, expected",
    [
        pytest.param(1, 1.0, id="single_action"),
        pytest.param(3, 3.0, id="three_actions"),
        pytest.param(10, 10.0, id="many_actions"),
    ],
)
def test_end_location(num_actions, expected):
    cursor = create_cursor(num_actions=num_actions, initial_location=0.0)
    assert cursor.end_location == expected


@pytest.mark.parametrize(
    "location, can_forward, can_backward",
    [
        pytest.param(0.0, True, False, id="at_start_can_only_move_forward"),
        pytest.param(0.001, True, True, id="just_after_start"),
        pytest.param(1.5, True, True, id="in_middle_can_move_both_ways"),
        pytest.param(2.999, True, True, id="just_before_end"),
        pytest.param(3.0, False, True, id="at_end_can_only_move_backward"),
    ],
)
def test_movement_options(location, can_forward, can_backward):
    cursor = create_cursor(num_actions=3, initial_location=location)
    options = cursor.get_movement_options()
    assert (MovementOption.CAN_MOVE_FORWARD in options) == can_forward
    assert (MovementOption.CAN_MOVE_BACKWARD in options) == can_backward


@pytest.mark.parametrize(
    "num_actions, location, has_next",
    [
        pytest.param(3, 0.5, True, id="from_first_action"),
        pytest.param(3, 1.5, True, id="from_second_action"),
        pytest.param(3, 2.5, False, id="at_last_action_returns_none"),
        pytest.param(3, 3.0, False, id="at_trajectory_end_returns_none"),
        pytest.param(1, 0.5, False, id="single_action_returns_none"),
    ],
)
def test_next_action(num_actions, location, has_next):
    cursor = create_cursor(num_actions=num_actions, initial_location=location)
    assert (cursor.next_action is not None) == has_next


@pytest.mark.parametrize(
    "num_actions, location",
    [
        pytest.param(3, 0.0, id="at_start"),
        pytest.param(3, 3.0, id="at_trajectory_end_returns_last"),
        pytest.param(1, 0.5, id="single_action"),
    ],
)
def test_current_action_always_returns_action(num_actions, location):
    cursor = create_cursor(num_actions=num_actions, initial_location=location)
    assert cursor.current_action is not None
