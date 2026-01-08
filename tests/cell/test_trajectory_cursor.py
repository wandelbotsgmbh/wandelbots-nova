"""Tests for TrajectoryCursor action index and location logic."""

import pytest
from unittest.mock import MagicMock

from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor, MovementOption
from nova.actions.motions import lin
from nova.types import Pose


def create_cursor(num_actions: int, initial_location: float) -> TrajectoryCursor:
    """Helper to create a TrajectoryCursor with mock dependencies."""
    actions = [lin(Pose((i * 100, 0, 0, 0, 0, 0))) for i in range(num_actions)]

    # Mock joint trajectory with locations from 0 to num_actions
    joint_trajectory = MagicMock()
    joint_trajectory.locations = [MagicMock(root=float(i)) for i in range(num_actions + 1)]

    cursor = object.__new__(TrajectoryCursor)
    cursor.joint_trajectory = joint_trajectory
    cursor.actions = MagicMock()
    cursor.actions.__len__ = lambda self: num_actions
    cursor.actions.__getitem__ = lambda self, i: actions[i]
    cursor._current_location = initial_location
    cursor._target_location = initial_location

    return cursor


class TestCurrentActionIndex:
    """Tests for current_action_index property."""

    def test_at_trajectory_start(self):
        cursor = create_cursor(num_actions=3, initial_location=0.0)
        assert cursor.current_action_index == 0

    def test_midway_through_first_action(self):
        cursor = create_cursor(num_actions=3, initial_location=0.5)
        assert cursor.current_action_index == 0

    def test_at_action_boundary(self):
        cursor = create_cursor(num_actions=3, initial_location=1.0)
        assert cursor.current_action_index == 1

    def test_midway_through_middle_action(self):
        cursor = create_cursor(num_actions=3, initial_location=1.7)
        assert cursor.current_action_index == 1

    def test_at_last_action(self):
        cursor = create_cursor(num_actions=3, initial_location=2.5)
        assert cursor.current_action_index == 2

    def test_at_trajectory_end_clamps_to_last_action(self):
        cursor = create_cursor(num_actions=3, initial_location=3.0)
        assert cursor.current_action_index == 2  # last action index

    def test_beyond_trajectory_end_clamps_to_last_action(self):
        cursor = create_cursor(num_actions=3, initial_location=5.0)
        assert cursor.current_action_index == 2

    def test_single_action_at_start(self):
        cursor = create_cursor(num_actions=1, initial_location=0.0)
        assert cursor.current_action_index == 0

    def test_single_action_at_end(self):
        cursor = create_cursor(num_actions=1, initial_location=1.0)
        assert cursor.current_action_index == 0


class TestCurrentActionStart:
    """Tests for current_action_start property."""

    def test_at_action_start(self):
        cursor = create_cursor(num_actions=3, initial_location=1.0)
        assert cursor.current_action_start == 1.0

    def test_midway_through_action(self):
        cursor = create_cursor(num_actions=3, initial_location=1.7)
        assert cursor.current_action_start == 1.0

    def test_at_trajectory_start(self):
        cursor = create_cursor(num_actions=3, initial_location=0.0)
        assert cursor.current_action_start == 0.0

    def test_just_before_boundary(self):
        cursor = create_cursor(num_actions=3, initial_location=0.999)
        assert cursor.current_action_start == 0.0


class TestCurrentActionEnd:
    """Tests for current_action_end property."""

    def test_at_action_start_boundary(self):
        cursor = create_cursor(num_actions=3, initial_location=1.0)
        assert cursor.current_action_end == 1.0  # ceil(1.0) = 1.0

    def test_midway_through_action(self):
        cursor = create_cursor(num_actions=3, initial_location=1.3)
        assert cursor.current_action_end == 2.0

    def test_just_after_boundary(self):
        cursor = create_cursor(num_actions=3, initial_location=1.001)
        assert cursor.current_action_end == 2.0

    def test_at_trajectory_start(self):
        cursor = create_cursor(num_actions=3, initial_location=0.0)
        assert cursor.current_action_end == 0.0


class TestNextActionStart:
    """Tests for next_action_start property."""

    def test_equals_current_action_end(self):
        cursor = create_cursor(num_actions=3, initial_location=0.5)
        assert cursor.next_action_start == cursor.current_action_end
        assert cursor.next_action_start == 1.0

    def test_at_exact_boundary(self):
        cursor = create_cursor(num_actions=3, initial_location=1.0)
        assert cursor.next_action_start == 1.0

    def test_midway_through_last_action(self):
        cursor = create_cursor(num_actions=3, initial_location=2.5)
        assert cursor.next_action_start == 3.0


class TestPreviousActionStart:
    """Tests for previous_action_start property."""

    def test_from_second_action(self):
        cursor = create_cursor(num_actions=3, initial_location=1.5)
        assert cursor.previous_action_start == 0.0

    def test_from_first_action_returns_negative(self):
        cursor = create_cursor(num_actions=3, initial_location=0.5)
        assert cursor.previous_action_start == -1.0

    def test_at_exact_boundary(self):
        cursor = create_cursor(num_actions=3, initial_location=2.0)
        assert cursor.previous_action_start == 1.0

    def test_from_third_action(self):
        cursor = create_cursor(num_actions=3, initial_location=2.5)
        assert cursor.previous_action_start == 1.0


class TestPreviousActionIndex:
    """Tests for previous_action_index property."""

    def test_from_second_action_midway(self):
        cursor = create_cursor(num_actions=3, initial_location=1.5)
        # ceil(1.5) - 1 = 2 - 1 = 1
        assert cursor.previous_action_index == 1

    def test_at_trajectory_start_clamps_to_zero(self):
        cursor = create_cursor(num_actions=3, initial_location=0.0)
        # ceil(0.0) - 1 = 0 - 1 = -1, clamped to 0
        assert cursor.previous_action_index == 0

    def test_from_first_action_midway(self):
        cursor = create_cursor(num_actions=3, initial_location=0.5)
        # ceil(0.5) - 1 = 1 - 1 = 0
        assert cursor.previous_action_index == 0

    def test_at_exact_boundary(self):
        cursor = create_cursor(num_actions=3, initial_location=2.0)
        # ceil(2.0) - 1 = 2 - 1 = 1
        assert cursor.previous_action_index == 1

    def test_from_last_action(self):
        cursor = create_cursor(num_actions=3, initial_location=2.5)
        # ceil(2.5) - 1 = 3 - 1 = 2
        assert cursor.previous_action_index == 2


class TestEndLocation:
    """Tests for end_location property."""

    def test_returns_last_trajectory_location(self):
        cursor = create_cursor(num_actions=3, initial_location=0.0)
        assert cursor.end_location == 3.0

    def test_single_action(self):
        cursor = create_cursor(num_actions=1, initial_location=0.0)
        assert cursor.end_location == 1.0

    def test_many_actions(self):
        cursor = create_cursor(num_actions=10, initial_location=0.0)
        assert cursor.end_location == 10.0


class TestMovementOptions:
    """Tests for get_movement_options method."""

    def test_at_start_can_only_move_forward(self):
        cursor = create_cursor(num_actions=3, initial_location=0.0)
        options = cursor.get_movement_options()
        assert MovementOption.CAN_MOVE_FORWARD in options
        assert MovementOption.CAN_MOVE_BACKWARD not in options

    def test_at_end_can_only_move_backward(self):
        cursor = create_cursor(num_actions=3, initial_location=3.0)
        options = cursor.get_movement_options()
        assert MovementOption.CAN_MOVE_FORWARD not in options
        assert MovementOption.CAN_MOVE_BACKWARD in options

    def test_in_middle_can_move_both_ways(self):
        cursor = create_cursor(num_actions=3, initial_location=1.5)
        options = cursor.get_movement_options()
        assert MovementOption.CAN_MOVE_FORWARD in options
        assert MovementOption.CAN_MOVE_BACKWARD in options

    def test_just_after_start(self):
        cursor = create_cursor(num_actions=3, initial_location=0.001)
        options = cursor.get_movement_options()
        assert MovementOption.CAN_MOVE_FORWARD in options
        assert MovementOption.CAN_MOVE_BACKWARD in options

    def test_just_before_end(self):
        cursor = create_cursor(num_actions=3, initial_location=2.999)
        options = cursor.get_movement_options()
        assert MovementOption.CAN_MOVE_FORWARD in options
        assert MovementOption.CAN_MOVE_BACKWARD in options


class TestNextAction:
    """Tests for next_action property."""

    def test_from_first_action_returns_second(self):
        cursor = create_cursor(num_actions=3, initial_location=0.5)
        assert cursor.next_action is not None

    def test_at_last_action_returns_none(self):
        cursor = create_cursor(num_actions=3, initial_location=2.5)
        assert cursor.next_action is None

    def test_at_trajectory_end_returns_none(self):
        cursor = create_cursor(num_actions=3, initial_location=3.0)
        assert cursor.next_action is None

    def test_at_second_to_last_action(self):
        cursor = create_cursor(num_actions=3, initial_location=1.5)
        assert cursor.next_action is not None

    def test_single_action_returns_none(self):
        cursor = create_cursor(num_actions=1, initial_location=0.5)
        assert cursor.next_action is None


class TestCurrentAction:
    """Tests for current_action property."""

    def test_returns_action_at_current_index(self):
        cursor = create_cursor(num_actions=3, initial_location=0.0)
        action = cursor.current_action
        assert action is not None

    def test_at_trajectory_end_returns_last_action(self):
        cursor = create_cursor(num_actions=3, initial_location=3.0)
        action = cursor.current_action
        assert action is not None

    def test_single_action(self):
        cursor = create_cursor(num_actions=1, initial_location=0.5)
        action = cursor.current_action
        assert action is not None