"""Tests for Nova Movement Controller behavior and integration"""

import pytest

from nova.actions import CombinedActions, MovementControllerContext
from nova.actions.motions import cartesian_ptp


class TestMovementControllerContext:
    """Test the MovementControllerContext data class"""

    @pytest.fixture
    def base_context_args(self):
        """Common arguments for creating contexts"""
        actions = [cartesian_ptp((100, 200, 300, 0, 0, 0))]
        return {
            "combined_actions": CombinedActions(items=tuple(actions)),
            "motion_id": "test_motion_123",
            "robot_id": "0@test_controller",
        }

    def test_context_with_explicit_speed(self, base_context_args):
        """Test creating context with explicit speed"""
        context = MovementControllerContext(**base_context_args, effective_speed=50)
        assert context.effective_speed == 50
        assert context.motion_id == "test_motion_123"
        assert len(context.combined_actions.items) == 1

    def test_context_default_speed(self, base_context_args):
        """Test that default speed is 100%"""
        context = MovementControllerContext(**base_context_args)
        assert context.effective_speed == 100

    @pytest.mark.parametrize("speed", [0, 10, 25, 50, 75, 100])
    def test_context_valid_speeds(self, base_context_args, speed):
        """Test that valid speed values are accepted"""
        context = MovementControllerContext(**base_context_args, effective_speed=speed)
        assert context.effective_speed == speed

    def test_context_with_empty_actions(self):
        """Test context with empty actions"""
        context = MovementControllerContext(
            combined_actions=CombinedActions(items=tuple()),
            motion_id="empty_motion",
            motion_group_id="0@test_controller",
            effective_speed=25,
        )
        assert context.effective_speed == 25
        assert context.motion_id == "empty_motion"
        assert len(context.combined_actions.items) == 0

    def test_context_field_access(self, base_context_args):
        """Test that MovementControllerContext fields are accessible"""
        context = MovementControllerContext(**base_context_args, effective_speed=50)

        # These should work (reading)
        assert context.effective_speed == 50
        assert context.motion_id == "test_motion_123"
        assert isinstance(context.combined_actions, CombinedActions)

        # These should work (writing) - Pydantic models are mutable by default
        context.effective_speed = 75
        assert context.effective_speed == 75
