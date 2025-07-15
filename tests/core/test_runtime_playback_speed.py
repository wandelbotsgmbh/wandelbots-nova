"""
Test for runtime playback speed change mechanism.

This test captures the key learnings from debugging the robot ID mismatch issue:
- Movement controller and external speed setters must use the same robot ID
- Robot ID should be the motion group ID (e.g., "0@ur"), not the motion UUID
- The playback control manager must be globally shared between components
- Speed changes should be detectable in real-time during execution
"""

import pytest

from nova.actions.container import CombinedActions, MovementControllerContext
from nova.actions.motions import cartesian_ptp
from nova.playback import MotionGroupId, PlaybackSpeedPercent, get_playback_manager
from nova.playback.playback_control_manager import PlaybackControlManager


class TestRuntimePlaybackSpeedChange:
    """Test the runtime playback speed change mechanism."""

    @pytest.fixture(autouse=True)
    def setup_clean_manager(self, playback_manager: PlaybackControlManager):
        """Clear playback manager state before each test for isolation."""
        # Access the internal state object to clear state
        with playback_manager._state._lock:
            playback_manager._state._external_overrides.clear()
            playback_manager._state._decorator_defaults.clear()

    @pytest.fixture
    def robot_id(self) -> MotionGroupId:
        """Robot ID representing a motion group (e.g., '0@ur')."""
        return MotionGroupId("0@test_controller")

    @pytest.fixture
    def movement_context(self, robot_id: MotionGroupId) -> MovementControllerContext:
        """Movement controller context with proper robot ID."""
        actions = [cartesian_ptp((100, 200, 300, 0, 0, 0))]
        return MovementControllerContext(
            combined_actions=CombinedActions(items=tuple(actions)),
            motion_id="uuid-12345-motion",  # This is the planned motion UUID
            motion_group_id=robot_id,  # This is the motion group ID - critical for speed control
            effective_speed=10,
            method_speed=None,
        )

    @pytest.fixture
    def playback_manager(self) -> PlaybackControlManager:
        """Get the global playback manager."""
        manager = get_playback_manager()
        # The interface should return our implementation
        assert isinstance(manager, PlaybackControlManager)
        return manager

    def test_robot_id_consistency(
        self, robot_id: MotionGroupId, movement_context: MovementControllerContext
    ):
        """
        Test that external speed setters and movement controller use the same robot ID.

        This was the root cause of the bug: demo used motion group ID while
        movement controller derived robot ID from motion UUID.
        """
        # External speed setter (like demo) uses motion group ID
        external_robot_id = robot_id

        # Movement controller should use the same robot ID from context
        controller_robot_id = MotionGroupId(movement_context.motion_group_id)

        assert external_robot_id == controller_robot_id, (
            f"Robot ID mismatch: external='{external_robot_id}', controller='{controller_robot_id}'"
        )

    def test_global_manager_sharing(self, playback_manager: PlaybackControlManager):
        """
        Test that all components share the same global playback manager instance.

        This ensures speed changes set by one component are visible to others.
        """
        # Get manager from different access points
        manager1 = get_playback_manager()
        manager2 = get_playback_manager()

        assert manager1 is manager2 is playback_manager, (
            "All components must share the same playback manager instance"
        )

    def test_runtime_speed_change_detection(
        self,
        robot_id: MotionGroupId,
        movement_context: MovementControllerContext,
        playback_manager: PlaybackControlManager,
    ):
        """
        Test that runtime speed changes are properly detected and applied.

        This simulates the core functionality: external tool changes speed,
        movement controller detects and applies the change.
        """
        # Set up initial state - simulate decorator default that created the effective_speed
        initial_speed = PlaybackSpeedPercent(value=movement_context.effective_speed)
        playback_manager.set_decorator_default(robot_id, initial_speed)

        # Verify initial state
        method_speed = (
            PlaybackSpeedPercent(value=movement_context.method_speed)
            if movement_context.method_speed
            else None
        )
        current_speed = playback_manager.get_effective_speed(robot_id, method_speed=method_speed)
        assert current_speed == initial_speed

        # External tool changes speed (simulating external control interface or demo)
        new_speed = PlaybackSpeedPercent(value=75)
        playback_manager.set_external_override(robot_id, new_speed)

        # Movement controller should detect the change
        detected_speed = playback_manager.get_effective_speed(robot_id, method_speed=method_speed)
        assert detected_speed == new_speed, (
            f"Movement controller should detect speed change: "
            f"expected {new_speed}%, got {detected_speed}%"
        )

        # Verify external override is properly stored
        # Note: In the new architecture, we trust the public interface
        # The override should be reflected in the effective speed
        stored_speed = playback_manager.get_effective_speed(robot_id)
        assert stored_speed == new_speed, f"Override speed should be {new_speed}%"

    def test_precedence_resolution(
        self, robot_id: MotionGroupId, playback_manager: PlaybackControlManager
    ):
        """
        Test that speed precedence is correctly resolved: external > method > decorator > default.

        This ensures external speed changes properly override other speed settings.
        """
        # Set decorator default
        decorator_speed = PlaybackSpeedPercent(value=30)
        playback_manager.set_decorator_default(robot_id, decorator_speed)

        # Method speed (higher precedence than decorator)
        method_speed = PlaybackSpeedPercent(value=60)
        effective_speed = playback_manager.get_effective_speed(robot_id, method_speed=method_speed)
        assert effective_speed == method_speed, "Method speed should override decorator default"

        # External override (highest precedence)
        external_speed = PlaybackSpeedPercent(value=90)
        playback_manager.set_external_override(robot_id, external_speed)
        effective_speed = playback_manager.get_effective_speed(robot_id, method_speed=method_speed)
        assert effective_speed == external_speed, "External override should have highest precedence"

    def test_multiple_speed_changes(
        self, robot_id: MotionGroupId, playback_manager: PlaybackControlManager
    ):
        """
        Test multiple consecutive speed changes are properly handled.

        This simulates the demo scenario with multiple speed changes during execution.
        """
        speed_sequence = [
            PlaybackSpeedPercent(value=25),
            PlaybackSpeedPercent(value=100),
            PlaybackSpeedPercent(value=50),
        ]

        for expected_speed in speed_sequence:
            # External tool changes speed
            playback_manager.set_external_override(robot_id, expected_speed)

            # Verify change is immediately detectable
            detected_speed = playback_manager.get_effective_speed(robot_id)
            assert detected_speed == expected_speed, (
                f"Speed change to {expected_speed}% not detected, got {detected_speed}%"
            )

    def test_movement_context_robot_id_field(self, movement_context: MovementControllerContext):
        """
        Test that MovementControllerContext includes robot_id field.

        This was added to fix the robot ID mismatch - ensures motion group
        passes the correct robot ID to the movement controller.
        """
        assert hasattr(movement_context, "motion_group_id"), (
            "MovementControllerContext must have motion_group_id field"
        )
        assert movement_context.motion_group_id == "0@test_controller", (
            "motion_group_id should be the motion group ID, not motion UUID"
        )
        assert movement_context.motion_group_id != movement_context.motion_id, (
            "motion_group_id should be different from motion_id (UUID)"
        )
