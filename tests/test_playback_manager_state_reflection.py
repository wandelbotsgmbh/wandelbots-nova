"""Integration Tests for Nova Playback Control Manager State Reflection

This module tests the PlaybackControlManager to ensure that all state changes
(pause, resume, speed changes, direction changes) are properly reflected
and integrated at the manager level.
"""

import threading
import time

import pytest

from nova.playback import PlaybackSpeedPercent, PlaybackState, get_playback_manager
from nova.playback.playback_control_manager import PlaybackControlManager
from nova.playback.playback_events import PlaybackDirection


class TestPlaybackControlManagerStateReflection:
    """Test suite for PlaybackControlManager state reflection and integration"""

    @pytest.fixture
    def manager(self):
        """Fresh manager instance for each test"""
        return PlaybackControlManager()

    @pytest.fixture
    def robot_id(self):
        """Standard robot ID for testing"""
        return "test_robot_integration"

    def test_pause_state_immediately_reflected(self, manager, robot_id):
        """Test that pause state is immediately reflected in get_effective_state"""
        # Register robot first
        manager.register_robot(robot_id, "Test Robot")

        # Initially should be PLAYING
        assert manager.get_effective_state(robot_id) == PlaybackState.PLAYING

        # Pause should be immediately reflected
        manager.pause(robot_id)
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

        # Multiple pauses should not change state
        manager.pause(robot_id)
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

    def test_resume_state_immediately_reflected(self, manager, robot_id):
        """Test that resume state is immediately reflected in get_effective_state"""
        # Register robot and pause it
        manager.register_robot(robot_id, "Test Robot")
        manager.pause(robot_id)
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

        # Resume should be immediately reflected
        manager.resume(robot_id)
        assert manager.get_effective_state(robot_id) == PlaybackState.PLAYING

    def test_speed_change_immediately_reflected(self, manager, robot_id):
        """Test that speed changes are immediately reflected in get_effective_speed"""
        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        # Initial speed should be 100 (default)
        assert manager.get_effective_speed(robot_id).value == 100

        # Speed change should be immediately reflected
        new_speed = PlaybackSpeedPercent(value=30)
        manager.set_external_override(robot_id, new_speed)
        assert manager.get_effective_speed(robot_id).value == 30

        # Another speed change should be immediately reflected
        another_speed = PlaybackSpeedPercent(value=85)
        manager.set_external_override(robot_id, another_speed)
        assert manager.get_effective_speed(robot_id).value == 85

    def test_direction_change_immediately_reflected(self, manager, robot_id):
        """Test that direction changes are immediately reflected in get_effective_direction"""
        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        # Initial direction should be FORWARD
        assert manager.get_effective_direction(robot_id) == PlaybackDirection.FORWARD

        # Direction change should be immediately reflected
        manager.set_external_override(
            robot_id, PlaybackSpeedPercent(value=50), direction=PlaybackDirection.BACKWARD
        )
        assert manager.get_effective_direction(robot_id) == PlaybackDirection.BACKWARD

        # Change back to forward
        manager.set_external_override(
            robot_id, PlaybackSpeedPercent(value=50), direction=PlaybackDirection.FORWARD
        )
        assert manager.get_effective_direction(robot_id) == PlaybackDirection.FORWARD

    def test_combined_state_changes_reflected(self, manager, robot_id):
        """Test that combined state changes (speed + state + direction) are all reflected"""
        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        # Set combined state
        manager.set_external_override(
            robot_id,
            PlaybackSpeedPercent(value=25),
            state=PlaybackState.PAUSED,
            direction=PlaybackDirection.BACKWARD,
        )

        # All aspects should be immediately reflected
        assert manager.get_effective_speed(robot_id).value == 25
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED
        assert manager.get_effective_direction(robot_id) == PlaybackDirection.BACKWARD

    def test_pause_preserves_speed_and_direction(self, manager, robot_id):
        """Test that pause preserves current speed and direction settings"""
        # Register robot and set initial state
        manager.register_robot(robot_id, "Test Robot")
        manager.set_external_override(
            robot_id, PlaybackSpeedPercent(value=40), direction=PlaybackDirection.BACKWARD
        )

        # Verify initial state
        assert manager.get_effective_speed(robot_id).value == 40
        assert manager.get_effective_direction(robot_id) == PlaybackDirection.BACKWARD
        assert manager.get_effective_state(robot_id) == PlaybackState.PLAYING

        # Pause should preserve speed and direction
        manager.pause(robot_id)
        assert manager.get_effective_speed(robot_id).value == 40
        assert manager.get_effective_direction(robot_id) == PlaybackDirection.BACKWARD
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

    def test_resume_preserves_speed_and_direction(self, manager, robot_id):
        """Test that resume preserves current speed and direction settings"""
        # Register robot and set initial state
        manager.register_robot(robot_id, "Test Robot")
        manager.set_external_override(
            robot_id, PlaybackSpeedPercent(value=60), direction=PlaybackDirection.BACKWARD
        )
        manager.pause(robot_id)

        # Resume should preserve speed and direction
        manager.resume(robot_id)
        assert manager.get_effective_speed(robot_id).value == 60
        assert manager.get_effective_direction(robot_id) == PlaybackDirection.BACKWARD
        assert manager.get_effective_state(robot_id) == PlaybackState.PLAYING

    def test_speed_change_during_pause_reflected(self, manager, robot_id):
        """Test that speed changes during pause are properly reflected"""
        # Register robot and pause
        manager.register_robot(robot_id, "Test Robot")
        manager.pause(robot_id)

        # Change speed while paused
        manager.set_external_override(robot_id, PlaybackSpeedPercent(value=15))

        # Both speed and pause state should be reflected
        assert manager.get_effective_speed(robot_id).value == 15
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

    def test_direction_change_during_pause_reflected(self, manager, robot_id):
        """Test that direction changes during pause are properly reflected"""
        # Register robot and pause
        manager.register_robot(robot_id, "Test Robot")
        manager.pause(robot_id)

        # Change direction while paused
        manager.set_external_override(
            robot_id, PlaybackSpeedPercent(value=50), direction=PlaybackDirection.BACKWARD
        )

        # Both direction and pause state should be reflected
        assert manager.get_effective_direction(robot_id) == PlaybackDirection.BACKWARD
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

    def test_external_override_with_playing_state_clears_pause(self, manager, robot_id):
        """Test that external override with PLAYING state clears pause state"""
        # Register robot and pause
        manager.register_robot(robot_id, "Test Robot")
        manager.pause(robot_id)
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

        # Set external override with PLAYING state
        manager.set_external_override(
            robot_id, PlaybackSpeedPercent(value=80), state=PlaybackState.PLAYING
        )

        # Should clear pause and be playing
        assert manager.get_effective_state(robot_id) == PlaybackState.PLAYING
        assert manager.get_effective_speed(robot_id).value == 80

    def test_clear_external_override_preserves_pause_state(self, manager, robot_id):
        """Test that clearing external override preserves pause state"""
        # Register robot and set default speed
        manager.register_robot(robot_id, "Test Robot")
        manager.set_decorator_default(robot_id, PlaybackSpeedPercent(value=70))

        # Set external override and pause
        manager.set_external_override(robot_id, PlaybackSpeedPercent(value=30))
        manager.pause(robot_id)

        # Clear external override
        manager.clear_external_override(robot_id)

        # Should fall back to decorator default but preserve pause state
        assert manager.get_effective_speed(robot_id).value == 70
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

    def test_multiple_robots_state_isolation(self, manager):
        """Test that state changes for one robot don't affect others"""
        robot1 = "robot_1"
        robot2 = "robot_2"

        # Register both robots
        manager.register_robot(robot1, "Robot 1")
        manager.register_robot(robot2, "Robot 2")

        # Set different states for each robot
        manager.set_external_override(robot1, PlaybackSpeedPercent(value=20))
        manager.set_external_override(robot2, PlaybackSpeedPercent(value=90))
        manager.pause(robot1)
        # robot2 remains playing

        # Verify isolation
        assert manager.get_effective_speed(robot1).value == 20
        assert manager.get_effective_speed(robot2).value == 90
        assert manager.get_effective_state(robot1) == PlaybackState.PAUSED
        assert manager.get_effective_state(robot2) == PlaybackState.PLAYING

        # Change robot1 state, verify robot2 unaffected
        manager.set_external_override(robot1, PlaybackSpeedPercent(value=95))
        manager.resume(robot1)

        assert manager.get_effective_speed(robot1).value == 95
        assert manager.get_effective_speed(robot2).value == 90  # unchanged
        assert manager.get_effective_state(robot1) == PlaybackState.PLAYING
        assert manager.get_effective_state(robot2) == PlaybackState.PLAYING  # unchanged

    def test_event_emission_on_state_changes(self, manager, robot_id):
        """Test that state changes trigger appropriate events"""
        events = []

        def capture_event(event):
            events.append(event)

        # Register robot and event callback
        manager.register_robot(robot_id, "Test Robot")
        manager.register_event_callback(capture_event)

        # Clear initial registration events
        events.clear()

        # Test pause event
        manager.pause(robot_id)
        assert len(events) >= 1
        state_change_events = [e for e in events if e.event_type == "state_change"]
        assert len(state_change_events) >= 1
        assert state_change_events[-1].new_state == PlaybackState.PAUSED

        # Test resume event
        events.clear()
        manager.resume(robot_id)
        assert len(events) >= 1
        state_change_events = [e for e in events if e.event_type == "state_change"]
        assert len(state_change_events) >= 1
        assert state_change_events[-1].new_state == PlaybackState.PLAYING

        # Test speed change event
        events.clear()
        manager.set_external_override(robot_id, PlaybackSpeedPercent(value=35))
        assert len(events) >= 1
        speed_change_events = [e for e in events if e.event_type == "speed_change"]
        assert len(speed_change_events) >= 1
        assert speed_change_events[-1].new_speed.value == 35

    def test_thread_safety_state_consistency(self, manager, robot_id):
        """Test that concurrent state changes are thread-safe and consistent"""
        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        results = []
        errors = []

        def concurrent_operations(thread_id):
            try:
                # Perform various operations
                speed = 10 + (thread_id * 5)  # Different speed for each thread

                # Set speed
                manager.set_external_override(robot_id, PlaybackSpeedPercent(value=speed))

                # Pause and resume
                if thread_id % 2 == 0:
                    manager.pause(robot_id)
                    time.sleep(0.001)  # Small delay
                    manager.resume(robot_id)

                # Record final state
                final_speed = manager.get_effective_speed(robot_id)
                final_state = manager.get_effective_state(robot_id)
                results.append((thread_id, final_speed.value, final_state))

            except Exception as e:
                errors.append((thread_id, e))

        # Run multiple threads
        threads = []
        for i in range(10):
            thread = threading.Thread(target=concurrent_operations, args=(i,))
            threads.append(thread)
            thread.start()

        # Wait for all threads
        for thread in threads:
            thread.join()

        # Verify no errors
        assert len(errors) == 0, f"Errors occurred: {errors}"

        # Verify we got results from all threads
        assert len(results) == 10

        # Final state should be consistent
        final_speed = manager.get_effective_speed(robot_id)
        final_state = manager.get_effective_state(robot_id)

        # Speed should be one of the values set by threads
        expected_speeds = [10 + (i * 5) for i in range(10)]
        assert final_speed.value in expected_speeds
        assert final_state in [PlaybackState.PLAYING, PlaybackState.PAUSED]

    def test_immediate_state_reflection_timing(self, manager, robot_id):
        """Test that state changes are reflected immediately without delay"""
        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        # Test pause timing
        start_time = time.time()
        manager.pause(robot_id)
        state_after_pause = manager.get_effective_state(robot_id)
        end_time = time.time()

        assert state_after_pause == PlaybackState.PAUSED
        assert (end_time - start_time) < 0.01  # Should be near-instantaneous

        # Test resume timing
        start_time = time.time()
        manager.resume(robot_id)
        state_after_resume = manager.get_effective_state(robot_id)
        end_time = time.time()

        assert state_after_resume == PlaybackState.PLAYING
        assert (end_time - start_time) < 0.01  # Should be near-instantaneous

        # Test speed change timing
        start_time = time.time()
        manager.set_external_override(robot_id, PlaybackSpeedPercent(value=77))
        speed_after_change = manager.get_effective_speed(robot_id)
        end_time = time.time()

        assert speed_after_change.value == 77
        assert (end_time - start_time) < 0.01  # Should be near-instantaneous

    def test_global_manager_state_persistence(self):
        """Test that global manager maintains state across different calls"""
        robot_id = "global_test_robot"

        # Get global manager and set state
        manager1 = get_playback_manager()
        manager1.register_robot(robot_id, "Global Test Robot", PlaybackSpeedPercent(value=100))
        manager1.set_external_override(robot_id, PlaybackSpeedPercent(value=33))
        manager1.pause(robot_id)

        # Get global manager again and verify state is preserved
        manager2 = get_playback_manager()
        assert manager2.get_effective_speed(robot_id).value == 33
        assert manager2.get_effective_state(robot_id) == PlaybackState.PAUSED

        # Verify it's the same instance
        assert manager1 is manager2

    def test_precedence_hierarchy_state_reflection(self, manager, robot_id):
        """Test that precedence hierarchy is properly reflected in state"""
        # Register robot with initial speed
        manager.register_robot(robot_id, "Test Robot", PlaybackSpeedPercent(value=100))

        # Set decorator default
        manager.set_decorator_default(robot_id, PlaybackSpeedPercent(value=80))
        assert manager.get_effective_speed(robot_id).value == 80

        # Set external override (should take precedence)
        manager.set_external_override(robot_id, PlaybackSpeedPercent(value=60))
        assert manager.get_effective_speed(robot_id).value == 60

        # Method parameter should NOT override external override (external has highest precedence)
        method_speed = manager.get_effective_speed(
            robot_id, method_speed=PlaybackSpeedPercent(value=40)
        )
        assert method_speed.value == 60  # External override still wins
        assert manager.get_effective_speed(robot_id).value == 60  # Without method parameter

        # Clear external override - should fall back to decorator default
        manager.clear_external_override(robot_id)
        assert manager.get_effective_speed(robot_id).value == 80

        # Now method parameter should take precedence over decorator default
        method_speed = manager.get_effective_speed(
            robot_id, method_speed=PlaybackSpeedPercent(value=40)
        )
        assert method_speed.value == 40  # Method parameter overrides decorator default
        assert manager.get_effective_speed(robot_id).value == 80  # Without method parameter

    def test_can_pause_can_resume_state_reflection(self, manager, robot_id):
        """Test that can_pause and can_resume properly reflect current state"""
        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        # Set to executing state
        manager.set_execution_state(robot_id, PlaybackState.EXECUTING)
        assert manager.can_pause(robot_id)
        assert not manager.can_resume(robot_id)

        # Pause
        manager.pause(robot_id)
        assert not manager.can_pause(robot_id)
        assert manager.can_resume(robot_id)

        # Resume
        manager.resume(robot_id)
        assert manager.can_pause(robot_id)  # Should be pausable again after resume
        assert not manager.can_resume(robot_id)

        # Set to executing again
        manager.set_execution_state(robot_id, PlaybackState.EXECUTING)
        assert manager.can_pause(robot_id)
        assert not manager.can_resume(robot_id)
