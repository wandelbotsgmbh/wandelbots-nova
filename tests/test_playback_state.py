"""Tests for Nova Playback State Management

This module tests the PlaybackControlState class to ensure that playback state
is always reflected correctly across all operations like pause, resume, speed changes,
and direction changes.
"""

import threading
from datetime import datetime, timezone
from unittest.mock import Mock

import pytest

from nova.playback.playback_events import PlaybackDirection, PlaybackSpeedPercent, PlaybackState
from nova.playback.playback_state import PlaybackControl, PlaybackControlState, RobotMetadata


class TestPlaybackControlState:
    """Test suite for PlaybackControlState state management"""

    @pytest.fixture
    def state(self):
        """Fresh state instance for each test"""
        return PlaybackControlState()

    @pytest.fixture
    def robot_id(self):
        """Standard robot ID for testing"""
        return "test_robot_1"

    @pytest.fixture
    def robot_metadata(self):
        """Sample robot metadata"""
        return RobotMetadata(
            name="Test Robot",
            registered_at=datetime.now(timezone.utc),
            initial_speed=PlaybackSpeedPercent(value=100),
        )

    @pytest.fixture
    def playback_control(self):
        """Sample playback control"""
        return PlaybackControl(
            speed=PlaybackSpeedPercent(value=50),
            source="external",
            state=PlaybackState.PLAYING,
            direction=PlaybackDirection.FORWARD,
        )

    def test_initial_state_is_empty(self, state):
        """Test that initial state has no robots or overrides"""
        assert state.get_all_robots() == []
        assert state.get_external_override("any_robot") is None
        assert state.get_decorator_default("any_robot") is None
        assert state.get_execution_state("any_robot") is None
        assert state.get_robot_metadata("any_robot") is None

    def test_robot_registration_state_persistence(self, state, robot_id, robot_metadata):
        """Test that robot metadata is stored and retrieved correctly"""
        # Register robot
        state.set_robot_metadata(robot_id, robot_metadata)

        # Verify metadata is stored
        retrieved_metadata = state.get_robot_metadata(robot_id)
        assert retrieved_metadata is not None
        assert retrieved_metadata.name == robot_metadata.name
        assert retrieved_metadata.initial_speed == robot_metadata.initial_speed

        # Verify robot appears in all_robots list
        assert robot_id in state.get_all_robots()

    def test_external_override_state_reflection(self, state, robot_id, playback_control):
        """Test that external overrides are immediately reflected in state"""
        # Initially no override
        assert state.get_external_override(robot_id) is None

        # Set external override
        state.set_external_override(robot_id, playback_control)

        # Verify override is immediately reflected
        retrieved_control = state.get_external_override(robot_id)
        assert retrieved_control is not None
        assert retrieved_control.speed == playback_control.speed
        assert retrieved_control.source == playback_control.source
        assert retrieved_control.state == playback_control.state
        assert retrieved_control.direction == playback_control.direction

    def test_external_override_clearing_state_reflection(self, state, robot_id, playback_control):
        """Test that clearing external overrides is immediately reflected"""
        # Set override first
        state.set_external_override(robot_id, playback_control)
        assert state.get_external_override(robot_id) is not None

        # Clear override
        state.clear_external_override(robot_id)

        # Verify override is immediately cleared
        assert state.get_external_override(robot_id) is None

    def test_decorator_default_state_reflection(self, state, robot_id, playback_control):
        """Test that decorator defaults are immediately reflected in state"""
        # Initially no default
        assert state.get_decorator_default(robot_id) is None

        # Set decorator default
        state.set_decorator_default(robot_id, playback_control)

        # Verify default is immediately reflected
        retrieved_control = state.get_decorator_default(robot_id)
        assert retrieved_control is not None
        assert retrieved_control.speed == playback_control.speed
        assert retrieved_control.source == playback_control.source

    def test_execution_state_pause_resume_reflection(self, state, robot_id):
        """Test that pause/resume state changes are immediately reflected"""
        # Initially no execution state
        assert state.get_execution_state(robot_id) is None

        # Set to EXECUTING
        state.set_execution_state(robot_id, PlaybackState.EXECUTING)
        assert state.get_execution_state(robot_id) == PlaybackState.EXECUTING

        # Pause (set to PAUSED)
        state.set_execution_state(robot_id, PlaybackState.PAUSED)
        assert state.get_execution_state(robot_id) == PlaybackState.PAUSED

        # Resume (set to PLAYING)
        state.set_execution_state(robot_id, PlaybackState.PLAYING)
        assert state.get_execution_state(robot_id) == PlaybackState.PLAYING

    def test_speed_change_state_reflection(self, state, robot_id):
        """Test that speed changes are immediately reflected in state"""
        # Set initial speed
        initial_control = PlaybackControl(speed=PlaybackSpeedPercent(value=100), source="external")
        state.set_external_override(robot_id, initial_control)
        assert state.get_external_override(robot_id).speed.value == 100

        # Change speed
        new_control = PlaybackControl(speed=PlaybackSpeedPercent(value=25), source="external")
        state.set_external_override(robot_id, new_control)
        assert state.get_external_override(robot_id).speed.value == 25

        # Change speed again
        another_control = PlaybackControl(speed=PlaybackSpeedPercent(value=75), source="external")
        state.set_external_override(robot_id, another_control)
        assert state.get_external_override(robot_id).speed.value == 75

    def test_direction_change_state_reflection(self, state, robot_id):
        """Test that direction changes are immediately reflected in state"""
        # Set initial direction
        forward_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=50),
            source="external",
            direction=PlaybackDirection.FORWARD,
        )
        state.set_external_override(robot_id, forward_control)
        assert state.get_external_override(robot_id).direction == PlaybackDirection.FORWARD

        # Change direction
        backward_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=50),
            source="external",
            direction=PlaybackDirection.BACKWARD,
        )
        state.set_external_override(robot_id, backward_control)
        assert state.get_external_override(robot_id).direction == PlaybackDirection.BACKWARD

        # Change back to forward
        forward_again_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=50),
            source="external",
            direction=PlaybackDirection.FORWARD,
        )
        state.set_external_override(robot_id, forward_again_control)
        assert state.get_external_override(robot_id).direction == PlaybackDirection.FORWARD

    def test_combined_state_changes_reflection(self, state, robot_id):
        """Test that combined state changes (speed + direction + state) are reflected"""
        # Set initial combined state
        initial_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=100),
            source="external",
            state=PlaybackState.PLAYING,
            direction=PlaybackDirection.FORWARD,
        )
        state.set_external_override(robot_id, initial_control)

        retrieved = state.get_external_override(robot_id)
        assert retrieved.speed.value == 100
        assert retrieved.state == PlaybackState.PLAYING
        assert retrieved.direction == PlaybackDirection.FORWARD

        # Change all aspects simultaneously
        new_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=25),
            source="external",
            state=PlaybackState.PAUSED,
            direction=PlaybackDirection.BACKWARD,
        )
        state.set_external_override(robot_id, new_control)

        retrieved = state.get_external_override(robot_id)
        assert retrieved.speed.value == 25
        assert retrieved.state == PlaybackState.PAUSED
        assert retrieved.direction == PlaybackDirection.BACKWARD

    def test_multiple_robots_state_isolation(self, state, robot_metadata):
        """Test that state changes for one robot don't affect others"""
        robot1_id = "robot_1"
        robot2_id = "robot_2"

        # Set up different states for each robot
        robot1_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=30),
            source="external",
            state=PlaybackState.PLAYING,
            direction=PlaybackDirection.FORWARD,
        )
        robot2_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=70),
            source="external",
            state=PlaybackState.PAUSED,
            direction=PlaybackDirection.BACKWARD,
        )

        state.set_external_override(robot1_id, robot1_control)
        state.set_external_override(robot2_id, robot2_control)

        # Verify each robot has its own state
        robot1_state = state.get_external_override(robot1_id)
        robot2_state = state.get_external_override(robot2_id)

        assert robot1_state.speed.value == 30
        assert robot1_state.state == PlaybackState.PLAYING
        assert robot1_state.direction == PlaybackDirection.FORWARD

        assert robot2_state.speed.value == 70
        assert robot2_state.state == PlaybackState.PAUSED
        assert robot2_state.direction == PlaybackDirection.BACKWARD

        # Change robot1 state and verify robot2 is unaffected
        robot1_new_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=90),
            source="external",
            state=PlaybackState.PAUSED,
            direction=PlaybackDirection.BACKWARD,
        )
        state.set_external_override(robot1_id, robot1_new_control)

        # Robot1 should have new state
        robot1_updated = state.get_external_override(robot1_id)
        assert robot1_updated.speed.value == 90
        assert robot1_updated.state == PlaybackState.PAUSED
        assert robot1_updated.direction == PlaybackDirection.BACKWARD

        # Robot2 should be unchanged
        robot2_unchanged = state.get_external_override(robot2_id)
        assert robot2_unchanged.speed.value == 70
        assert robot2_unchanged.state == PlaybackState.PAUSED
        assert robot2_unchanged.direction == PlaybackDirection.BACKWARD

    def test_robot_removal_state_cleanup(self, state, robot_id, robot_metadata, playback_control):
        """Test that removing a robot cleans up all its state"""
        # Set up complete state for robot
        state.set_robot_metadata(robot_id, robot_metadata)
        state.set_external_override(robot_id, playback_control)
        state.set_decorator_default(robot_id, playback_control)
        state.set_execution_state(robot_id, PlaybackState.EXECUTING)

        # Verify state is set
        assert state.get_robot_metadata(robot_id) is not None
        assert state.get_external_override(robot_id) is not None
        assert state.get_decorator_default(robot_id) is not None
        assert state.get_execution_state(robot_id) is not None
        assert robot_id in state.get_all_robots()

        # Remove robot
        state.remove_robot(robot_id)

        # Verify all state is cleaned up
        assert state.get_robot_metadata(robot_id) is None
        assert state.get_external_override(robot_id) is None
        assert state.get_decorator_default(robot_id) is None
        assert state.get_execution_state(robot_id) is None
        assert robot_id not in state.get_all_robots()

    def test_program_state_reflection(self, state):
        """Test that program-level state changes are immediately reflected"""
        # Initially no active program
        assert state.get_active_program_name() is None
        assert state.get_active_program_speed() is None

        # Set active program
        state.set_active_program_name("test_program")
        assert state.get_active_program_name() == "test_program"

        # Set program speed
        state.set_active_program_speed(75)
        assert state.get_active_program_speed() == 75

        # Clear program
        state.set_active_program_name(None)
        assert state.get_active_program_name() is None

        # Clear program speed
        state.set_active_program_speed(None)
        assert state.get_active_program_speed() is None

    def test_event_callback_state_reflection(self, state):
        """Test that event callback registration is immediately reflected"""
        # Initially no callbacks
        assert state.get_event_callbacks() == []

        # Add callback
        callback1 = Mock()
        state.add_event_callback(callback1)
        assert len(state.get_event_callbacks()) == 1
        assert callback1 in state.get_event_callbacks()

        # Add another callback
        callback2 = Mock()
        state.add_event_callback(callback2)
        assert len(state.get_event_callbacks()) == 2
        assert callback1 in state.get_event_callbacks()
        assert callback2 in state.get_event_callbacks()

    def test_thread_safety_state_consistency(self, state, robot_metadata):
        """Test that state changes are thread-safe and consistent"""
        robot_id = "thread_test_robot"
        state.set_robot_metadata(robot_id, robot_metadata)

        results = []
        errors = []

        def modify_state(speed_value, thread_id):
            try:
                # Create control with unique speed
                control = PlaybackControl(
                    speed=PlaybackSpeedPercent(value=speed_value),
                    source="external",
                    state=PlaybackState.PLAYING,
                    direction=PlaybackDirection.FORWARD,
                )

                # Set override
                state.set_external_override(robot_id, control)

                # Immediately read back
                retrieved = state.get_external_override(robot_id)
                results.append((thread_id, retrieved.speed.value if retrieved else None))

                # Also test execution state changes
                execution_state = (
                    PlaybackState.PAUSED if speed_value % 2 == 0 else PlaybackState.EXECUTING
                )
                state.set_execution_state(robot_id, execution_state)

                # Read back execution state
                retrieved_execution = state.get_execution_state(robot_id)
                results.append((f"{thread_id}_execution", retrieved_execution))

            except Exception as e:
                errors.append((thread_id, e))

        # Run multiple threads modifying state
        threads = []
        for i in range(10):
            speed = 10 * (i + 1)  # 10 to 100
            thread = threading.Thread(target=modify_state, args=(speed, i))
            threads.append(thread)
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

        # Should have no errors
        assert len(errors) == 0, f"Errors occurred: {errors}"

        # Should have results from all threads (20 results: 10 speed + 10 execution)
        assert len(results) == 20

        # Final state should be consistent
        final_override = state.get_external_override(robot_id)
        final_execution = state.get_execution_state(robot_id)
        assert final_override is not None
        assert final_execution is not None
        assert final_override.speed.value in [10 * (i + 1) for i in range(10)]
        assert final_execution in [PlaybackState.PAUSED, PlaybackState.EXECUTING]

    def test_state_persistence_across_operations(self, state, robot_id, robot_metadata):
        """Test that state persists correctly across multiple operations"""
        # Set initial state
        state.set_robot_metadata(robot_id, robot_metadata)
        initial_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=80),
            source="external",
            state=PlaybackState.PLAYING,
            direction=PlaybackDirection.FORWARD,
        )
        state.set_external_override(robot_id, initial_control)
        state.set_execution_state(robot_id, PlaybackState.EXECUTING)

        # Perform multiple operations
        operations = [
            (
                lambda: state.set_execution_state(robot_id, PlaybackState.PAUSED),
                PlaybackState.PAUSED,
            ),
            (
                lambda: state.set_execution_state(robot_id, PlaybackState.PLAYING),
                PlaybackState.PLAYING,
            ),
            (
                lambda: state.set_external_override(
                    robot_id,
                    PlaybackControl(
                        speed=PlaybackSpeedPercent(value=50),
                        source="external",
                        direction=PlaybackDirection.BACKWARD,
                    ),
                ),
                50,
            ),
            (
                lambda: state.set_external_override(
                    robot_id,
                    PlaybackControl(
                        speed=PlaybackSpeedPercent(value=25),
                        source="external",
                        state=PlaybackState.PAUSED,
                    ),
                ),
                25,
            ),
        ]

        for operation, expected_value in operations:
            operation()

            # Verify state is always consistent after each operation
            assert state.get_robot_metadata(robot_id) is not None
            assert state.get_external_override(robot_id) is not None

            if isinstance(expected_value, PlaybackState):
                assert state.get_execution_state(robot_id) == expected_value
            elif isinstance(expected_value, int):
                assert state.get_external_override(robot_id).speed.value == expected_value

    def test_immediate_state_reflection_timing(self, state, robot_id):
        """Test that state changes are reflected immediately without delay"""
        import time

        # Test speed change timing
        start_time = time.time()
        speed_control = PlaybackControl(speed=PlaybackSpeedPercent(value=42), source="external")
        state.set_external_override(robot_id, speed_control)

        # State should be immediately available
        retrieved = state.get_external_override(robot_id)
        end_time = time.time()

        assert retrieved.speed.value == 42
        assert (end_time - start_time) < 0.01  # Should be near-instantaneous

        # Test execution state change timing
        start_time = time.time()
        state.set_execution_state(robot_id, PlaybackState.PAUSED)

        # State should be immediately available
        execution_state = state.get_execution_state(robot_id)
        end_time = time.time()

        assert execution_state == PlaybackState.PAUSED
        assert (end_time - start_time) < 0.01  # Should be near-instantaneous

    def test_state_immutability_after_setting(self, state, robot_id):
        """Test that state objects remain immutable after being set"""
        # Set initial control
        original_control = PlaybackControl(
            speed=PlaybackSpeedPercent(value=60),
            source="external",
            state=PlaybackState.PLAYING,
            direction=PlaybackDirection.FORWARD,
        )
        state.set_external_override(robot_id, original_control)

        # Get the stored control
        retrieved_control = state.get_external_override(robot_id)

        # Verify it's the same content but immutable
        assert retrieved_control.speed.value == 60
        assert retrieved_control.state == PlaybackState.PLAYING
        assert retrieved_control.direction == PlaybackDirection.FORWARD

        # The PlaybackControl is frozen, so modification should fail
        with pytest.raises(Exception):  # Should raise ValidationError or similar
            retrieved_control.speed = PlaybackSpeedPercent(value=30)
