"""Tests for Nova Playback Control core functionality"""

import threading
from datetime import datetime

import pytest
from pydantic import ValidationError

from nova.playback import (
    InvalidSpeedError,
    PlaybackSpeedPercent,
    PlaybackState,
    get_playback_manager,
)
from nova.playback.playback_control_manager import PlaybackControlManager
from nova.playback.playback_state import PlaybackControl


class TestPlaybackControlManager:
    """Test suite for PlaybackControlManager core functionality"""

    @pytest.fixture
    def manager(self):
        """Fresh manager instance for each test"""
        return PlaybackControlManager()

    @pytest.fixture
    def robot_id(self):
        """Standard robot ID for testing"""
        return "test_robot_1"

    def test_default_speed_is_one(self, manager, robot_id):
        """Test that default speed is 1.0 when no overrides are set"""
        speed = manager.get_effective_speed(robot_id)
        assert speed == 100

    def test_method_parameter_overrides_default(self, manager, robot_id):
        """Test that method parameter takes precedence over default"""
        speed = manager.get_effective_speed(robot_id, method_speed=PlaybackSpeedPercent(value=50))
        assert speed == 50

    def test_decorator_default_overrides_system_default(self, manager, robot_id):
        """Test that decorator default takes precedence over system default"""
        manager.set_decorator_default(robot_id, PlaybackSpeedPercent(70))
        speed = manager.get_effective_speed(robot_id)
        assert speed == 70

    def test_method_parameter_overrides_decorator_default(self, manager, robot_id):
        """Test that method parameter overrides decorator default"""
        manager.set_decorator_default(robot_id, PlaybackSpeedPercent(70))
        speed = manager.get_effective_speed(robot_id, method_speed=PlaybackSpeedPercent(30))
        assert speed == 30

    def test_external_override_has_highest_precedence(self, manager, robot_id):
        """Test that external override takes precedence over all other settings"""
        manager.set_decorator_default(robot_id, PlaybackSpeedPercent(70))
        manager.set_external_override(robot_id, PlaybackSpeedPercent(20))

        # External override should win even with method parameter
        speed = manager.get_effective_speed(robot_id, method_speed=PlaybackSpeedPercent(90))
        assert speed == 20

    def test_pause_resume_functionality(self, manager, robot_id):
        """Test pause and resume state management"""
        # Initially playing
        assert manager.get_effective_state(robot_id) == PlaybackState.PLAYING

        # Pause
        manager.pause(robot_id)
        assert manager.get_effective_state(robot_id) == PlaybackState.PAUSED

        # Resume
        manager.resume(robot_id)
        assert manager.get_effective_state(robot_id) == PlaybackState.PLAYING

    def test_clear_external_override(self, manager, robot_id):
        """Test clearing external overrides falls back to lower precedence"""
        manager.set_decorator_default(robot_id, PlaybackSpeedPercent(60))
        manager.set_external_override(robot_id, PlaybackSpeedPercent(30))
        assert manager.get_effective_speed(robot_id) == 30

        manager.clear_external_override(robot_id)
        assert manager.get_effective_speed(robot_id) == 60

    def test_speed_validation(self, manager, robot_id):
        """Test that invalid speeds are rejected"""
        with pytest.raises(ValueError, match="Speed percent must be between 0 and 100"):
            manager.set_external_override(robot_id, PlaybackSpeedPercent(-10))

        with pytest.raises(ValueError, match="Speed percent must be between 0 and 100"):
            manager.set_external_override(robot_id, PlaybackSpeedPercent(110))

    def test_thread_safety(self, manager, robot_id):
        """Test that concurrent access is thread-safe"""
        results = []
        errors = []

        def set_speed(speed):
            try:
                manager.set_external_override(robot_id, PlaybackSpeedPercent(value=speed))
                result = manager.get_effective_speed(robot_id)
                results.append(result)
            except Exception as e:
                errors.append(e)

        # Run concurrent speed settings
        threads = []
        for i in range(10):
            speed = 10 * (i + 1)  # 10 to 100
            thread = threading.Thread(target=set_speed, args=(speed,))
            threads.append(thread)
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

        # Should have no errors
        assert len(errors) == 0

        # Should have 10 results
        assert len(results) == 10

        # Final speed should be one of the set values
        final_speed = manager.get_effective_speed(robot_id)
        expected_speeds = [10 * (i + 1) for i in range(10)]
        assert final_speed in expected_speeds

    def test_get_all_robots(self, manager):
        """Test getting list of robots with settings"""
        robot1 = "robot1"
        robot2 = "robot2"
        robot3 = "robot3"

        # Initially no robots
        assert len(manager.get_all_robots()) == 0

        # Add some robots
        manager.set_decorator_default(robot1, PlaybackSpeedPercent(50))
        manager.set_external_override(robot2, PlaybackSpeedPercent(30))
        manager.set_decorator_default(robot3, PlaybackSpeedPercent(80))

        robots = manager.get_all_robots()
        assert len(robots) == 3
        assert robot1 in robots
        assert robot2 in robots
        assert robot3 in robots


class TestPlaybackControlManagerSingleton:
    """Test the global singleton functionality"""

    def test_global_manager_singleton(self):
        """Test that get_playback_manager returns the same instance"""
        manager1 = get_playback_manager()
        manager2 = get_playback_manager()
        assert manager1 is manager2

    def test_global_manager_persistence(self):
        """Test that settings persist across get_playback_manager calls"""
        robot_id = "persistent_robot"

        manager1 = get_playback_manager()
        manager1.set_decorator_default(robot_id, PlaybackSpeedPercent(42))

        manager2 = get_playback_manager()
        speed = manager2.get_effective_speed(robot_id)
        assert speed == 42


class TestPlaybackControlDataClasses:
    """Test the data classes and types"""

    def test_playback_control_immutable(self):
        """Test that PlaybackControl is immutable"""
        control = PlaybackControl(
            speed=PlaybackSpeedPercent(50), source="external", state=PlaybackState.PAUSED
        )

        # Should not be able to modify - Pydantic frozen model raises ValidationError
        with pytest.raises(ValidationError):
            control.speed = PlaybackSpeedPercent(70)

    def test_playback_control_defaults(self):
        """Test PlaybackControl default values"""
        control = PlaybackControl(speed=PlaybackSpeedPercent(100), source="default")
        assert control.speed == 100
        assert control.source == "default"
        assert control.state is None  # Default is None, not PLAYING
        assert isinstance(control.set_at, datetime)

    def test_invalid_speed_error(self):
        """Test InvalidSpeedError functionality"""
        error = InvalidSpeedError("Speed must be 0-100%, got 150%")
        assert "150" in str(error)


@pytest.mark.parametrize("invalid_speed", [-10, 110, 200, 1000000, -1000000])
def test_speed_validation_parametrized(invalid_speed):
    """Test that various invalid speeds are rejected"""
    manager = PlaybackControlManager()
    robot_id = "test_robot"

    with pytest.raises(ValueError, match="Speed percent must be between 0 and 100"):
        manager.set_external_override(robot_id, PlaybackSpeedPercent(value=invalid_speed))


@pytest.mark.parametrize("valid_speed", [0, 10, 50, 90, 100])
def test_speed_validation_valid_speeds(valid_speed):
    """Test that valid speeds are accepted"""
    manager = PlaybackControlManager()
    robot_id = "test_robot"

    # Should not raise any exception
    manager.set_external_override(robot_id, PlaybackSpeedPercent(value=valid_speed))
    speed = manager.get_effective_speed(robot_id)
    assert speed == valid_speed
