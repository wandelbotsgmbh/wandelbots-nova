"""Tests for Nova Playback Control core functionality"""

import threading
from datetime import datetime

import pytest

from nova.core.playback_control import (
    InvalidSpeedError,
    PlaybackControl,
    PlaybackControlManager,
    PlaybackSpeed,
    PlaybackState,
    RobotId,
    get_playback_manager,
)


class TestPlaybackControlManager:
    """Test suite for PlaybackControlManager core functionality"""

    @pytest.fixture
    def manager(self):
        """Fresh manager instance for each test"""
        return PlaybackControlManager()

    @pytest.fixture
    def robot_id(self):
        """Standard robot ID for testing"""
        return RobotId("test_robot_1")

    def test_default_speed_is_one(self, manager, robot_id):
        """Test that default speed is 1.0 when no overrides are set"""
        speed = manager.get_effective_speed(robot_id)
        assert speed == 1.0

    def test_method_parameter_overrides_default(self, manager, robot_id):
        """Test that method parameter takes precedence over default"""
        speed = manager.get_effective_speed(robot_id, method_speed=PlaybackSpeed(0.5))
        assert speed == 0.5

    def test_decorator_default_overrides_system_default(self, manager, robot_id):
        """Test that decorator default takes precedence over system default"""
        manager.set_decorator_default(robot_id, PlaybackSpeed(0.7))
        speed = manager.get_effective_speed(robot_id)
        assert speed == 0.7

    def test_method_parameter_overrides_decorator_default(self, manager, robot_id):
        """Test that method parameter overrides decorator default"""
        manager.set_decorator_default(robot_id, PlaybackSpeed(0.7))
        speed = manager.get_effective_speed(robot_id, method_speed=PlaybackSpeed(0.3))
        assert speed == 0.3

    def test_external_override_has_highest_precedence(self, manager, robot_id):
        """Test that external override takes precedence over all other settings"""
        manager.set_decorator_default(robot_id, PlaybackSpeed(0.7))
        manager.set_external_override(robot_id, PlaybackSpeed(0.2))

        # External override should win even with method parameter
        speed = manager.get_effective_speed(robot_id, method_speed=PlaybackSpeed(0.9))
        assert speed == 0.2

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
        manager.set_decorator_default(robot_id, PlaybackSpeed(0.6))
        manager.set_external_override(robot_id, PlaybackSpeed(0.3))
        assert manager.get_effective_speed(robot_id) == 0.3

        manager.clear_external_override(robot_id)
        assert manager.get_effective_speed(robot_id) == 0.6

    def test_speed_validation(self, manager, robot_id):
        """Test that invalid speeds are rejected"""
        with pytest.raises(ValueError, match="Speed must be between 0.0 and 1.0"):
            manager.set_external_override(robot_id, PlaybackSpeed(-0.1))

        with pytest.raises(ValueError, match="Speed must be between 0.0 and 1.0"):
            manager.set_external_override(robot_id, PlaybackSpeed(1.1))

    def test_thread_safety(self, manager, robot_id):
        """Test that concurrent access is thread-safe"""
        results = []
        errors = []

        def set_speed(speed):
            try:
                manager.set_external_override(robot_id, PlaybackSpeed(speed))
                result = manager.get_effective_speed(robot_id)
                results.append(result)
            except Exception as e:
                errors.append(e)

        # Run concurrent speed settings
        threads = []
        for i in range(10):
            speed = 0.1 * (i + 1)  # 0.1 to 1.0
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
        expected_speeds = [0.1 * (i + 1) for i in range(10)]
        assert final_speed in expected_speeds

    def test_get_all_robots(self, manager):
        """Test getting list of robots with settings"""
        robot1 = RobotId("robot1")
        robot2 = RobotId("robot2")
        robot3 = RobotId("robot3")

        # Initially no robots
        assert len(manager.get_all_robots()) == 0

        # Add some robots
        manager.set_decorator_default(robot1, PlaybackSpeed(0.5))
        manager.set_external_override(robot2, PlaybackSpeed(0.3))
        manager.set_decorator_default(robot3, PlaybackSpeed(0.8))

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
        robot_id = RobotId("persistent_robot")

        manager1 = get_playback_manager()
        manager1.set_decorator_default(robot_id, PlaybackSpeed(0.42))

        manager2 = get_playback_manager()
        speed = manager2.get_effective_speed(robot_id)
        assert speed == 0.42


class TestPlaybackControlDataClasses:
    """Test the data classes and types"""

    def test_playback_control_immutable(self):
        """Test that PlaybackControl is immutable"""
        control = PlaybackControl(speed=PlaybackSpeed(0.5), state=PlaybackState.PAUSED)

        # Should not be able to modify
        with pytest.raises(AttributeError):
            control.speed = PlaybackSpeed(0.7)  # type: ignore

    def test_playback_control_defaults(self):
        """Test PlaybackControl default values"""
        control = PlaybackControl()
        assert control.speed == 1.0
        assert control.state == PlaybackState.PLAYING
        assert control.source == "default"
        assert isinstance(control.timestamp, datetime)

    def test_invalid_speed_error(self):
        """Test InvalidSpeedError functionality"""
        error = InvalidSpeedError(1.5)
        assert "1.5" in str(error)
        assert error.requested_speed == 1.5
        assert isinstance(error.timestamp, datetime)


@pytest.mark.parametrize("invalid_speed", [-0.1, 1.1, 2.0, float("inf"), float("-inf")])
def test_speed_validation_parametrized(invalid_speed):
    """Test that various invalid speeds are rejected"""
    manager = PlaybackControlManager()
    robot_id = RobotId("test_robot")

    with pytest.raises(ValueError, match="Speed must be between 0.0 and 1.0"):
        manager.set_external_override(robot_id, PlaybackSpeed(invalid_speed))


@pytest.mark.parametrize("valid_speed", [0.0, 0.1, 0.5, 0.9, 1.0])
def test_speed_validation_valid_speeds(valid_speed):
    """Test that valid speeds are accepted"""
    manager = PlaybackControlManager()
    robot_id = RobotId("test_robot")

    # Should not raise any exception
    manager.set_external_override(robot_id, PlaybackSpeed(valid_speed))
    speed = manager.get_effective_speed(robot_id)
    assert speed == valid_speed
