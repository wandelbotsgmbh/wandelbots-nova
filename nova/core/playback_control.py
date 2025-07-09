"""Nova Playback Control System

This module provides centralized playback speed control and pause/resume functionality
for Nova robot executions. It implements a clear precedence hierarchy:

1. External override (VS Code extension, future WebSocket/HTTP)
2. Method parameter (playback_speed=0.5)
3. Decorator default (@nova.program(playback_speed=0.3))
4. System default (1.0 = 100% speed)
"""

import threading
from datetime import datetime, timezone
from enum import Enum
from typing import Literal, NewType, Optional

from pydantic import BaseModel, Field

# Type-safe identifiers
RobotId = NewType("RobotId", str)
PlaybackSpeed = NewType("PlaybackSpeed", float)

# Control source types with strict validation
PlaybackSourceType = Literal["external", "method", "decorator", "default"]


class PlaybackState(Enum):
    """Robot execution state for pause/resume control"""

    PLAYING = "playing"
    PAUSED = "paused"


class PlaybackControl(BaseModel):
    """Immutable playback control configuration"""

    model_config = {"frozen": True}

    speed: PlaybackSpeed = PlaybackSpeed(1.0)
    state: PlaybackState = PlaybackState.PLAYING
    source: PlaybackSourceType = "default"
    timestamp: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class PlaybackControlManager:
    """Centralized playback control with precedence resolution

    This manager maintains state for all robots and resolves playback speed
    and state based on a clear precedence hierarchy. Thread-safe for
    concurrent access from multiple sources.
    """

    def __init__(self):
        """Initialize with thread-safe data structures"""
        self._external_overrides: dict[RobotId, PlaybackControl] = {}
        self._decorator_defaults: dict[RobotId, PlaybackSpeed] = {}
        self._lock = threading.Lock()

    def set_external_override(
        self, robot_id: RobotId, speed: PlaybackSpeed, state: PlaybackState = PlaybackState.PLAYING
    ) -> None:
        """Set external override (highest precedence)

        Args:
            robot_id: Unique robot identifier
            speed: Playback speed (0.0-1.0)
            state: Playback state (playing/paused)

        Raises:
            ValueError: If speed is outside valid range
        """
        self._validate_speed(speed)

        with self._lock:
            self._external_overrides[robot_id] = PlaybackControl(
                speed=speed, state=state, source="external", timestamp=datetime.now(timezone.utc)
            )

    def set_decorator_default(self, robot_id: RobotId, speed: PlaybackSpeed) -> None:
        """Set decorator default speed (lower precedence)

        Args:
            robot_id: Unique robot identifier
            speed: Default playback speed (0.0-1.0)

        Raises:
            ValueError: If speed is outside valid range
        """
        self._validate_speed(speed)

        with self._lock:
            self._decorator_defaults[robot_id] = speed

    def _get_effective_speed_locked(
        self, robot_id: RobotId, method_speed: Optional[PlaybackSpeed] = None
    ) -> PlaybackSpeed:
        """Get effective playback speed without acquiring lock (internal use only)

        Args:
            robot_id: Unique robot identifier
            method_speed: Speed from method parameter

        Returns:
            Effective playback speed (0.0-1.0)
        """
        # 1. External override (highest precedence)
        if robot_id in self._external_overrides:
            return self._external_overrides[robot_id].speed

        # 2. Method parameter
        if method_speed is not None:
            self._validate_speed(method_speed)
            return method_speed

        # 3. Decorator default
        if robot_id in self._decorator_defaults:
            return self._decorator_defaults[robot_id]

        # 4. System default
        return PlaybackSpeed(1.0)

    def get_effective_speed(
        self, robot_id: RobotId, method_speed: Optional[PlaybackSpeed] = None
    ) -> PlaybackSpeed:
        """Get effective speed with precedence resolution

        Precedence (highest to lowest):
        1. External override
        2. Method parameter
        3. Decorator default
        4. System default (1.0)

        Args:
            robot_id: Unique robot identifier
            method_speed: Speed from method parameter

        Returns:
            Effective playback speed (0.0-1.0)
        """
        with self._lock:
            return self._get_effective_speed_locked(robot_id, method_speed)

    def get_effective_state(self, robot_id: RobotId) -> PlaybackState:
        """Get effective playback state (playing/paused)

        Only external overrides can change state from default PLAYING.

        Args:
            robot_id: Unique robot identifier

        Returns:
            Current playback state
        """
        with self._lock:
            if robot_id in self._external_overrides:
                return self._external_overrides[robot_id].state
            return PlaybackState.PLAYING

    def pause(self, robot_id: RobotId) -> None:
        """Pause execution (external control only)

        Args:
            robot_id: Unique robot identifier
        """
        with self._lock:
            # Get current effective speed without recursive lock
            effective_speed = self._get_effective_speed_locked(robot_id)
            current_control = self._external_overrides.get(
                robot_id, PlaybackControl(speed=effective_speed)
            )
            self._external_overrides[robot_id] = PlaybackControl(
                speed=current_control.speed,
                state=PlaybackState.PAUSED,
                source="external",
                timestamp=datetime.now(timezone.utc),
            )

    def resume(self, robot_id: RobotId) -> None:
        """Resume execution (external control only)

        Args:
            robot_id: Unique robot identifier
        """
        with self._lock:
            effective_speed = self._get_effective_speed_locked(robot_id)
            current_control = self._external_overrides.get(
                robot_id, PlaybackControl(speed=effective_speed)
            )
            self._external_overrides[robot_id] = PlaybackControl(
                speed=current_control.speed,
                state=PlaybackState.PLAYING,
                source="external",
                timestamp=datetime.now(timezone.utc),
            )

    def clear_external_override(self, robot_id: RobotId) -> None:
        """Remove external override, falling back to lower-priority settings

        Args:
            robot_id: Unique robot identifier
        """
        with self._lock:
            self._external_overrides.pop(robot_id, None)

    def get_all_robots(self) -> list[RobotId]:
        """Get list of all robots with any playback settings

        Returns:
            List of robot IDs that have playback settings
        """
        with self._lock:
            all_robots: set[RobotId] = set()
            all_robots.update(self._external_overrides.keys())
            all_robots.update(self._decorator_defaults.keys())
            return list(all_robots)

    def _validate_speed(self, speed: PlaybackSpeed) -> None:
        """Validate speed is in valid range

        Args:
            speed: Speed value to validate

        Raises:
            ValueError: If speed is outside valid range
        """
        if not (0.0 <= speed <= 1.0):
            raise ValueError(f"Speed must be between 0.0 and 1.0, got {speed}")


class PlaybackControlError(Exception):
    """Base exception for playback control errors"""

    def __init__(self, message: str, requested_speed: Optional[float] = None):
        super().__init__(message)
        self.timestamp = datetime.now(timezone.utc)
        self.requested_speed = requested_speed


class InvalidSpeedError(PlaybackControlError):
    """Raised when an invalid speed value is provided"""

    def __init__(self, speed: float):
        super().__init__(
            f"Invalid playback speed: {speed}. Speed must be between 0.0 and 1.0",
            requested_speed=speed,
        )


# Global instance - singleton pattern for system-wide state management
_playback_manager = PlaybackControlManager()


def get_playback_manager() -> PlaybackControlManager:
    """Get the global playback control manager instance

    Returns:
        Global PlaybackControlManager instance
    """
    return _playback_manager
