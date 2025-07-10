"""Nova Playback Control System

This module provides centralized playback speed control and pause/resume functionality
for Nova robot executions. It implements a clear precedence hierarchy:

1. External override (e.g., from external integrations)
2. Method parameter (playback_speed=0.5)
3. Decorator default (@nova.program(playback_speed_percent=30))
4. System default (100% speed)
"""

import threading
from datetime import datetime, timezone
from enum import Enum
from typing import Callable, Literal, NewType, Optional

from pydantic import BaseModel, Field

from nova.core import logger

# Type-safe identifiers
MotionGroupId = NewType("MotionGroupId", str)
PlaybackSpeedPercent = NewType("PlaybackSpeedPercent", int)


def motion_group_id(id_str: str | MotionGroupId) -> MotionGroupId:
    """Convert string or MotionGroupId to MotionGroupId

    Args:
        id_str: String ID from motion_group.motion_group_id or existing MotionGroupId

    Returns:
        MotionGroupId for use with playback control functions

    Example:
        # Both of these work:
        mgid = motion_group_id(motion_group.motion_group_id)  # str
        mgid = motion_group_id(existing_motion_group_id)      # MotionGroupId
    """
    return MotionGroupId(id_str)


# Control source types with strict validation
PlaybackSourceType = Literal["external", "method", "decorator", "default"]


class PlaybackState(Enum):
    """Robot execution state for pause/resume control"""

    PLAYING = "playing"
    PAUSED = "paused"
    EXECUTING = "executing"
    IDLE = "idle"


class PlaybackDirection(Enum):
    """Robot execution direction for forward/backward control"""

    FORWARD = "forward"
    BACKWARD = "backward"


# State change callback type
StateChangeCallback = Callable[
    [MotionGroupId, PlaybackState, PlaybackSpeedPercent, PlaybackDirection], None
]


class PlaybackControl(BaseModel):
    """Immutable playback control configuration"""

    model_config = {"frozen": True}

    speed: PlaybackSpeedPercent = PlaybackSpeedPercent(100)
    state: PlaybackState = PlaybackState.PLAYING
    direction: PlaybackDirection = PlaybackDirection.FORWARD
    source: PlaybackSourceType = "default"
    timestamp: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class PlaybackControlManager:
    """Centralized playback control with precedence resolution and state notifications

    This manager maintains state for all robots and resolves playback speed
    and state based on a clear precedence hierarchy. Thread-safe for
    concurrent access from multiple sources.
    """

    def __init__(self):
        """Initialize with thread-safe data structures"""
        self._external_overrides: dict[MotionGroupId, PlaybackControl] = {}
        self._decorator_defaults: dict[MotionGroupId, PlaybackSpeedPercent] = {}
        self._execution_states: dict[MotionGroupId, PlaybackState] = {}
        self._state_callbacks: list[StateChangeCallback] = []
        self._lock = threading.Lock()

    def set_external_override(
        self,
        motion_group_id: str | MotionGroupId,
        speed: PlaybackSpeedPercent,
        state: PlaybackState = PlaybackState.PLAYING,
        direction: PlaybackDirection = PlaybackDirection.FORWARD,
    ) -> None:
        """Set external override (highest precedence)

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
            speed: Playback speed percent (0-100)
            state: Playback state (playing/paused)
            direction: Playback direction (forward/backward)

        Raises:
            ValueError: If speed is outside valid range
        """
        self._validate_speed(speed)
        mgid = MotionGroupId(motion_group_id)

        with self._lock:
            self._external_overrides[mgid] = PlaybackControl(
                speed=speed,
                state=state,
                direction=direction,
                source="external",
                timestamp=datetime.now(timezone.utc),
            )
        # Notify outside of lock to avoid deadlock
        self._notify_state_change(mgid)

    def set_decorator_default(
        self, motion_group_id: str | MotionGroupId, speed: PlaybackSpeedPercent
    ) -> None:
        """Set decorator default speed (lower precedence)

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
            speed: Default playback speed percent (0-100)

        Raises:
            ValueError: If speed is outside valid range
        """
        self._validate_speed(speed)
        mgid = MotionGroupId(motion_group_id)

        with self._lock:
            self._decorator_defaults[mgid] = speed

    def get_decorator_default(
        self, motion_group_id: str | MotionGroupId
    ) -> Optional[PlaybackSpeedPercent]:
        """Get decorator default speed for a motion group

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)

        Returns:
            Decorator default speed if set, None otherwise
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            return self._decorator_defaults.get(mgid)

    def _get_effective_speed_locked(
        self, motion_group_id: MotionGroupId, method_speed: Optional[PlaybackSpeedPercent] = None
    ) -> PlaybackSpeedPercent:
        """Get effective playback speed percent without acquiring lock (internal use only)"""
        # 1. External override (highest precedence)
        if motion_group_id in self._external_overrides:
            return self._external_overrides[motion_group_id].speed

        # 2. Method parameter
        if method_speed is not None:
            self._validate_speed(method_speed)
            return method_speed

        # 3. Decorator default
        if motion_group_id in self._decorator_defaults:
            return self._decorator_defaults[motion_group_id]

        # 4. System default
        return PlaybackSpeedPercent(100)

    def get_effective_speed(
        self,
        motion_group_id: str | MotionGroupId,
        method_speed: Optional[PlaybackSpeedPercent] = None,
    ) -> PlaybackSpeedPercent:
        """Get effective speed percent with precedence resolution

        Precedence (highest to lowest):
        1. External override  2. Method parameter  3. Decorator default  4. System default (100)
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            return self._get_effective_speed_locked(mgid, method_speed)

    def get_effective_state(self, motion_group_id: str | MotionGroupId) -> PlaybackState:
        """Get effective playback state (playing/paused)"""
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            if mgid in self._external_overrides:
                return self._external_overrides[mgid].state
            return PlaybackState.PLAYING

    def get_effective_direction(self, motion_group_id: str | MotionGroupId) -> PlaybackDirection:
        """Get effective playback direction (forward/backward)"""
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            if mgid in self._external_overrides:
                return self._external_overrides[mgid].direction
            return PlaybackDirection.FORWARD

    def pause(self, motion_group_id: str | MotionGroupId) -> None:
        """Pause execution (external control only)"""
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            effective_speed = self._get_effective_speed_locked(mgid)
            current_control = self._external_overrides.get(
                mgid, PlaybackControl(speed=effective_speed)
            )
            self._external_overrides[mgid] = PlaybackControl(
                speed=current_control.speed,
                state=PlaybackState.PAUSED,
                direction=current_control.direction,
                source="external",
                timestamp=datetime.now(timezone.utc),
            )
        self._notify_state_change(mgid)

    def resume(self, motion_group_id: str | MotionGroupId) -> None:
        """Resume execution (external control only)"""
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            effective_speed = self._get_effective_speed_locked(mgid)
            current_control = self._external_overrides.get(
                mgid, PlaybackControl(speed=effective_speed)
            )
            self._external_overrides[mgid] = PlaybackControl(
                speed=current_control.speed,
                state=PlaybackState.PLAYING,
                direction=current_control.direction,
                source="external",
                timestamp=datetime.now(timezone.utc),
            )
        self._notify_state_change(mgid)

    def set_direction_forward(self, motion_group_id: str | MotionGroupId) -> None:
        """Set execution direction to forward"""
        self._set_direction(motion_group_id, PlaybackDirection.FORWARD)

    def set_direction_backward(self, motion_group_id: str | MotionGroupId) -> None:
        """Set execution direction to backward"""
        self._set_direction(motion_group_id, PlaybackDirection.BACKWARD)

    def _set_direction(
        self, motion_group_id: str | MotionGroupId, direction: PlaybackDirection
    ) -> None:
        """Helper method to set direction"""
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            effective_speed = self._get_effective_speed_locked(mgid)
            current_control = self._external_overrides.get(
                mgid, PlaybackControl(speed=effective_speed)
            )
            self._external_overrides[mgid] = PlaybackControl(
                speed=current_control.speed,
                state=current_control.state,
                direction=direction,
                source="external",
                timestamp=datetime.now(timezone.utc),
            )
        self._notify_state_change(mgid)

    def clear_external_override(self, motion_group_id: str | MotionGroupId) -> None:
        """Remove external override, falling back to lower-priority settings"""
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            self._external_overrides.pop(mgid, None)

    def get_all_robots(self) -> list[MotionGroupId]:
        """Get list of all robots with any playback settings"""
        with self._lock:
            all_robots: set[MotionGroupId] = set()
            all_robots.update(self._external_overrides.keys())
            all_robots.update(self._decorator_defaults.keys())
            return list(all_robots)

    def _validate_speed(self, speed: PlaybackSpeedPercent) -> None:
        """Validate speed percent is in valid range"""
        if not (0 <= speed <= 100):
            raise ValueError(f"Speed percent must be between 0 and 100, got {speed}")

    def set_execution_state(
        self, motion_group_id: str | MotionGroupId, state: PlaybackState
    ) -> None:
        """Set the execution state for a motion group (used by movement controller)"""
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            self._execution_states[mgid] = state

    def get_execution_state(self, motion_group_id: str | MotionGroupId) -> PlaybackState:
        """Get the execution state for a motion group"""
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            return self._execution_states.get(mgid, PlaybackState.IDLE)

    def is_movement_active(self, motion_group_id: str | MotionGroupId) -> bool:
        """Check if movement is actively executing for a motion group"""
        return self.get_execution_state(motion_group_id) == PlaybackState.EXECUTING

    def can_pause(self, motion_group_id: str | MotionGroupId) -> bool:
        """Check if motion group can be paused (movement is active and not already paused)"""
        execution_state = self.get_execution_state(motion_group_id)
        playback_state = self.get_effective_state(motion_group_id)
        return (
            execution_state == PlaybackState.EXECUTING and playback_state == PlaybackState.PLAYING
        )

    def can_resume(self, motion_group_id: str | MotionGroupId) -> bool:
        """Check if motion group can be resumed (movement is paused)"""
        return self.get_effective_state(motion_group_id) == PlaybackState.PAUSED

    def _notify_state_change(self, motion_group_id: MotionGroupId) -> None:
        """Notify registered callbacks of a state change"""
        control = None
        callbacks = None

        with self._lock:
            if motion_group_id in self._external_overrides:
                control = self._external_overrides[motion_group_id]
            callbacks = self._state_callbacks.copy()

        # Call callbacks outside of lock to avoid deadlock
        if control and callbacks:
            for callback in callbacks:
                try:
                    callback(motion_group_id, control.state, control.speed, control.direction)
                except Exception as e:
                    logger.warning(f"State change callback error: {e}")

    def register_state_change_callback(self, callback: StateChangeCallback) -> None:
        """Register a callback for state changes"""
        with self._lock:
            self._state_callbacks.append(callback)

    def unregister_state_change_callback(self, callback: StateChangeCallback) -> None:
        """Unregister a callback for state changes"""
        with self._lock:
            self._state_callbacks.remove(callback)


class PlaybackControlError(Exception):
    """Base exception for playback control errors"""

    def __init__(self, message: str, requested_speed: Optional[int] = None):
        super().__init__(message)
        self.timestamp = datetime.now(timezone.utc)
        self.requested_speed = requested_speed


class InvalidSpeedError(PlaybackControlError):
    """Raised when an invalid speed value is provided"""

    def __init__(self, speed: int):
        super().__init__(
            f"Invalid playback speed percent: {speed}. Speed percent must be between 0 and 100",
            requested_speed=speed,
        )


# Global instance - singleton pattern for system-wide state management
_playback_manager = PlaybackControlManager()

# Global registry for tracking active program's playback speed percent
_active_program_playback_speed_percent: int | None = None


def get_playback_manager() -> PlaybackControlManager:
    """Get the global playback control manager instance"""
    return _playback_manager


def set_active_program_playback_speed_percent(speed_percent: int) -> None:
    """Set the active program's default playback speed percent

    Called by the program decorator when a program starts.
    """
    global _active_program_playback_speed_percent
    _active_program_playback_speed_percent = speed_percent


def get_active_program_playback_speed_percent() -> int | None:
    """Get the active program's default playback speed percent"""
    return _active_program_playback_speed_percent


def clear_active_program_playback_speed_percent() -> None:
    """Clear the active program's playback speed percent

    Called when a program ends.
    """
    global _active_program_playback_speed_percent
    _active_program_playback_speed_percent = None


# External API functions for integrations
def get_all_active_robots() -> list[MotionGroupId]:
    """Get list of all robots with any settings (active or configured)"""
    manager = get_playback_manager()
    with manager._lock:
        all_robots: set[MotionGroupId] = set()
        all_robots.update(manager._external_overrides.keys())
        all_robots.update(manager._decorator_defaults.keys())
        all_robots.update(manager._execution_states.keys())
        return list(all_robots)


def get_robot_status_summary() -> dict:
    """Get a summary of all robot statuses for external UIs"""
    active_robots = get_all_active_robots()
    manager = get_playback_manager()
    executing_robots = [
        robot
        for robot in active_robots
        if manager.get_execution_state(robot) == PlaybackState.EXECUTING
    ]

    return {
        "total_robots": len(active_robots),
        "executing_robots": len(executing_robots),
        "motion_group_ids": active_robots,
        "executing_motion_group_ids": executing_robots,
        "has_robots": len(active_robots) > 0,
    }
