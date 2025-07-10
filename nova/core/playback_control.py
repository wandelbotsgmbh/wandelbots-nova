"""Nova Playback Control System

This module provides centralized playback speed control and pause/resume functionality
for Nova robot executions. It implements a clear precedence hierarchy:

1. External override (VS Code extension, future WebSocket/HTTP)
2. Method parameter (playback_speed=0.5)
3. Decorator default (@nova.program(playback_speed_percent=30))
4. System default (1.0 = 100% speed)
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

# Convenience type alias that accepts both str and MotionGroupId
RobotId = MotionGroupId


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


# State change callback type (defined after enums to avoid forward reference issues)
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

    Supports state change callbacks for VS Code extension integration.
    """

    def __init__(self):
        """Initialize with thread-safe data structures and callback support"""
        self._external_overrides: dict[MotionGroupId, PlaybackControl] = {}
        self._decorator_defaults: dict[MotionGroupId, PlaybackSpeedPercent] = {}
        self._execution_states: dict[
            MotionGroupId, PlaybackState
        ] = {}  # Track actual execution state
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
        """Get effective playback speed percent without acquiring lock (internal use only)

        Args:
            motion_group_id: Motion group identifier
            method_speed: Speed percent from method parameter

        Returns:
            Effective playback speed percent (0-100)
        """
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
        1. External override
        2. Method parameter
        3. Decorator default
        4. System default (100)

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
            method_speed: Speed percent from method parameter

        Returns:
            Effective playback speed percent (0-100)
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            return self._get_effective_speed_locked(mgid, method_speed)

    def get_effective_state(self, motion_group_id: str | MotionGroupId) -> PlaybackState:
        """Get effective playback state (playing/paused)

        Only external overrides can change state from default PLAYING.

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)

        Returns:
            Current playback state
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            if mgid in self._external_overrides:
                return self._external_overrides[mgid].state
            return PlaybackState.PLAYING

    def get_effective_direction(self, motion_group_id: str | MotionGroupId) -> PlaybackDirection:
        """Get effective playback direction (forward/backward)

        Only external overrides can change direction from default FORWARD.

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)

        Returns:
            Current playback direction
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            if mgid in self._external_overrides:
                return self._external_overrides[mgid].direction
            return PlaybackDirection.FORWARD

    def pause(self, motion_group_id: str | MotionGroupId) -> None:
        """Pause execution (external control only)

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            # Get current effective speed without recursive lock
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
        # Notify outside of lock to avoid deadlock
        self._notify_state_change(mgid)

    def resume(self, motion_group_id: str | MotionGroupId) -> None:
        """Resume execution (external control only)

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
        """
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
        # Notify outside of lock to avoid deadlock
        self._notify_state_change(mgid)

    def set_direction_forward(self, motion_group_id: str | MotionGroupId) -> None:
        """Set execution direction to forward (external control only)

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            effective_speed = self._get_effective_speed_locked(mgid)
            current_control = self._external_overrides.get(
                mgid, PlaybackControl(speed=effective_speed)
            )
            self._external_overrides[mgid] = PlaybackControl(
                speed=current_control.speed,
                state=current_control.state,
                direction=PlaybackDirection.FORWARD,
                source="external",
                timestamp=datetime.now(timezone.utc),
            )
        # Notify outside of lock to avoid deadlock
        self._notify_state_change(mgid)

    def set_direction_backward(self, motion_group_id: str | MotionGroupId) -> None:
        """Set execution direction to backward (external control only)

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            effective_speed = self._get_effective_speed_locked(mgid)
            current_control = self._external_overrides.get(
                mgid, PlaybackControl(speed=effective_speed)
            )
            self._external_overrides[mgid] = PlaybackControl(
                speed=current_control.speed,
                state=current_control.state,
                direction=PlaybackDirection.BACKWARD,
                source="external",
                timestamp=datetime.now(timezone.utc),
            )
        # Notify outside of lock to avoid deadlock
        self._notify_state_change(mgid)

    def clear_external_override(self, motion_group_id: str | MotionGroupId) -> None:
        """Remove external override, falling back to lower-priority settings

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            self._external_overrides.pop(mgid, None)

    def get_all_robots(self) -> list[MotionGroupId]:
        """Get list of all robots with any playback settings

        Returns:
            List of robot IDs that have playback settings
        """
        with self._lock:
            all_robots: set[MotionGroupId] = set()
            all_robots.update(self._external_overrides.keys())
            all_robots.update(self._decorator_defaults.keys())
            return list(all_robots)

    def _validate_speed(self, speed: PlaybackSpeedPercent) -> None:
        """Validate speed percent is in valid range

        Args:
            speed: Speed percent value to validate

        Raises:
            ValueError: If speed is outside valid range
        """
        if not (0 <= speed <= 100):
            raise ValueError(f"Speed percent must be between 0 and 100, got {speed}")

    def set_execution_state(
        self, motion_group_id: str | MotionGroupId, state: PlaybackState
    ) -> None:
        """Set the execution state for a motion group (used by movement controller)

        This tracks whether a motion group is actually executing movement vs idle.
        Used to enable/disable pause buttons in VS Code extension.

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)
            state: Execution state (EXECUTING, IDLE, etc.)
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            self._execution_states[mgid] = state
            # Note: We don't notify callbacks for execution state changes
            # since these are internal state changes, not user-initiated playback changes

    def get_execution_state(self, motion_group_id: str | MotionGroupId) -> PlaybackState:
        """Get the execution state for a motion group

        This tells you whether the motion group is actively executing movement.
        Use this to enable/disable pause buttons in your VS Code extension.

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)

        Returns:
            Current execution state (EXECUTING/IDLE)
        """
        mgid = MotionGroupId(motion_group_id)
        with self._lock:
            return self._execution_states.get(mgid, PlaybackState.IDLE)

    def is_movement_active(self, motion_group_id: str | MotionGroupId) -> bool:
        """Check if movement is actively executing for a motion group

        Convenience method for VS Code extension UI logic.

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)

        Returns:
            True if movement is executing and pause button should be enabled
        """
        execution_state = self.get_execution_state(motion_group_id)
        return execution_state == PlaybackState.EXECUTING

    def can_pause(self, motion_group_id: str | MotionGroupId) -> bool:
        """Check if motion group can be paused (movement is active and not already paused)

        Use this to enable/disable pause button in VS Code extension.

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)

        Returns:
            True if pause button should be enabled
        """
        execution_state = self.get_execution_state(motion_group_id)
        playback_state = self.get_effective_state(motion_group_id)
        return (
            execution_state == PlaybackState.EXECUTING and playback_state == PlaybackState.PLAYING
        )

    def can_resume(self, motion_group_id: str | MotionGroupId) -> bool:
        """Check if motion group can be resumed (movement is paused)

        Use this to enable/disable play forward/backward buttons in VS Code extension.

        Args:
            motion_group_id: Motion group identifier (string or MotionGroupId)

        Returns:
            True if play buttons should be enabled
        """
        playback_state = self.get_effective_state(motion_group_id)
        return playback_state == PlaybackState.PAUSED

    def _notify_state_change(self, motion_group_id: MotionGroupId) -> None:
        """Notify registered callbacks of a state change

        Args:
            motion_group_id: Motion group identifier
        """
        # Get the current state without holding the lock during callback execution
        control = None
        callbacks = None

        with self._lock:
            if motion_group_id in self._external_overrides:
                control = self._external_overrides[motion_group_id]
            callbacks = self._state_callbacks.copy()  # Copy to avoid lock during iteration

        # Call callbacks outside of lock to avoid deadlock
        if control and callbacks:
            for callback in callbacks:
                try:
                    callback(motion_group_id, control.state, control.speed, control.direction)
                except Exception as e:
                    # Don't let callback errors break the system
                    logger.warning(f"State change callback error: {e}")

    def register_state_change_callback(self, callback: StateChangeCallback) -> None:
        """Register a callback for state changes

        Args:
            callback: Callable that receives robot_id, state, speed, and direction
        """
        with self._lock:
            self._state_callbacks.append(callback)

    def unregister_state_change_callback(self, callback: StateChangeCallback) -> None:
        """Unregister a callback for state changes

        Args:
            callback: Callable to remove from the registry
        """
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
    """Get the global playback control manager instance

    Returns:
        Global PlaybackControlManager instance
    """
    return _playback_manager


def set_active_program_playback_speed_percent(speed_percent: int) -> None:
    """Set the active program's default playback speed percent

    This is called by the program decorator when a program starts.

    Args:
        speed_percent: Default playback speed percent for the program (0-100)
    """
    global _active_program_playback_speed_percent
    _active_program_playback_speed_percent = speed_percent


def get_active_program_playback_speed_percent() -> int | None:
    """Get the active program's default playback speed percent

    Returns:
        Active program's default playback speed percent if set, None otherwise
    """
    return _active_program_playback_speed_percent


def clear_active_program_playback_speed_percent() -> None:
    """Clear the active program's playback speed percent

    Called when a program ends.
    """
    global _active_program_playback_speed_percent
    _active_program_playback_speed_percent = None


# External API functions for VS Code extension integration


def get_all_active_robots() -> list[MotionGroupId]:
    """Get list of all currently active robots

    This is for VS Code extensions to discover robots independently.

    Returns:
        List of robot IDs that are currently active/moving
    """
    manager = get_playback_manager()
    active_robots = []

    # Check all known robots for activity
    for robot_id in manager._external_overrides.keys():
        if manager.is_movement_active(robot_id):
            active_robots.append(robot_id)

    # Also check execution states
    for robot_id in manager._execution_states.keys():
        if (
            manager.get_execution_state(robot_id) == PlaybackState.EXECUTING
            and robot_id not in active_robots
        ):
            active_robots.append(robot_id)

    return active_robots


def get_robot_info(motion_group_id: str | MotionGroupId) -> dict:
    """Get complete information about a motion group's state

    This is for VS Code extensions to get all motion group state data.

    Args:
        motion_group_id: The motion group to get info for (string or MotionGroupId)

    Returns:
        Dictionary with complete motion group state information
    """
    manager = get_playback_manager()
    mgid = MotionGroupId(motion_group_id)

    return {
        "motion_group_id": mgid,
        "execution_state": manager.get_execution_state(mgid),
        "playback_state": manager.get_effective_state(mgid),
        "speed": manager.get_effective_speed(mgid),
        "direction": manager.get_effective_direction(mgid),
        "is_moving": manager.is_movement_active(mgid),
        "can_pause": manager.can_pause(mgid),
        "can_resume": manager.can_resume(mgid),
    }


def register_global_state_change_callback(callback) -> None:
    """Register a callback for ALL robot state changes

    This is for VS Code extensions to monitor all robots globally.
    The callback will be called for any robot state change.

    Args:
        callback: Function to call when any robot state changes
                 Signature: callback(robot_id, state, speed, direction)
    """
    manager = get_playback_manager()
    manager.register_state_change_callback(callback)


def unregister_global_state_change_callback(callback) -> None:
    """Unregister a global state change callback

    Args:
        callback: The callback function to unregister
    """
    manager = get_playback_manager()
    manager.unregister_state_change_callback(callback)


def pause_robot(robot_id: MotionGroupId) -> bool:
    """Pause a specific robot

    Args:
        robot_id: The robot to pause

    Returns:
        True if pause was successful, False otherwise
    """
    manager = get_playback_manager()
    if manager.can_pause(robot_id):
        manager.pause(robot_id)
        return True
    return False


def resume_robot(robot_id: MotionGroupId) -> bool:
    """Resume a specific robot

    Args:
        robot_id: The robot to resume

    Returns:
        True if resume was successful, False otherwise
    """
    manager = get_playback_manager()
    if manager.can_resume(robot_id):
        manager.resume(robot_id)
        return True
    return False


def set_robot_direction_forward(robot_id: MotionGroupId) -> None:
    """Set robot direction to forward

    Args:
        robot_id: The robot to set direction for
    """
    manager = get_playback_manager()
    manager.set_direction_forward(robot_id)


def set_robot_direction_backward(robot_id: MotionGroupId) -> None:
    """Set robot direction to backward

    Args:
        robot_id: The robot to set direction for
    """
    manager = get_playback_manager()
    manager.set_direction_backward(robot_id)


def set_robot_speed(robot_id: MotionGroupId, speed: int) -> None:
    """Set robot speed

    Args:
        robot_id: The robot to set speed for
        speed: Speed percentage (0-100)
    """
    manager = get_playback_manager()
    current_state = manager.get_effective_state(robot_id)
    current_direction = manager.get_effective_direction(robot_id)

    manager.set_external_override(
        robot_id, PlaybackSpeedPercent(speed), current_state, current_direction
    )


def get_primary_robot_id() -> Optional[MotionGroupId]:
    """Get the primary/main robot ID if one exists

    This is a convenience function for VS Code extensions that want to
    automatically control the "main" robot without manual selection.

    Returns the first robot ID that has been active, or None if no robots.

    Returns:
        Primary robot ID if available, None otherwise
    """
    active_robots = get_all_active_robots()
    if active_robots:
        return active_robots[0]  # Return first robot as primary
    return None


def get_currently_executing_robots() -> list[MotionGroupId]:
    """Get list of robots that are currently executing movement

    This helps VS Code extensions focus on robots that are actively moving
    and would benefit from pause/speed control.

    Returns:
        List of robot IDs that are currently executing movement
    """
    manager = get_playback_manager()
    executing_robots = []

    for robot_id in get_all_active_robots():
        if manager.get_execution_state(robot_id) == PlaybackState.EXECUTING:
            executing_robots.append(robot_id)

    return executing_robots


def get_robot_count() -> int:
    """Get total number of active robots

    Useful for VS Code extension UI to show robot count or decide
    whether to show single-robot vs multi-robot interface.

    Returns:
        Number of active robots
    """
    return len(get_all_active_robots())


def has_any_active_robots() -> bool:
    """Check if there are any active robots

    Quick check for VS Code extensions to enable/disable robot controls.

    Returns:
        True if any robots are active, False otherwise
    """
    return get_robot_count() > 0


def get_robot_status_summary() -> dict:
    """Get a summary of all robot statuses

    Provides a complete overview for VS Code extension dashboards.

    Returns:
        Dictionary with robot count, execution summary, and robot details
    """
    active_robots = get_all_active_robots()
    executing_robots = get_currently_executing_robots()

    return {
        "total_robots": len(active_robots),
        "executing_robots": len(executing_robots),
        "idle_robots": len(active_robots) - len(executing_robots),
        "robot_ids": active_robots,
        "executing_robot_ids": executing_robots,
        "has_robots": len(active_robots) > 0,
        "primary_robot_id": get_primary_robot_id(),
    }
