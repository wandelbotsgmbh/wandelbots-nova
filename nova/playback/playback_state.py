"""Nova Playback Control State Management

This module handles the core state management for playback control,
separated from the main manager to reduce file size and improve maintainability.
"""

import threading
from datetime import datetime, timezone
from typing import Optional

from pydantic import BaseModel

from nova.playback.playback_events import (
    PlaybackDirection,
    PlaybackSourceType,
    PlaybackSpeedPercent,
    PlaybackState,
)


class PlaybackControl(BaseModel):
    """Represents a playback control setting with its source and priority"""

    model_config = {"frozen": True}

    speed: PlaybackSpeedPercent
    source: PlaybackSourceType
    state: Optional[PlaybackState] = None
    direction: Optional[PlaybackDirection] = None
    set_at: datetime = datetime.now(timezone.utc)


class RobotMetadata(BaseModel):
    """Metadata for a registered robot"""

    name: str
    registered_at: datetime
    initial_speed: PlaybackSpeedPercent


class PlaybackControlState:
    """Manages the internal state of playback control system"""

    def __init__(self):
        self._lock = threading.Lock()

        # Control state storage
        self._external_overrides: dict[str, PlaybackControl] = {}
        self._decorator_defaults: dict[str, PlaybackControl] = {}
        self._execution_states: dict[str, PlaybackState] = {}
        self._robot_metadata: dict[str, RobotMetadata] = {}

        # Program-level state
        self._active_program_playback_speed: Optional[int] = None
        self._active_program_name: Optional[str] = None

        # Callback storage
        self._event_callbacks: list = []

    def get_external_override(self, motion_group_id: str) -> Optional[PlaybackControl]:
        """Get external override for a robot"""
        with self._lock:
            return self._external_overrides.get(motion_group_id)

    def set_external_override(self, motion_group_id: str, control: PlaybackControl) -> None:
        """Set external override for a robot"""
        with self._lock:
            self._external_overrides[motion_group_id] = control

    def clear_external_override(self, motion_group_id: str) -> None:
        """Clear external override for a robot"""
        with self._lock:
            self._external_overrides.pop(motion_group_id, None)

    def get_decorator_default(self, motion_group_id: str) -> Optional[PlaybackControl]:
        """Get decorator default for a robot"""
        with self._lock:
            return self._decorator_defaults.get(motion_group_id)

    def set_decorator_default(self, motion_group_id: str, control: PlaybackControl) -> None:
        """Set decorator default for a robot"""
        with self._lock:
            self._decorator_defaults[motion_group_id] = control

    def get_execution_state(self, motion_group_id: str) -> Optional[PlaybackState]:
        """Get execution state for a robot"""
        with self._lock:
            return self._execution_states.get(motion_group_id)

    def set_execution_state(self, motion_group_id: str, state: PlaybackState) -> None:
        """Set execution state for a robot"""
        with self._lock:
            self._execution_states[motion_group_id] = state

    def get_robot_metadata(self, motion_group_id: str) -> Optional[RobotMetadata]:
        """Get robot metadata"""
        with self._lock:
            return self._robot_metadata.get(motion_group_id)

    def set_robot_metadata(self, motion_group_id: str, metadata: RobotMetadata) -> None:
        """Set robot metadata"""
        with self._lock:
            self._robot_metadata[motion_group_id] = metadata

    def remove_robot(self, motion_group_id: str) -> None:
        """Remove all state for a robot"""
        with self._lock:
            self._external_overrides.pop(motion_group_id, None)
            self._decorator_defaults.pop(motion_group_id, None)
            self._execution_states.pop(motion_group_id, None)
            self._robot_metadata.pop(motion_group_id, None)

    def get_all_robots(self) -> list[str]:
        """Get all registered robots"""
        with self._lock:
            return list(self._robot_metadata.keys())

    def get_active_program_speed(self) -> Optional[int]:
        """Get active program playback speed"""
        return self._active_program_playback_speed

    def set_active_program_speed(self, speed: Optional[int]) -> None:
        """Set active program playback speed"""
        self._active_program_playback_speed = speed

    def get_active_program_name(self) -> Optional[str]:
        """Get active program name"""
        return self._active_program_name

    def set_active_program_name(self, name: Optional[str]) -> None:
        """Set active program name"""
        self._active_program_name = name

    def add_event_callback(self, callback) -> None:
        """Add event callback"""
        with self._lock:
            self._event_callbacks.append(callback)

    def get_event_callbacks(self) -> list:
        """Get all event callbacks"""
        with self._lock:
            return self._event_callbacks.copy()
