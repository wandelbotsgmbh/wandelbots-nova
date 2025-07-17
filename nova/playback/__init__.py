"""Nova Playback Control System

This module provides centralized playback speed control and pause/resume functionality
for Nova robot executions. It implements a clear precedence hierarchy:

1. External override (e.g., from external integrations)
2. Method parameter (playback_speed=0.5)
3. Decorator default (@nova.program(playback_speed_percent=30))
4. System default (100% speed)

Key Components:
- events.py: Event system for playback state changes
- state.py: Internal state management
- manager.py: Main playback control manager
- types.py: Type definitions and utilities
"""

# Export public API
from nova.playback.playback_control import (
    ExecutionStartedEvent,
    ExecutionStoppedEvent,
    InvalidSpeedError,
    # Exceptions
    PlaybackControlError,
    PlaybackControlManager,
    PlaybackDirection,
    # Events
    PlaybackEvent,
    PlaybackSpeedPercent,
    PlaybackState,
    ProgramStartedEvent,
    ProgramStoppedEvent,
    RobotRegisteredEvent,
    RobotUnregisteredEvent,
    SpeedChangeEvent,
    StateChangeEvent,
    clear_active_program_playback_speed_percent,
    # Program-level functions
    get_active_program_playback_speed_percent,
    # Utility functions
    get_all_active_robots,
    # Manager interface
    get_playback_manager,
    get_robot_status_summary,
    set_active_program_playback_speed_percent,
    set_playback_manager,
)

__all__ = [
    # Core types
    "PlaybackSpeedPercent",
    "PlaybackState",
    "PlaybackDirection",
    # Events
    "PlaybackEvent",
    "RobotRegisteredEvent",
    "RobotUnregisteredEvent",
    "StateChangeEvent",
    "SpeedChangeEvent",
    "ExecutionStartedEvent",
    "ExecutionStoppedEvent",
    "ProgramStartedEvent",
    "ProgramStoppedEvent",
    # Manager
    "get_playback_manager",
    "set_playback_manager",
    "PlaybackControlManager",
    # Program-level functions
    "get_active_program_playback_speed_percent",
    "set_active_program_playback_speed_percent",
    "clear_active_program_playback_speed_percent",
    # Utility functions
    "get_all_active_robots",
    "get_robot_status_summary",
    # Exceptions
    "PlaybackControlError",
    "InvalidSpeedError",
]
