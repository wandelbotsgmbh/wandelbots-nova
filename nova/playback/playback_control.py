"""Nova Playback Control System

This module provides centralized playback speed control and pause/resume functionality
for Nova robot executions. It implements a clear precedence hierarchy:

1. External override (e.g., from external integrations)
2. Method parameter (playback_speed=0.5)
3. Decorator default (@nova.program(playback_speed_percent=30))
4. System default (100% speed)

This is the main entry point that re-exports all necessary components.
For implementation details, see the individual modules:
- playback_events.py: Event system
- playback_state.py: State management
- playback_control_interface.py: Manager interface
- playback_control_manager.py: Manager implementation
"""

# Re-export all public API from the individual modules
from nova.playback.playback_control_interface import (
    clear_active_program_playback_speed_percent,
    get_active_program_playback_speed_percent,
    get_all_active_robots,
    get_playback_manager,
    get_robot_status_summary,
    set_active_program_playback_speed_percent,
    set_playback_manager,
)
from nova.playback.playback_control_manager import (
    InvalidSpeedError,
    PlaybackControlError,
    PlaybackControlManager,
)
from nova.playback.playback_events import (
    ExecutionStartedEvent,
    ExecutionStoppedEvent,
    PlaybackDirection,
    PlaybackEvent,
    PlaybackSpeedPercent,
    PlaybackState,
    ProgramStartedEvent,
    ProgramStoppedEvent,
    RobotRegisteredEvent,
    RobotUnregisteredEvent,
    SpeedChangeEvent,
    StateChangeEvent,
    motion_group_id,
)

__all__ = [
    # Core types
    "PlaybackSpeedPercent",
    "PlaybackState",
    "PlaybackDirection",
    "motion_group_id",
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
    # Manager interface
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
