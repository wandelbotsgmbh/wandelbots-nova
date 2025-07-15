"""Nova Playback Event System

Event definitions and types for the playback control system.
This module contains all event-related classes and is designed to be imported
without circular dependencies.
"""

from datetime import datetime, timezone
from enum import Enum
from typing import Callable, Literal, Optional

from pydantic import BaseModel, Field


# Type-safe identifiers (re-exported from types module to avoid circulars)
class PlaybackSpeedPercent(BaseModel):
    """Playback speed percentage with validation (1-100)"""

    value: int = Field(..., ge=1, le=100, description="Speed percentage between 1 and 100")


# Control source types with strict validation
PlaybackSourceType = Literal["external", "method", "decorator", "default"]


class PlaybackState(Enum):
    """Current state of robot execution"""

    IDLE = "idle"
    EXECUTING = "executing"
    PAUSED = "paused"
    PLAYING = "playing"  # Resumed from pause


class PlaybackDirection(Enum):
    """Direction of trajectory execution"""

    FORWARD = "forward"
    BACKWARD = "backward"


# Enhanced event system for comprehensive robot control
class PlaybackEvent(BaseModel):
    """Base class for all playback events"""

    event_type: str
    motion_group_id: str
    timestamp: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class RobotRegisteredEvent(PlaybackEvent):
    """Robot registration event"""

    event_type: str = "robot_registered"
    robot_name: str
    initial_speed: PlaybackSpeedPercent


class RobotUnregisteredEvent(PlaybackEvent):
    """Robot unregistration event"""

    event_type: str = "robot_unregistered"


class StateChangeEvent(PlaybackEvent):
    """Robot state change event"""

    event_type: str = "state_change"
    old_state: PlaybackState
    new_state: PlaybackState
    speed: PlaybackSpeedPercent
    direction: PlaybackDirection = PlaybackDirection.FORWARD


class SpeedChangeEvent(PlaybackEvent):
    """Speed change event"""

    event_type: str = "speed_change"
    old_speed: PlaybackSpeedPercent
    new_speed: PlaybackSpeedPercent
    source: PlaybackSourceType


class ExecutionStartedEvent(PlaybackEvent):
    """Execution started event"""

    event_type: str = "execution_started"
    speed: PlaybackSpeedPercent


class ExecutionStoppedEvent(PlaybackEvent):
    """Execution stopped event"""

    event_type: str = "execution_stopped"


class ProgramStartedEvent(PlaybackEvent):
    """Program started event"""

    event_type: str = "program_started"
    program_name: Optional[str] = None
    total_robots: int = 0


class ProgramStoppedEvent(PlaybackEvent):
    """Program stopped event"""

    event_type: str = "program_stopped"
    program_name: Optional[str] = None


# Event callback types
EventCallback = Callable[[PlaybackEvent], None]


def motion_group_id(id_str: str) -> str:
    """Convert string to motion group id (passthrough for consistency)

    Args:
        id_str: String ID from motion_group.motion_group_id

    Returns:
        str for use with playback control functions

    Example:
        # Both of these work:
        mgid = motion_group_id(motion_group.motion_group_id)  # str
        mgid = motion_group_id("some_robot_id")               # str
    """
    return id_str
