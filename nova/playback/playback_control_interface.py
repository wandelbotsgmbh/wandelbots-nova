"""Nova Playback Control Manager Interface

Simplified interface for the playback control manager that other modules can import
without circular dependency risks. The actual implementation is injected at runtime.
"""

from typing import Optional, Protocol

from nova.playback.playback_events import (
    EventCallback,
    PlaybackDirection,
    PlaybackSpeedPercent,
    PlaybackState,
)


class PlaybackControlManagerInterface(Protocol):
    """Interface for the playback control manager"""

    def get_effective_speed(
        self, motion_group_id: str, method_speed: Optional[PlaybackSpeedPercent] = None
    ) -> PlaybackSpeedPercent:
        """Get the effective speed for a robot"""
        ...

    def set_external_override(
        self,
        motion_group_id: str,
        speed: PlaybackSpeedPercent,
        state: Optional[PlaybackState] = None,
        direction: Optional[PlaybackDirection] = None,
    ) -> None:
        """Set external override for a robot"""
        ...

    def clear_external_override(self, motion_group_id: str) -> None:
        """Clear external override for a robot"""
        ...

    def set_decorator_default(self, motion_group_id: str, speed: PlaybackSpeedPercent) -> None:
        """Set decorator default speed"""
        ...

    def pause(self, motion_group_id: str) -> None:
        """Pause robot execution"""
        ...

    def resume(self, motion_group_id: str) -> None:
        """Resume robot execution"""
        ...

    def can_pause(self, motion_group_id: str) -> bool:
        """Check if robot can be paused"""
        ...

    def can_resume(self, motion_group_id: str) -> bool:
        """Check if robot can be resumed"""
        ...

    def get_execution_state(self, motion_group_id: str) -> Optional[PlaybackState]:
        """Get execution state"""
        ...

    def set_execution_state(self, motion_group_id: str, state: PlaybackState) -> None:
        """Set execution state"""
        ...

    def get_effective_state(self, motion_group_id: str) -> PlaybackState:
        """Get effective state considering overrides"""
        ...

    def get_effective_direction(self, motion_group_id: str) -> PlaybackDirection:
        """Get effective direction considering overrides"""
        ...

    def register_robot(
        self, motion_group_id: str, robot_name: str, initial_speed: PlaybackSpeedPercent
    ) -> None:
        """Register a robot"""
        ...

    def unregister_robot(self, motion_group_id: str) -> None:
        """Unregister a robot"""
        ...

    def get_robot_metadata(self, motion_group_id: str) -> Optional[dict]:
        """Get robot metadata"""
        ...

    def register_event_callback(self, callback: EventCallback) -> None:
        """Register event callback"""
        ...

    def start_program(self, program_name: Optional[str] = None) -> None:
        """Start a program"""
        ...

    def stop_program(self, program_name: Optional[str] = None) -> None:
        """Stop a program"""
        ...

    def get_active_program(self) -> Optional[str]:
        """Get active program name"""
        ...

    def get_all_robots(self) -> list[str]:
        """Get all registered robots"""
        ...


# Global manager instance - injected at runtime to avoid circulars
_manager: Optional[PlaybackControlManagerInterface] = None


def set_playback_manager(manager: PlaybackControlManagerInterface) -> None:
    """Set the global playback manager instance"""
    global _manager
    _manager = manager


def get_playback_manager() -> PlaybackControlManagerInterface:
    """Get the global playback manager instance"""
    if _manager is None:
        # Lazy import to avoid circular dependencies
        from nova.playback.playback_control_manager import PlaybackControlManager

        manager = PlaybackControlManager()
        set_playback_manager(manager)
    assert _manager is not None, "Failed to initialize playback manager"
    return _manager


# Program-level speed management (thread-safe)
_active_program_playback_speed: Optional[int] = None


def get_active_program_playback_speed_percent() -> Optional[int]:
    """Get the active program playback speed percentage"""
    return _active_program_playback_speed


def set_active_program_playback_speed_percent(speed: int) -> None:
    """Set the active program playback speed percentage"""
    global _active_program_playback_speed
    _active_program_playback_speed = speed


def clear_active_program_playback_speed_percent() -> None:
    """Clear the active program playback speed percentage"""
    global _active_program_playback_speed
    _active_program_playback_speed = None


def get_all_active_robots() -> list[str]:
    """Get all active robots"""
    return get_playback_manager().get_all_robots() if _manager else []


def get_robot_status_summary() -> dict:
    """Get a summary of all robot statuses for external UIs"""
    if not _manager:
        return {
            "total_robots": 0,
            "executing_robots": 0,
            "motion_group_ids": [],
            "executing_motion_group_ids": [],
            "has_robots": False,
            "active_program": None,
            "external_control_active": False,
        }

    active_robots = get_all_active_robots()
    manager = get_playback_manager()
    executing_robots = [
        robot
        for robot in active_robots
        if manager.get_execution_state(robot) == PlaybackState.EXECUTING
    ]

    # Get program information
    active_program = manager.get_active_program()

    return {
        "total_robots": len(active_robots),
        "executing_robots": len(executing_robots),
        "motion_group_ids": active_robots,
        "executing_motion_group_ids": executing_robots,
        "has_robots": len(active_robots) > 0,
        "active_program": active_program,
        "external_control_active": active_program is not None,
    }
