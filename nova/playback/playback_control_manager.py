"""Nova Playback Control Manager Implementation

The actual implementation of the playback control manager, separated to avoid
circular imports and reduce file size.
"""

from datetime import datetime, timezone
from typing import Optional

from nova.core import logger
from nova.playback.playback_control_interface import PlaybackControlManagerInterface
from nova.playback.playback_events import (
    EventCallback,
    ExecutionStartedEvent,
    ExecutionStoppedEvent,
    PlaybackDirection,
    PlaybackSpeedPercent,
    PlaybackState,
    ProgramStartedEvent,
    ProgramStoppedEvent,
    RobotRegisteredEvent,
    RobotUnregisteredEvent,
    SpeedChangeEvent,
    StateChangeEvent,
)
from nova.playback.playback_state import PlaybackControl, PlaybackControlState, RobotMetadata


class PlaybackControlError(Exception):
    """Base exception for playback control errors"""

    pass


class InvalidSpeedError(PlaybackControlError):
    """Raised when an invalid speed value is provided"""

    pass


class PlaybackControlManager(PlaybackControlManagerInterface):
    """Centralized playback control with precedence resolution and state notifications

    This manager maintains state for all robots and resolves playback speed
    and state based on a clear precedence hierarchy. Thread-safe for
    concurrent access from multiple sources.
    """

    def __init__(self):
        """Initialize with thread-safe data structures"""
        self._state = PlaybackControlState()

    def register_robot(
        self,
        motion_group_id: str,
        robot_name: Optional[str] = None,
        initial_speed: PlaybackSpeedPercent = PlaybackSpeedPercent(value=100),
    ) -> None:
        """Register a robot as available for control

        Args:
            motion_group_id: Motion group identifier
            robot_name: Human-readable robot name
            initial_speed: Initial speed setting
        """
        mgid = motion_group_id

        metadata = RobotMetadata(
            name=robot_name or str(mgid),
            registered_at=datetime.now(timezone.utc),
            initial_speed=initial_speed,
        )
        self._state.set_robot_metadata(mgid, metadata)

        # Set initial decorator default if not already set
        if self._state.get_decorator_default(mgid) is None:
            control = PlaybackControl(speed=initial_speed, source="default")
            self._state.set_decorator_default(mgid, control)

        # Emit registration event
        event = RobotRegisteredEvent(
            motion_group_id=mgid, robot_name=metadata.name, initial_speed=initial_speed
        )
        self._emit_event(event)
        logger.debug(f"Robot registered: {mgid} ({metadata.name}) at {initial_speed}%")

    def unregister_robot(self, motion_group_id: str) -> None:
        """Unregister a robot from control

        Args:
            motion_group_id: Motion group identifier
        """
        mgid = motion_group_id

        # Get metadata before removal for event
        metadata = self._state.get_robot_metadata(mgid)
        if metadata is None:
            return

        # Remove all state for this robot
        self._state.remove_robot(mgid)

        # Emit unregistration event
        event = RobotUnregisteredEvent(motion_group_id=mgid)
        self._emit_event(event)
        logger.debug(f"Robot unregistered: {mgid} ({metadata.name})")

    def get_all_robots(self) -> list[str]:
        """Get all robots with any settings (registered, overrides, or defaults)"""
        all_robots = set()

        # Add registered robots
        all_robots.update(self._state.get_all_robots())

        # Add robots with external overrides
        with self._state._lock:
            all_robots.update(self._state._external_overrides.keys())

        # Add robots with decorator defaults
        with self._state._lock:
            all_robots.update(self._state._decorator_defaults.keys())

        return list(all_robots)

    def set_external_override(
        self,
        motion_group_id: str,
        speed: PlaybackSpeedPercent,
        state: Optional[PlaybackState] = None,
        direction: Optional[PlaybackDirection] = None,
    ) -> None:
        """Set external override for a robot

        Args:
            motion_group_id: Motion group identifier
            speed: New speed (0-100%)
            state: Optional state change
            direction: Optional direction change
        """
        mgid = motion_group_id
        old_override = self._state.get_external_override(mgid)
        old_speed = old_override.speed if old_override else PlaybackSpeedPercent(value=100)

        # Create new control setting
        control = PlaybackControl(speed=speed, source="external", state=state, direction=direction)
        self._state.set_external_override(mgid, control)

        # Emit speed change event if speed actually changed
        if old_speed != speed:
            event = SpeedChangeEvent(
                motion_group_id=mgid, old_speed=old_speed, new_speed=speed, source="external"
            )
            self._emit_event(event)

        # Emit state change event if state was provided (but don't update execution state)
        # The execution state should only be changed by explicit pause/resume calls
        # Exception: if setting PlaybackState.PLAYING while paused, clear the pause
        if state is not None:
            current_execution_state = self._state.get_execution_state(mgid)
            if state == PlaybackState.PLAYING and current_execution_state == PlaybackState.PAUSED:
                # Clear the pause when external override sets to PLAYING
                with self._state._lock:
                    self._state._execution_states.pop(mgid, None)

            old_effective_state = self.get_effective_state(mgid)
            if old_effective_state != state:
                state_event = StateChangeEvent(
                    motion_group_id=mgid,
                    old_state=old_effective_state,
                    new_state=state,
                    speed=speed,
                    direction=direction or PlaybackDirection.FORWARD,
                )
                self._emit_event(state_event)

        logger.debug(
            f"External override set: {mgid} -> {speed}% (state: {state}, direction: {direction})"
        )

    def clear_external_override(self, motion_group_id: str) -> None:
        """Clear external override for a robot

        Args:
            motion_group_id: Motion group identifier
        """
        mgid = motion_group_id
        old_override = self._state.get_external_override(mgid)

        if old_override:
            # Remove the external override
            self._state.clear_external_override(mgid)

            # Calculate new effective speed after clearing override
            new_speed = self.get_effective_speed(mgid)

            # Emit speed change event
            event = SpeedChangeEvent(
                motion_group_id=mgid,
                old_speed=old_override.speed,
                new_speed=new_speed,
                source="external",
            )
            self._emit_event(event)

            logger.debug(f"External override cleared: {mgid} -> fallback to {new_speed}%")

    def set_decorator_default(self, motion_group_id: str, speed: PlaybackSpeedPercent) -> None:
        """Set decorator default speed for a robot

        Args:
            motion_group_id: Motion group identifier
            speed: Default speed from @nova.program decorator
        """
        if not (0 <= speed.value <= 100):
            raise ValueError(f"Speed percent must be between 0 and 100, got {speed.value}")

        mgid = motion_group_id
        control = PlaybackControl(speed=speed, source="decorator")
        self._state.set_decorator_default(mgid, control)
        logger.debug(f"Decorator default set: {mgid} -> {speed}%")

    def get_effective_speed(
        self, motion_group_id: str, method_speed: Optional[PlaybackSpeedPercent] = None
    ) -> PlaybackSpeedPercent:
        """Get the effective speed based on precedence hierarchy

        Precedence: external > method > decorator > default (100%)

        Args:
            motion_group_id: Motion group identifier
            method_speed: Optional method-level speed

        Returns:
            Effective speed percentage
        """
        mgid = motion_group_id

        # 1. External override (highest precedence)
        external = self._state.get_external_override(mgid)
        if external is not None:
            return external.speed

        # 2. Method parameter
        if method_speed is not None:
            return method_speed

        # 3. Decorator default
        decorator = self._state.get_decorator_default(mgid)
        if decorator is not None:
            return decorator.speed

        # 4. System default
        return PlaybackSpeedPercent(value=100)

    def pause(self, motion_group_id: str) -> None:
        """Pause robot execution"""
        mgid = motion_group_id
        old_state = self.get_effective_state(mgid)  # Always returns a state now

        # Only pause if not already paused
        if old_state != PlaybackState.PAUSED:
            self._state.set_execution_state(mgid, PlaybackState.PAUSED)

            # Emit state change event
            event = StateChangeEvent(
                motion_group_id=mgid,
                old_state=old_state,
                new_state=PlaybackState.PAUSED,
                speed=self.get_effective_speed(mgid),
                direction=PlaybackDirection.FORWARD,
            )
            self._emit_event(event)
            logger.debug(f"Robot paused: {mgid} (was {old_state})")

    def resume(self, motion_group_id: str) -> None:
        """Resume robot execution"""
        mgid = motion_group_id
        old_state = self.get_effective_state(mgid)  # Use effective state

        if old_state == PlaybackState.PAUSED:
            self._state.set_execution_state(mgid, PlaybackState.PLAYING)

            # Emit state change event
            event = StateChangeEvent(
                motion_group_id=mgid,
                old_state=old_state,
                new_state=PlaybackState.PLAYING,
                speed=self.get_effective_speed(mgid),
                direction=PlaybackDirection.FORWARD,
            )
            self._emit_event(event)
            logger.debug(f"Robot resumed: {mgid}")

    def can_pause(self, motion_group_id: str) -> bool:
        """Check if robot can be paused"""
        mgid = motion_group_id
        state = self._state.get_execution_state(mgid)
        return state in (PlaybackState.EXECUTING, PlaybackState.PLAYING)

    def can_resume(self, motion_group_id: str) -> bool:
        """Check if robot can be resumed"""
        mgid = motion_group_id
        state = self._state.get_execution_state(mgid)
        return state == PlaybackState.PAUSED

    def get_execution_state(self, motion_group_id: str) -> Optional[PlaybackState]:
        """Get current execution state"""
        mgid = motion_group_id
        return self._state.get_execution_state(mgid)

    def set_execution_state(self, motion_group_id: str, state: PlaybackState) -> None:
        """Set execution state"""
        mgid = motion_group_id
        old_state = self._state.get_execution_state(mgid) or PlaybackState.IDLE
        self._state.set_execution_state(mgid, state)

        if old_state != state:
            # Emit appropriate events
            if state == PlaybackState.EXECUTING:
                start_event = ExecutionStartedEvent(
                    motion_group_id=mgid, speed=self.get_effective_speed(mgid)
                )
                self._emit_event(start_event)
            elif old_state == PlaybackState.EXECUTING:
                stop_event = ExecutionStoppedEvent(motion_group_id=mgid)
                self._emit_event(stop_event)

    def get_effective_state(self, motion_group_id: str) -> PlaybackState:
        """Get effective state considering overrides"""
        mgid = motion_group_id

        # Check execution state first - pause/resume takes highest precedence
        execution_state = self._state.get_execution_state(mgid)
        if execution_state is not None:
            return execution_state

        # Check external override next
        external = self._state.get_external_override(mgid)
        if external and external.state:
            return external.state

        # Default to PLAYING
        return PlaybackState.PLAYING

    def get_effective_direction(self, motion_group_id: str) -> PlaybackDirection:
        """Get effective direction considering overrides"""
        mgid = motion_group_id

        # Check external override first
        external = self._state.get_external_override(mgid)
        if external and external.direction:
            return external.direction

        # Default to FORWARD direction
        return PlaybackDirection.FORWARD

    def get_robot_metadata(self, motion_group_id: str) -> Optional[dict]:
        """Get robot metadata"""
        mgid = motion_group_id
        metadata = self._state.get_robot_metadata(mgid)
        if metadata:
            return {
                "name": metadata.name,
                "registered_at": metadata.registered_at,
                "initial_speed": metadata.initial_speed,
            }
        return None

    def register_event_callback(self, callback: EventCallback) -> None:
        """Register callback for playback events"""
        self._state.add_event_callback(callback)

    def start_program(self, program_name: Optional[str] = None) -> None:
        """Notify that a program has started"""
        self._state.set_active_program_name(program_name)

        event = ProgramStartedEvent(
            motion_group_id="global",  # Global event
            program_name=program_name,
            total_robots=len(self._state.get_all_robots()),
        )
        self._emit_event(event)
        logger.info(f"Program started: {program_name}")

    def stop_program(self, program_name: Optional[str] = None) -> None:
        """Notify that a program has stopped"""
        self._state.set_active_program_name(None)

        event = ProgramStoppedEvent(
            motion_group_id="global",  # Global event
            program_name=program_name,
        )
        self._emit_event(event)
        logger.info(f"Program stopped: {program_name}")

    def get_active_program(self) -> Optional[str]:
        """Get active program name"""
        return self._state.get_active_program_name()

    def _emit_event(self, event) -> None:
        """Emit event to all registered callbacks"""
        # Event system
        for callback in self._state.get_event_callbacks():
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Error in event callback: {e}")
