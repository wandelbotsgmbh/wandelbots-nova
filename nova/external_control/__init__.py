"""Nova External Control Module

This module provides external interfaces for controlling Nova robot playback.
External tools and applications can use WebSocket or direct API access.

WebSocket Control (Recommended):
    Use the @nova.program decorator with external_control=WebSocketControl()
    External clients connect to ws://localhost:8765
    Real-time bidirectional communication with live updates.

Direct API Control (In-Process):
    from nova.playback import get_playback_manager, MotionGroupId, PlaybackSpeedPercent

    manager = get_playback_manager()
    manager.set_external_override(MotionGroupId("robot1"), PlaybackSpeedPercent(value=50))
    manager.pause(MotionGroupId("robot1"))
    manager.resume(MotionGroupId("robot1"))
"""

# Re-export playback control for convenience
from nova.external_control.websocket_control import (
    get_websocket_server,
    start_websocket_server,
    stop_websocket_server,
)
from nova.external_control.websocket_program_control import WebSocketControl
from nova.playback import PlaybackDirection, PlaybackSpeedPercent, get_playback_manager

__all__ = [
    "PlaybackSpeedPercent",
    "PlaybackDirection",
    "get_playback_manager",
    "start_websocket_server",
    "stop_websocket_server",
    "get_websocket_server",
    "WebSocketControl",
]
