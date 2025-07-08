"""Nova External Control Module

This module provides external interfaces for controlling Nova robot playback
from external tools like VS Code extensions, future WebSocket/HTTP servers, etc.
"""

from nova.external_control.external_integration import (
    nova_get_available_robots,
    nova_pause_robot,
    nova_resume_robot,
    nova_set_playback_speed,
    register_external_control_functions,
)

__all__ = [
    "nova_set_playback_speed",
    "nova_pause_robot",
    "nova_resume_robot",
    "nova_get_available_robots",
    "register_external_control_functions",
]

# Auto-register functions when module is imported
register_external_control_functions()
