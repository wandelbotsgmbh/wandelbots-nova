"""Jogging session — velocity-streamed motion control via NOVA Jogging API.

This subpackage manages the WebSocket connection to the NOVA Jogging API and
streams velocity commands computed from a trapezoidal velocity profile.
"""

from policy.jogging.session import JoggingSession, JoggingStateTracker

__all__ = [
    "JoggingSession",
    "JoggingStateTracker",
]
