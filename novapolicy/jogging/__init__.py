"""Jogging — waypoint-based motion control via NOVA action chunk streaming.

This subpackage owns all jogging functionality: the low-level
``WaypointJoggingSession`` that streams timestamped waypoints, the
``JoggingStateTracker`` that detects blocking pauses, and the high-level
``jog_joints`` / ``jog_tcp`` context managers (plus their ``JointJogger`` /
``TcpJogger`` implementations) for manual jogging.
"""

from novapolicy.jogging.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
from novapolicy.jogging.live_buffer import LiveBuffer
from novapolicy.jogging.modules import (
    EndpointRamp,
    HoldPadding,
    JointClamp,
    default_pipeline,
)
from novapolicy.jogging.pipeline import (
    ChunkModule,
    EaseIn,
    ModuleContext,
    Timestamps,
    WaypointModule,
)
from novapolicy.jogging.session import JoggingStateTracker
from novapolicy.jogging.waypoint_session import WaypointJoggingSession

__all__ = [
    "ChunkModule",
    "EaseIn",
    "EndpointRamp",
    "HoldPadding",
    "JoggingStateTracker",
    "JointClamp",
    "JointJogger",
    "LiveBuffer",
    "ModuleContext",
    "TcpJogger",
    "Timestamps",
    "WaypointJoggingSession",
    "WaypointModule",
    "default_pipeline",
    "jog_joints",
    "jog_tcp",
]
