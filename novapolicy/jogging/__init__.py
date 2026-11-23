"""Jogging: waypoint motion through NOVA action chunk streaming (docs/jogging.md).

``WaypointJoggingSession`` streams timestamped waypoints; ``jog_joints`` / ``jog_tcp`` are the
high-level context managers.
"""

from novapolicy.jogging.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
from novapolicy.jogging.live_buffer import LiveBuffer
from novapolicy.jogging.modules import (
    Bridge,
    DropLate,
    EndpointRamp,
    HoldPadding,
    JointClamp,
    Smoothing,
    default_pipeline,
)
from novapolicy.jogging.pipeline import (
    ChunkModule,
    EaseIn,
    ModuleContext,
    Timestamps,
    WaypointFilter,
    WaypointModule,
)
from novapolicy.jogging.session import JoggingStateTracker
from novapolicy.jogging.waypoint_session import WaypointJoggingSession

__all__ = [
    "Bridge",
    "ChunkModule",
    "DropLate",
    "EaseIn",
    "EndpointRamp",
    "HoldPadding",
    "JoggingStateTracker",
    "JointClamp",
    "JointJogger",
    "LiveBuffer",
    "ModuleContext",
    "Smoothing",
    "TcpJogger",
    "Timestamps",
    "WaypointFilter",
    "WaypointJoggingSession",
    "WaypointModule",
    "default_pipeline",
    "jog_joints",
    "jog_tcp",
]
