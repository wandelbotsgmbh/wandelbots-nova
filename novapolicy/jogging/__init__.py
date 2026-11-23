"""Jogging: waypoint motion through NOVA action chunk streaming (docs/jogging.md).

``jog_joints`` / ``jog_tcp`` are the high-level context managers. The other names are
re-exported from their homes (``novapolicy.pipeline``, ``novapolicy.transport``) so imports
from here keep working.
"""

from novapolicy.jogging.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
from novapolicy.pipeline.base import (
    ChunkModule,
    ModuleContext,
    Timestamps,
    WaypointFilter,
    WaypointModule,
)
from novapolicy.pipeline.defaults import default_pipeline
from novapolicy.pipeline.modules.bridge import Bridge
from novapolicy.pipeline.modules.drop_late import DropLate
from novapolicy.pipeline.modules.ease_in import EaseIn
from novapolicy.pipeline.modules.endpoint_ramp import EndpointRamp
from novapolicy.pipeline.modules.hold_padding import HoldPadding
from novapolicy.pipeline.modules.joint_clamp import JointClamp
from novapolicy.pipeline.modules.live_buffer import LiveBuffer
from novapolicy.pipeline.modules.smoothing import Smoothing
from novapolicy.transport.session import WaypointJoggingSession
from novapolicy.transport.state import JoggingStateTracker

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
