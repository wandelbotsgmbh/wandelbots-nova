"""Jogging — waypoint-based motion control via NOVA action chunk streaming.

This subpackage owns all jogging functionality: the low-level
``WaypointJoggingSession`` that streams timestamped waypoints, the
``JoggingStateTracker`` that detects blocking pauses, and the high-level
``jog_joints`` / ``jog_tcp`` context managers (plus their ``JointJogger`` /
``TcpJogger`` implementations) for manual jogging.
"""

from novapolicy.jogging.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
from novapolicy.jogging.session import JoggingStateTracker
from novapolicy.jogging.stages import EaseIn, StageContext, WaypointStage
from novapolicy.jogging.waypoint_session import WaypointJoggingSession

__all__ = [
    "EaseIn",
    "JoggingStateTracker",
    "JointJogger",
    "StageContext",
    "TcpJogger",
    "WaypointJoggingSession",
    "WaypointStage",
    "jog_joints",
    "jog_tcp",
]
