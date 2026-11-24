"""The send pipeline: the module kinds between the policy's chunk and NOVA (contracts T13).

``base`` defines the kinds and their order, ``modules`` the built-in modules, ``defaults``
the list that runs when none is given, and ``placement`` where a chunk sits on the clock.
"""

from novapolicy.pipeline.base import (
    ChunkModule,
    ModuleContext,
    Timestamps,
    WaypointFilter,
    WaypointModule,
)
from novapolicy.pipeline.defaults import default_pipeline
from novapolicy.pipeline.modules import (
    Bridge,
    DropLate,
    EaseIn,
    EndpointRamp,
    HoldPadding,
    JointClamp,
    LiveBuffer,
    ServoHorizon,
    Smoothing,
    TimestepTimeline,
)

__all__ = [
    "Bridge",
    "ChunkModule",
    "DropLate",
    "EaseIn",
    "EndpointRamp",
    "HoldPadding",
    "JointClamp",
    "LiveBuffer",
    "ModuleContext",
    "ServoHorizon",
    "Smoothing",
    "Timestamps",
    "TimestepTimeline",
    "WaypointFilter",
    "WaypointModule",
    "default_pipeline",
]
