"""Opt-in diagnostics for policy execution."""

from novapolicy.debug.trajectory_trace import (
    ExecutionTrajectoryTrace,
    RawActionChunkTrace,
    WaypointTrajectoryTrace,
)

__all__ = [
    "ExecutionTrajectoryTrace",
    "RawActionChunkTrace",
    "WaypointTrajectoryTrace",
]
