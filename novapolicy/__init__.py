"""policy — real-time robot control from action chunks via waypoint jogging.

.. warning::
    **EXPERIMENTAL** — This package is under active development and not ready
    for production use. The API will have breaking changes between releases.
    Do not depend on it for stable deployments.

This package enables AI policies to stream joint position targets (+ optional IO commands)
to one or more motion groups. Timestamped waypoints are sent via the NOVA Jogging API;
the server handles velocity profiling, interpolation, and IK internally.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

from novapolicy.cameras import CameraSource, WebRTCCameras
from novapolicy.chunking import (
    ConnectedActionChunk,
    InterpolatedActionChunk,
    connect_action_chunk,
    create_bridge_chunk,
    interpolate_action_chunk_ramps,
    smooth_action_chunk,
)
from novapolicy.executor import ExecutionResult, ExecutorStatus, Phase, PolicyExecutor
from novapolicy.gr00t import Gr00tPolicyClient, RTCConfig
from novapolicy.jogging import JointJogger, TcpJogger, jog_joints, jog_tcp
from novapolicy.policy_client import CallbackPolicyClient, PolicyClient
from novapolicy.schema import (
    Action,
    BoolMapping,
    Mapping,
    Observation,
    PolicySchema,
)
from novapolicy.types import (
    ActionChunk,
    ActionMode,
    EmergencyStopError,
    JoggingMode,
    JoggingNotSupportedError,
    MotionError,
    StopCondition,
    StopContext,
    WaypointConfig,
)

if TYPE_CHECKING:
    from novapolicy.lerobot import LeRobotPolicyClient


def __getattr__(name: str) -> object:
    """Load backend-specific clients only when requested."""
    if name == "LeRobotPolicyClient":
        try:
            return importlib.import_module("novapolicy.lerobot").LeRobotPolicyClient
        except ModuleNotFoundError as exc:
            msg = (
                "LeRobotPolicyClient requires the LeRobot policy extra. "
                "Install with `python -m pip install 'wandelbots-nova[novapolicy-lerobot]'`."
            )
            raise ModuleNotFoundError(msg) from exc
    raise AttributeError(name)


__all__ = [
    "Action",
    "ActionChunk",
    "ActionMode",
    "BoolMapping",
    "CallbackPolicyClient",
    "CameraSource",
    "ConnectedActionChunk",
    "EmergencyStopError",
    "ExecutionResult",
    "ExecutorStatus",
    "Gr00tPolicyClient",
    "InterpolatedActionChunk",
    "JoggingNotSupportedError",
    "JointJogger",
    "LeRobotPolicyClient",
    "Mapping",
    "MotionError",
    "Observation",
    "Phase",
    "PolicyClient",
    "PolicyExecutor",
    "PolicySchema",
    "RTCConfig",
    "StopCondition",
    "StopContext",
    "TcpJogger",
    "WaypointConfig",
    "WebRTCCameras",
    "connect_action_chunk",
    "create_bridge_chunk",
    "interpolate_action_chunk_ramps",
    "jog_joints",
    "jog_tcp",
    "smooth_action_chunk",
]
