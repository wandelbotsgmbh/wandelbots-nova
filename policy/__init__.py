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

from policy.cameras import CameraSource, WebRTCCameras
from policy.executor import ExecutionResult, ExecutorStatus, Phase, PolicyExecutor
from policy.gr00t import Gr00tPolicyClient, RTCConfig
from policy.jogging import JointJogger, TcpJogger, jog_joints, jog_tcp
from policy.policy_client import CallbackPolicyClient, PolicyClient
from policy.schema import (
    Action,
    BoolMapping,
    Mapping,
    Observation,
    PolicySchema,
)
from policy.types import (
    ActionChunk,
    ActionMode,
    EmergencyStopError,
    JoggingMode,
    MotionError,
    StopCondition,
    StopContext,
    WaypointConfig,
)

__all__ = [
    "Action",
    "ActionChunk",
    "ActionMode",
    "BoolMapping",
    "CallbackPolicyClient",
    "CameraSource",
    "EmergencyStopError",
    "ExecutionResult",
    "ExecutorStatus",
    "Gr00tPolicyClient",
    "JointJogger",
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
    "jog_joints",
    "jog_tcp",
]
