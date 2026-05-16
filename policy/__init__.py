"""policy — real-time PID-controlled jogging from action chunks.

.. warning::
    **EXPERIMENTAL** — This package is under active development and not ready
    for production use. The API will have breaking changes between releases.
    Do not depend on it for stable deployments.

This package enables AI policies to stream joint position targets (+ optional IO commands)
to one or more motion groups. A PID velocity controller converts position errors into
velocity commands sent via the NOVA Jogging API.
"""

from __future__ import annotations

from policy.cameras import CameraSource, WebRTCCameras
from policy.executor import ExecutionResult, ExecutorStatus, PolicyExecutor
from policy.gr00t import Gr00tPolicyClient
from policy.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
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
    GuardState,
    GuardStopError,
    JoggingMode,
    MotionError,
    PidConfig,
    ProfileConfig,
    TrajectoryConfig,
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
    "GuardState",
    "GuardStopError",
    "JointJogger",
    "Mapping",
    "MotionError",
    "Observation",
    "PidConfig",
    "PolicyClient",
    "PolicyExecutor",
    "PolicySchema",
    "ProfileConfig",
    "TcpJogger",
    "TrajectoryConfig",
    "WebRTCCameras",
    "jog_joints",
    "jog_tcp",
]
