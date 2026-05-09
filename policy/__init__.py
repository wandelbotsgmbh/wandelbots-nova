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

from policy.cameras import CameraSet, CameraSource, WebRTCCameraConfig, WebRTCCameras
from policy.executor import ExecutionResult, ExecutorStatus, Phase, PolicyExecutor
from policy.gr00t import Gr00tPolicyClient
from policy.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
from policy.policy_client import CallbackPolicyClient, PolicyClient
from policy.pose import pose_to_tcp
from policy.runner import PolicyRunner
from policy.schema import (
    Action,
    BoolMapping,
    EnumMapping,
    GroupedObservation,
    IdentityMapping,
    Mapping,
    Observation,
    PolicySchema,
    ScaleMapping,
    TcpFormat,
)
from policy.types import (
    ActionChunk,
    EmergencyStopError,
    GuardState,
    GuardStopError,
    MotionError,
    PolicyResponse,
    PolicyRunnerConfig,
)

__all__ = [
    "Action",
    "ActionChunk",
    "BoolMapping",
    "CallbackPolicyClient",
    "CameraSet",
    "CameraSource",
    "EmergencyStopError",
    "EnumMapping",
    "ExecutionResult",
    "ExecutorStatus",
    "Gr00tPolicyClient",
    "GroupedObservation",
    "GuardState",
    "GuardStopError",
    "IdentityMapping",
    "JointJogger",
    "Mapping",
    "MotionError",
    "Observation",
    "Phase",
    "PolicyClient",
    "PolicyExecutor",
    "PolicyResponse",
    "PolicyRunner",
    "PolicyRunnerConfig",
    "PolicySchema",
    "ScaleMapping",
    "TcpFormat",
    "TcpJogger",
    "WebRTCCameraConfig",
    "WebRTCCameras",
    "jog_joints",
    "jog_tcp",
    "pose_to_tcp",
]
