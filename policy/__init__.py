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

from policy.cameras import CameraSet, CameraSource, WebRTCCameraConfig
from policy.executor import ExecutionResult, ExecutorStatus, Phase, PolicyExecutor
from policy.feature_map import FeatureGroup, FeatureMap, GroupObservation, TcpFormat
from policy.groot import Gr00tPolicyClient
from policy.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
from policy.nats import NatsPolicyClient
from policy.policy_client import CallbackPolicyClient, PolicyClient
from policy.pose import pose_to_tcp
from policy.runner import PolicyRunner
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
    "ActionChunk",
    "CallbackPolicyClient",
    "CameraSet",
    "CameraSource",
    "EmergencyStopError",
    "ExecutionResult",
    "ExecutorStatus",
    "FeatureGroup",
    "FeatureMap",
    "Gr00tPolicyClient",
    "GroupObservation",
    "GuardState",
    "GuardStopError",
    "JointJogger",
    "MotionError",
    "NatsPolicyClient",
    "Phase",
    "PolicyClient",
    "PolicyExecutor",
    "PolicyResponse",
    "PolicyRunner",
    "PolicyRunnerConfig",
    "TcpFormat",
    "TcpJogger",
    "WebRTCCameraConfig",
    "jog_joints",
    "jog_tcp",
    "pose_to_tcp",
]
