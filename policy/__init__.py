"""policy — real-time PID-controlled jogging from action chunks.

This package enables AI policies to stream joint position targets (+ optional IO commands)
to one or more motion groups. A PID velocity controller converts position errors into
velocity commands sent via the NOVA Jogging API.
"""

from __future__ import annotations

from policy.cameras import CameraSet, WebRTCCameraConfig
from policy.executor import ExecutionResult, ExecutorStatus, Phase, PolicyExecutor
from policy.feature_map import FeatureGroup, FeatureMap, TcpFormat
from policy.gr00t_client import Gr00tPolicyClient
from policy.nats_client import NatsPolicyClient
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
    "EmergencyStopError",
    "ExecutionResult",
    "ExecutorStatus",
    "FeatureGroup",
    "FeatureMap",
    "Gr00tPolicyClient",
    "GuardState",
    "GuardStopError",
    "MotionError",
    "NatsPolicyClient",
    "Phase",
    "PolicyExecutor",
    "PolicyResponse",
    "PolicyRunner",
    "PolicyRunnerConfig",
    "TcpFormat",
    "WebRTCCameraConfig",
    "pose_to_tcp",
]
