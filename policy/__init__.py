"""policy — real-time PID-controlled jogging from action chunks.

This package enables AI policies to stream joint position targets (+ optional IO commands)
to one or more motion groups. A PID velocity controller converts position errors into
velocity commands sent via the NOVA Jogging API.
"""

from __future__ import annotations

from policy.executor import ExecutionResult, ExecutorStatus, Phase, PolicyExecutor
from policy.feature_map import FeatureGroup, FeatureMap
from policy.gr00t_client import Gr00tPolicyClient
from policy.nats_client import NatsPolicyClient
from policy.policy_client import CallbackPolicyClient, PolicyClient, WebSocketPolicyClient
from policy.runner import PolicyRunner
from policy.types import (
    ActionChunk,
    EmergencyStopError,
    GuardState,
    GuardStopError,
    PolicyResponse,
    PolicyRunnerConfig,
)

__all__ = [
    "ActionChunk",
    "CallbackPolicyClient",
    "EmergencyStopError",
    "ExecutionResult",
    "ExecutorStatus",
    "FeatureGroup",
    "FeatureMap",
    "Gr00tPolicyClient",
    "GuardState",
    "GuardStopError",
    "NatsPolicyClient",
    "Phase",
    "PolicyClient",
    "PolicyExecutor",
    "PolicyResponse",
    "PolicyRunner",
    "PolicyRunnerConfig",
    "WebSocketPolicyClient",
]
