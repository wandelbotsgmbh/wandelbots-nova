"""nova_policy — real-time PID-controlled jogging from action chunks.

This package enables AI policies to stream joint position targets (+ optional IO commands)
to one or more motion groups. A PID velocity controller converts position errors into
velocity commands sent via the NOVA Jogging API.
"""

from __future__ import annotations

from nova_policy.executor import EpisodeResult, ExecutorStatus, Phase, PolicyExecutor
from nova_policy.feature_map import FeatureGroup, FeatureMap
from nova_policy.policy_client import CallbackPolicyClient, PolicyClient, WebSocketPolicyClient
from nova_policy.runner import PolicyRunner
from nova_policy.types import (
    ActionChunk,
    GuardState,
    PolicyResponse,
    PolicyRunnerConfig,
    SafetyStopError,
)

__all__ = [
    "ActionChunk",
    "CallbackPolicyClient",
    "EpisodeResult",
    "ExecutorStatus",
    "FeatureGroup",
    "FeatureMap",
    "GuardState",
    "Phase",
    "PolicyClient",
    "PolicyExecutor",
    "PolicyResponse",
    "PolicyRunner",
    "PolicyRunnerConfig",
    "SafetyStopError",
    "WebSocketPolicyClient",
]
