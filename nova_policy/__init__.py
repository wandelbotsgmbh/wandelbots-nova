"""nova_policy — real-time PID-controlled jogging from action chunks.

This package enables AI policies to stream joint position targets (+ optional IO commands)
to one or more motion groups. A PID velocity controller converts position errors into
velocity commands sent via the NOVA Jogging API.
"""

from __future__ import annotations

from nova_policy.executor import EpisodeResult, ExecutorStatus, Phase, PolicyExecutor
from nova_policy.feature_map import FeatureGroup, FeatureMap
from nova_policy.gr00t_client import Gr00tPolicyClient
from nova_policy.nats_client import NatsPolicyClient
from nova_policy.policy_client import CallbackPolicyClient, PolicyClient
from nova_policy.runner import PolicyRunner
from nova_policy.types import (
    ActionChunk,
    GuardState,
    GuardStopError,
    PolicyResponse,
    PolicyRunnerConfig,
)

__all__ = [
    "ActionChunk",
    "CallbackPolicyClient",
    "EpisodeResult",
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
]
