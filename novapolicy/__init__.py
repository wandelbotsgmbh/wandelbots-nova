"""novapolicy: run trained robot policies on NOVA through action chunk streaming.

.. warning::
    **EXPERIMENTAL**: the API will have breaking changes between releases.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

from novapolicy.cameras import CameraSource, WebRTCCameras
from novapolicy.executor import (
    ExecutionResult,
    ExecutorStatus,
    Phase,
    PolicyExecutor,
    resolve_actions,
)
from novapolicy.jogging.jogger import JointJogger, TcpJogger, jog_joints, jog_tcp
from novapolicy.mapping.ops import Clamp, OpDirection, Rad2Deg, Scale, ValueOp
from novapolicy.mapping.pose_sources import OmniverseScene, PoseSource
from novapolicy.mapping.poses import OrientationType
from novapolicy.mapping.schema import (
    Action,
    BoolMapping,
    Mapping,
    Observation,
    PolicySchema,
)
from novapolicy.pipeline import (
    Bridge,
    DropLate,
    EaseIn,
    EndpointRamp,
    HoldPadding,
    JointClamp,
    LiveBuffer,
    ServoHorizon,
    Smoothing,
    Timestamps,
    default_pipeline,
)
from novapolicy.providers.base import CallbackPolicyClient, PolicyClient, resolve_control_freq
from novapolicy.providers.gr00t import Gr00tPolicyClient
from novapolicy.providers.queued import QueuedPolicyClient
from novapolicy.types import (
    ActionChunk,
    ActionMode,
    ContinuousExecution,
    EmergencyStopError,
    ExecutionMode,
    JoggingMode,
    JoggingNotSupportedError,
    MotionError,
    SequentialExecution,
    StopCondition,
    StopContext,
    WaypointConfig,
)

if TYPE_CHECKING:
    from novapolicy.providers.leapp import LeappPolicyClient
    from novapolicy.providers.lerobot import LeRobotPolicyClient


def __getattr__(name: str) -> object:
    """Load backend-specific clients only when requested."""
    if name == "LeRobotPolicyClient":
        try:
            return importlib.import_module("novapolicy.providers.lerobot").LeRobotPolicyClient
        except ModuleNotFoundError as exc:
            msg = (
                "LeRobotPolicyClient requires the LeRobot policy extra. "
                "Install with `python -m pip install 'wandelbots-nova[novapolicy-lerobot]'`."
            )
            raise ModuleNotFoundError(msg) from exc
    if name == "LeappPolicyClient":
        try:
            return importlib.import_module("novapolicy.providers.leapp").LeappPolicyClient
        except ModuleNotFoundError as exc:
            msg = (
                "LeappPolicyClient requires the LEAPP policy extra. "
                "Install with `python -m pip install 'wandelbots-nova[novapolicy-leapp]'`."
            )
            raise ModuleNotFoundError(msg) from exc
    raise AttributeError(name)


__all__ = [
    "Action",
    "ActionChunk",
    "ActionMode",
    "BoolMapping",
    "Bridge",
    "CallbackPolicyClient",
    "CameraSource",
    "Clamp",
    "ContinuousExecution",
    "DropLate",
    "EaseIn",
    "EmergencyStopError",
    "EndpointRamp",
    "ExecutionMode",
    "ExecutionResult",
    "ExecutorStatus",
    "Gr00tPolicyClient",
    "HoldPadding",
    "JoggingNotSupportedError",
    "JointClamp",
    "JointJogger",
    "LeRobotPolicyClient",
    "LeappPolicyClient",
    "LiveBuffer",
    "Mapping",
    "MotionError",
    "Observation",
    "OmniverseScene",
    "OpDirection",
    "OrientationType",
    "Phase",
    "PolicyClient",
    "PolicyExecutor",
    "PolicySchema",
    "PoseSource",
    "QueuedPolicyClient",
    "Rad2Deg",
    "Scale",
    "SequentialExecution",
    "ServoHorizon",
    "Smoothing",
    "StopCondition",
    "StopContext",
    "TcpJogger",
    "Timestamps",
    "ValueOp",
    "WaypointConfig",
    "WebRTCCameras",
    "default_pipeline",
    "jog_joints",
    "jog_tcp",
    "resolve_actions",
    "resolve_control_freq",
]
