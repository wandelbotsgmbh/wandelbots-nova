from typing import AsyncIterator, Callable, TypeAlias

import nova.api as api
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.types.state import MotionState, RobotState
from nova.types.tcp import RobotTcp
from nova.types.vector3d import Vector3d

MovementResponse: TypeAlias = api.models.ExecuteTrajectoryResponse
ExecuteTrajectoryRequestStream: TypeAlias = AsyncIterator[api.models.ExecuteTrajectoryRequest]
ExecuteTrajectoryResponseStream: TypeAlias = AsyncIterator[api.models.ExecuteTrajectoryResponse]
MovementControllerFunction: TypeAlias = Callable[
    [ExecuteTrajectoryResponseStream], ExecuteTrajectoryRequestStream
]

__all__ = [
    "Vector3d",
    "Pose",
    "CollisionScene",
    "MovementResponse",
    "MotionState",
    "RobotState",
    "RobotTcp",
    "MotionSettings",
    "ExecuteTrajectoryRequestStream",
    "ExecuteTrajectoryResponseStream",
    "MovementControllerFunction",
]
