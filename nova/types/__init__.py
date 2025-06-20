from typing import AsyncIterator, Callable, TypeAlias

from nova import api
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.types.state import MotionState, RobotState
from nova.types.vector3d import Vector3d

LoadPlanResponse: TypeAlias = api.models.AddTrajectoryResponse
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
    "LoadPlanResponse",
    "MovementResponse",
    "MotionState",
    "RobotState",
    "MotionSettings",
    "ExecuteTrajectoryRequestStream",
    "ExecuteTrajectoryResponseStream",
    "MovementControllerFunction",
]
