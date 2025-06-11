from typing import AsyncIterator, Callable, TypeAlias

import nova.api as api
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.types.state import MotionState, RobotState
from nova.types.vector3d import Vector3d

LoadPlanResponse: TypeAlias = api.models.PlanSuccessfulResponse
InitialMovementStream: TypeAlias = AsyncIterator[api.models.StreamMoveResponse]
InitialMovementConsumer: TypeAlias = Callable[[api.models.StreamMoveResponse], None]
MovementResponse: TypeAlias = api.models.ExecuteTrajectoryResponse | api.models.StreamMoveResponse
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
    "InitialMovementStream",
    "InitialMovementConsumer",
    "MovementResponse",
    "MotionState",
    "RobotState",
    "MotionSettings",
    "ExecuteTrajectoryRequestStream",
    "ExecuteTrajectoryResponseStream",
    "MovementControllerFunction",
]
