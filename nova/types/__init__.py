from typing import AsyncIterator, Callable

from nova import api
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.types.state import MotionState, RobotState
from nova.types.vector3d import Vector3d

LoadPlanResponse = api.models.PlanSuccessfulResponse
InitialMovementStream = AsyncIterator[api.models.StreamMoveResponse]
InitialMovementConsumer = Callable[[api.models.StreamMoveResponse], None]
MovementResponse = api.models.ExecuteTrajectoryResponse | api.models.StreamMoveResponse
ExecuteTrajectoryRequestStream = AsyncIterator[api.models.ExecuteTrajectoryRequest]
ExecuteTrajectoryResponseStream = AsyncIterator[api.models.ExecuteTrajectoryResponse]
MovementControllerFunction = Callable[
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
