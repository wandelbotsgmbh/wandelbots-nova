from typing import AsyncGenerator, Callable

from nova import api
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.types.state import MotionState, RobotState
from nova.types.vector3d import Vector3d

LoadPlanResponse = api.models.PlanSuccessfulResponse
InitialMovementStream = AsyncGenerator[api.models.StreamMoveResponse, None]
InitialMovementConsumer = Callable[[api.models.StreamMoveResponse], None]
ExecuteTrajectoryRequestStream = AsyncGenerator[api.models.ExecuteTrajectoryRequest, None]
ExecuteTrajectoryResponseStream = AsyncGenerator[api.models.ExecuteTrajectoryResponse, None]
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
    "MotionState",
    "RobotState",
    "MotionSettings",
    "ExecuteTrajectoryRequestStream",
    "ExecuteTrajectoryResponseStream",
    "MovementControllerFunction",
]
