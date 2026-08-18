from typing import AsyncIterator, Callable, TypeAlias, Union

import nova.api as api
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.types.state import MotionState, RobotState
from nova.types.vector3d import Vector3d

ExecuteTrajectoryRequestStream: TypeAlias = AsyncIterator[
    Union[
        api.models.InitializeMovementRequest,
        api.models.StartMovementRequest,
        api.models.PauseMovementRequest,
        api.models.PlaybackSpeedRequest,
    ]
]
ExecuteTrajectoryResponseStream: TypeAlias = AsyncIterator[api.models.ExecuteTrajectoryResponse]
MovementControllerFunction: TypeAlias = Callable[
    [ExecuteTrajectoryResponseStream], ExecuteTrajectoryRequestStream
]

__all__ = [
    "Vector3d",
    "Pose",
    "CollisionScene",
    "MotionState",
    "RobotState",
    "MotionSettings",
    "ExecuteTrajectoryRequestStream",
    "ExecuteTrajectoryResponseStream",
    "MovementControllerFunction",
]
