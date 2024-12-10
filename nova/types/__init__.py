from wandelbots_api_client.models import *  # noqa: F401, F403
from nova.types.pose import Pose
from nova.types.vector3d import Vector3d
from nova.types.action import Action, Motion, MotionSettings, lin, spl, ptp, cir, jnt


# This is the stream of requests that is send to execute trajectory websocket
from typing import AsyncGenerator, Callable
import wandelbots_api_client as wb

from nova.types.movement_controller_context import MovementControllerContext

ExecuteTrajectoryRequestStream = AsyncGenerator[wb.models.ExecuteTrajectoryRequest, None]
ExecuteTrajectoryResponseStream = AsyncGenerator[wb.models.ExecuteTrajectoryResponse, None]
MovementControllerFunction = Callable[
    [ExecuteTrajectoryResponseStream], ExecuteTrajectoryRequestStream
]
LoadPlanResponse = wb.models.PlanSuccessfulResponse
InitialMovementStream = AsyncGenerator[wb.models.StreamMoveResponse, None]
InitialMovementConsumer = Callable[[wb.models.StreamMoveResponse], None]
MovementController = Callable[[MovementControllerContext], MovementControllerFunction]


__all__ = [
    "Vector3d",
    "Pose",
    "Motion",
    "MotionSettings",
    "lin",
    "spl",
    "ptp",
    "cir",
    "jnt",
    "Action",
    "ExecuteTrajectoryRequestStream",
    "ExecuteTrajectoryResponseStream",
    "MovementControllerFunction",
    "LoadPlanResponse",
    "InitialMovementStream",
    "InitialMovementConsumer",
    "MovementController",
]
