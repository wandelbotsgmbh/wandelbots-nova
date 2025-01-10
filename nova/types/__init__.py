# This is the stream of requests that is send to execute trajectory websocket
from typing import AsyncGenerator, Callable

import wandelbots_api_client as wb

from nova.types.collision_scene import CollisionScene
from nova.types.pose import Pose
from nova.types.vector3d import Vector3d

LoadPlanResponse = wb.models.PlanSuccessfulResponse
InitialMovementStream = AsyncGenerator[wb.models.StreamMoveResponse, None]
InitialMovementConsumer = Callable[[wb.models.StreamMoveResponse], None]


__all__ = [
    "Vector3d",
    "Pose",
    "CollisionScene",
    "LoadPlanResponse",
    "InitialMovementStream",
    "InitialMovementConsumer",
]
