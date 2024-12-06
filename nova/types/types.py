# This is the stream of requests that is send to execute trajectory websocket
from typing import AsyncGenerator, Callable
import wandelbots_api_client as wb

from nova.types.movement_controller_context import MovementControllerContext

ExecuteTrajectoryRequestStream = AsyncGenerator[wb.models.ExecuteTrajectoryRequest, None]

# This is the stream of responses that is received from execute trajectory websocket
ExecuteTrajectoryResponseStream = AsyncGenerator[wb.models.ExecuteTrajectoryResponse, None]

# A movement controller manages how the movement should happen using the execute_trajectory websocket
# We can decide to speed up move forward, slow down, stop, etc.
MovementControllerFunction = Callable[[ExecuteTrajectoryResponseStream], ExecuteTrajectoryRequestStream]

# This is the response we get when the plan loaded successfully
LoadPlanResponse = wb.models.PlanSuccessfulResponse

# This is the response we get when we try to move to the start position
InitialMovementStream = AsyncGenerator[wb.models.StreamMoveResponse, None]
InitialMovementConsumer = Callable[[wb.models.StreamMoveResponse], None]

# This is a function which takes a context and returns movement controller
MovementController = Callable[[MovementControllerContext], MovementControllerFunction]
