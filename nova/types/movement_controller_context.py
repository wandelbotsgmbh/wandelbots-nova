import pydantic

from nova.types.action import CombinedActions


class MovementControllerContext(pydantic.BaseModel):
    combined_actions: CombinedActions
    motion_id: str
