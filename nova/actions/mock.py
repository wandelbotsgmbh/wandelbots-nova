from typing import Any, Literal

import pydantic

from nova.actions.base import Action


class WaitAction(Action):
    type: Literal["Wait"] = "Wait"
    wait_for_in_seconds: float = 0.0

    @pydantic.model_serializer
    def serialize_model(self) -> dict[str, Any]:
        return self.model_dump()

    def is_motion(self) -> bool:
        return False

    def to_api_model(self) -> dict[str, Any]:
        return {"type": self.type, "wait_for_in_seconds": self.wait_for_in_seconds}


def wait(wait_for_in_seconds: float) -> WaitAction:
    return WaitAction(wait_for_in_seconds=wait_for_in_seconds)
