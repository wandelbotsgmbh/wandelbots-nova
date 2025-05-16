from typing import Any, Literal

from nova import api
from nova.actions.base import Action


class WriteAction(Action):
    type: Literal["Write"] = "Write"
    key: str
    value: Any
    device_id: str | None

    def to_api_model(self):
        return api.models.IOValue(io=self.key, boolean_value=self.value).model_dump()

    def is_motion(self) -> bool:
        return False


def io_write(key: str, value: Any, device_id: str | None = None) -> WriteAction:
    """Create a WriteAction

    Args:
        key: The key to write
        value: The value to write
        device_id: The device id

    Returns:
        The WriteAction

    """
    return WriteAction(key=key, value=value, device_id=device_id)


class ReadAction(Action):
    type: Literal["Read"] = "Read"
    key: str
    device_id: str

    def to_api_model(self):
        return super().model_dump()

    def is_motion(self) -> bool:
        return False


# TODO: Could move to WS if program representation is not in nova
class CallAction(Action):
    type: Literal["Call"] = "Call"
    device_id: str
    key: str
    arguments: list

    def to_api_model(self):
        return super().model_dump()

    def is_motion(self) -> bool:
        return False


# TODO: Could move to WS if program representation is not in nova
class ReadPoseAction(Action):
    type: Literal["ReadPose"] = "ReadPose"
    device_id: str
    tcp: str | None = None

    def to_api_model(self):
        return super().model_dump()

    def is_motion(self) -> bool:
        return False


# TODO: Could move to WS if program representation is not in nova
class ReadJointsAction(Action):
    type: Literal["ReadJoints"] = "ReadJoints"
    device_id: str

    def to_api_model(self):
        return super().model_dump()

    def is_motion(self) -> bool:
        return False
