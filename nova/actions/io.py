from typing import Literal

from nova import api
from nova.actions.base import Action


class WriteAction(Action):
    type: Literal["Write"] = "Write"
    key: str
    value: bool | int | float
    device_id: str | None
    origin: api.models.IOOrigin = api.models.IOOrigin.CONTROLLER
    at: api.models.AtTrigger | None = None
    """Optional path trigger placing this write between two motions.

    See :mod:`nova.actions.path_trigger`. When ``None`` the write fires at the motion
    boundary given by its position in the action list (the default behaviour).
    """

    def to_api_model(
        self,
    ) -> api.models.IOBooleanValue | api.models.IOIntegerValue | api.models.IOFloatValue:
        if isinstance(self.value, bool):
            return api.models.IOBooleanValue(io=self.key, value=self.value)
        elif isinstance(self.value, int):
            return api.models.IOIntegerValue(io=self.key, value=str(self.value))
        elif isinstance(self.value, float):
            return api.models.IOFloatValue(io=self.key, value=self.value)
        else:
            raise ValueError(f"Invalid value type: {type(self.value)}")

    def is_motion(self) -> bool:
        return False


def io_write(
    key: str,
    value: bool | int | float,
    device_id: str | None = None,
    origin: api.models.IOOrigin = api.models.IOOrigin.CONTROLLER,
    at: api.models.AtTrigger | None = None,
) -> WriteAction:
    """Create a WriteAction

    Args:
        key: The key to write
        value: The value to write
        device_id: The device id
        origin: The IO origin (controller or bus)
        at: Optional path trigger to fire this write at a precise point within the
            motion that follows it in the action list. Build one with the helpers in
            ``nova.actions``: ``after_start(seconds=... | millimeters=...)`` measures
            from the start of that motion, ``before_target(seconds=... | millimeters=...)``
            back from its target, ``at_path_fraction(f)`` is a fraction ``[0, 1)`` of it.
            When omitted, the write fires at the motion boundary given by its position
            in the action list, i.e. when the following motion starts.

    Returns:
        The WriteAction

    """
    return WriteAction(key=key, value=value, device_id=device_id, origin=origin, at=at)


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
