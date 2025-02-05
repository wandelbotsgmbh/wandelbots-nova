from typing import Protocol, Union, runtime_checkable

# TODO: we should not restrict to ValueType here, the restriction was probably only for the use in WS 
ValueType = Union[int, str, bool, float, dts.Pose]


@runtime_checkable
class InputDevice(Protocol):
    """A device which supports reading from"""

    async def read(self, key: str) -> ValueType:
        """Read a value given its key"""


@runtime_checkable
class OutputDevice(Protocol):
    """A device which supports writing to"""

    async def write(self, key: str, value: ValueType) -> None:
        """Write a value given its key and the new value"""


@runtime_checkable
class IODevice(InputDevice, OutputDevice, Protocol):
    """A device which supports reading and writing"""
