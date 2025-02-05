from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import AsyncIterable, Awaitable
from typing import Protocol, final, runtime_checkable, Generic, TypeVar


class AbstractDeviceState(Protocol):
    """A state of a device"""

    def __eq__(self, other: AbstractDeviceState) -> bool:
        """Check if the state is equal to another state"""


@runtime_checkable
class StateStreamingDevice(Protocol):
    """A device which supports streaming its state"""

    def state_stream(self, rate: int) -> AsyncIterable[AbstractDeviceState]:
        """Read a value given its key
        Args:
            rate: The rate at which the state should be streamed
        """


@runtime_checkable
class StoppableDevice(Protocol):
    """A device that can be stopped"""

    async def stop(self) -> None:
        """Stop the device"""


class Device(ABC):
    """A device that supports lifecycle management"""
    # TODO: do we need is_active?

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._is_active = False

    async def open(self) -> None:
        """Allocates the external hardware resource (i.e. establish a connection)"""
        self._is_active = True

    async def close(self):
        """Release the external hardware (i.e. close connection or set mode of external hardware back)"""
        self._is_active = False

    async def restart(self):
        if self._is_active:
            await self.close()
        await self.open()

    @final
    async def __aenter__(self):
        await self.open()
        return self

    @final
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()

    def __del__(self):
        # TODO: this cannt be async, hence awaiting close is not possible
        if self._is_active:
            # TODO(async) intentionally leaving this here to see if it matters
            self.close()


T = TypeVar("T")


class AsyncCallableDevice(Generic[T], Device):
    """An awaitable external function or service in the robot cell"""

    async def __call__(self, *args, **kwargs) -> Awaitable[T]:
        if not self._is_active:
            raise ValueError("The device is not activated.")
        return await self._call(*args)

    @abstractmethod
    async def _call(self, key, *args) -> Awaitable[T]:
        """The implementation of the call method. AbstractAwaitable guarantees that the device is activated.

        Args:
            key: A key that represents the identifier of the external function or service that is called
            *args: Parameters of the external callable

        Returns: the returned values of the external called function or service
        """