import asyncio
from typing import Callable, Generic, TypeVar

T = TypeVar("T")


class Callerator(Generic[T]):
    def __init__(self, callback: Callable[[T | None], None] | None):
        self._q: asyncio.Queue[T | None] = asyncio.Queue()
        self.callback = callback

    def __call__(self, value: T | None):
        if self.callback:
            self.callback(value)
        self._q.put_nowait(value)

    async def stream(self):
        while True:
            value = await self._q.get()
            if value is None:
                break
            yield value
