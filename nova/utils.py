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

class StreamExtractor:
    def __init__(self, controller_generator, stop_selector=None):
        self._queue = asyncio.Queue()
        self._controller_generator = controller_generator
        self._stop_selector = stop_selector or (lambda x: x is None)

    def __call__(self, in_stream):
        async def in_wrapper(in_stream_):
            async for in_value in in_stream_:
                if self._stop_selector(in_value):
                    self._queue.put_nowait(None)
                else:
                    self._queue.put_nowait(in_value)
                yield in_value

        return self._controller_generator(in_wrapper(in_stream))

    def __aiter__(self):
        return self
    
    async def __anext__(self):
        value = await self._queue.get()
        if value is None:
            raise StopAsyncIteration
        return value

#aiostream.stream.preserve(in_stream)
#aiostream.stream.until(stop_selector)