import asyncio
from typing import AsyncIterable, AsyncIterator, Callable, Generic, TypeVar

T = TypeVar("T")


class StreamExtractor(Generic[T]):
    def __init__(self, wrapped: Callable[[AsyncIterable[T]], AsyncIterable[T]], stop_selector: Callable[[T], bool] | None = None):
        self._queue: asyncio.Queue[T | None] = asyncio.Queue()
        self._wrapped = wrapped
        self._stop_selector = stop_selector or (lambda x: x is None)

    def __call__(self, in_stream: AsyncIterable[T]) -> AsyncIterable[T]:
        async def in_wrapper(in_stream_) -> AsyncIterable[T]:
            async for in_value in in_stream_:
                if self._stop_selector(in_value):
                    self._queue.put_nowait(None)
                else:
                    self._queue.put_nowait(in_value)
                yield in_value

        return self._wrapped(in_wrapper(in_stream))

    def __aiter__(self) -> AsyncIterator[T]:
        return self

    async def __anext__(self) -> T:
        value = await self._queue.get()
        if value is None:
            raise StopAsyncIteration
        return value
