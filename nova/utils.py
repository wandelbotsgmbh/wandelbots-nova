import asyncio
from typing import AsyncIterable, AsyncIterator, Callable, Generic, TypeVar

I = TypeVar("I")
O = TypeVar("O")
T = TypeVar("T")


class StreamExtractor(Generic[I, O]):
    def __init__(
        self,
        wrapped: Callable[[AsyncIterator[I]], AsyncIterable[O]],
        stop_selector: Callable[[I], bool] | None = None,
    ):
        self._in_queue: asyncio.Queue[I | None] = asyncio.Queue()
        self._wrapped = wrapped
        self._stop_selector = stop_selector or (lambda x: x is None)

    def __call__(self, in_stream: AsyncIterable[I]) -> AsyncIterable[O]:
        async def in_wrapper(in_stream_) -> AsyncIterator[I]:
            async for in_value in in_stream_:
                if self._stop_selector(in_value):
                    self._in_queue.put_nowait(None)
                else:
                    self._in_queue.put_nowait(in_value)
                yield in_value

        return self._wrapped(in_wrapper(in_stream))

    def __aiter__(self) -> AsyncIterator[I]:
        return self

    async def __anext__(self) -> I:
        value = await self._in_queue.get()
        if value is None:
            raise StopAsyncIteration
        return value