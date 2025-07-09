import asyncio
from typing import AsyncIterable, AsyncIterator, Callable, Generic, TypeVar

In = TypeVar("In")
Out = TypeVar("Out")

from datetime import datetime

from icecream import ic

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")


class StreamExtractor(Generic[In, Out]):
    def __init__(
        self,
        wrapped: Callable[[AsyncIterator[In]], AsyncIterable[Out]],
        stop_selector: Callable[[In], bool] | None = None,
    ):
        self._in_queue: asyncio.Queue[In | None] = asyncio.Queue()
        self._wrapped = wrapped
        self._stop_selector = stop_selector or (lambda x: x is None)

    def __call__(self, in_stream: AsyncIterable[In]) -> AsyncIterable[Out]:
        async def in_wrapper(in_stream_) -> AsyncIterator[In]:
            async for in_value in in_stream_:
                if self._stop_selector(in_value):
                    self._in_queue.put_nowait(None)
                else:
                    self._in_queue.put_nowait(in_value)
                yield in_value

        return self._wrapped(in_wrapper(in_stream))

    def __aiter__(self) -> AsyncIterator[In]:
        return self

    async def __anext__(self) -> In:
        value = await self._in_queue.get()
        if value is None:
            raise StopAsyncIteration
        return value


from aiostream import stream

AltIn = TypeVar("AltIn")


class MergingStreamExtractor(Generic[In, AltIn, Out]):
    def __init__(
        self,
        wrapped: Callable[[AsyncIterator[In]], AsyncIterable[Out]],
        alt_stream: AsyncIterable[AltIn],
        stop_selector: Callable[[In | AltIn], bool] | None = None,
    ):
        self._in_queue: asyncio.Queue[In | AltIn | None] = asyncio.Queue()
        self._wrapped = wrapped
        self._alt_stream = alt_stream
        self._stop_selector = stop_selector or (lambda x: x is None)

    def __call__(self, in_stream: AsyncIterable[In]) -> AsyncIterable[Out]:
        async def in_wrapper(in_stream_) -> AsyncIterator[In, AltIn]:
            merged_in_stream = stream.merge(
                in_stream_,
                self._alt_stream,
                key=lambda x: x,  # Use the value itself as the key
            )
            async for in_value in merged_in_stream.stream():
                # async for in_value in in_stream_:
                if self._stop_selector(in_value):
                    self._in_queue.put_nowait(None)
                else:
                    self._in_queue.put_nowait(in_value)
                yield in_value

        return self._wrapped(in_wrapper(in_stream))

    def __aiter__(self) -> AsyncIterator[In | AltIn]:
        return self

    async def __anext__(self) -> In | AltIn:
        value = await self._in_queue.get()
        if value is None:
            raise StopAsyncIteration
        return value


async def jogging()