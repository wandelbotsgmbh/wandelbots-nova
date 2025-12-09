import time
from typing import AsyncIterable, AsyncIterator, TypeVar

T = TypeVar("T")


async def downsample_stream(
    stream: AsyncIterable[T], target_frequency: float | None = None
) -> AsyncIterator[T]:
    """Downsample an async stream to a target frequency.

    Args:
        stream: The async iterable to downsample
        target_frequency: Target frequency in Hz. If None, yields all items without downsampling.

    Yields:
        Items from the stream at the specified frequency.
    """
    if target_frequency is None:
        async for item in stream:
            yield item
        return

    min_interval = 1.0 / target_frequency
    last_process_time: float | None = None

    async for item in stream:
        current_time = time.time()
        if last_process_time is not None:
            elapsed = current_time - last_process_time
            if elapsed < min_interval:
                continue  # Skip this item, not enough time has passed
        last_process_time = current_time
        yield item
