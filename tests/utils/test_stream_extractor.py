import asyncio

import pytest

from nova.utils import StreamExtractor

# TODO: these test are not complete and don't test the stream extractor fully
# currently there are used to trouble shoot issues with async generators and
# stream extractor behavior
# keeping them here even when they are not complete because writing this from scratch is time consuming


@pytest.fixture
async def stream_extractor():
    def wrapper():
        async def out_stream(input_stream):
            async for value in input_stream:
                yield value * 2

        return out_stream

    extractor = StreamExtractor(wrapper(), stop_selector=lambda x: x >= 5)
    return extractor


@pytest.mark.asyncio
async def test_stream_extractor_drive_input_stream(stream_extractor):
    async def in_stream():
        for i in range(10):
            print(f":Producing input stream data {i}")
            yield i

    async def run():
        async for value in stream_extractor(in_stream()):
            print(f"Consuming output stream: {value}")
            await asyncio.sleep(1)

    execution_task = asyncio.create_task(run())

    try:
        async for value in stream_extractor:
            print(f"Consuming input stream: {value}")
            if value >= 5:
                raise Exception("Stream extractor send more data than expected.")
    except Exception:
        execution_task.cancel()
    finally:
        await execution_task


# if the async generator was in sleep it gets canceled


@pytest.mark.asyncio
async def test_async_generators():
    async def square():
        try:
            for i in range(10):
                yield i * 2
                # await asyncio.sleep(0.1)
        except BaseException as e:
            print(f"Error in square generator: {e}")
        finally:
            print("Square generator is closing.")

    async def numbers():
        try:
            async for i in square():
                yield i * 2
                # await asyncio.sleep(0.1)
        except BaseException as e:
            print(f"Error in numbers generator: {e}")

    async for n in numbers():
        if n > 10:
            raise Exception("Stopping numbers generator")
        print(n)
