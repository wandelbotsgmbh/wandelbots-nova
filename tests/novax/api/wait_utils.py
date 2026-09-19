import asyncio


async def wait_for_message_count(messages: list, count: int, timeout: float = 15.0) -> None:
    """Poll until ``messages`` holds at least ``count`` items, or ``timeout`` seconds elapse.

    Returns as soon as the target count is reached so tests avoid paying a fixed sleep. On
    timeout it returns quietly and lets the caller's own assertion report the precise failure.
    """
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while len(messages) < count and loop.time() < deadline:
        await asyncio.sleep(0.05)
