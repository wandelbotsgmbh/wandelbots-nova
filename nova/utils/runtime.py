from typing import Awaitable

import anyio

# from exceptiongroup import ExceptionGroup


async def stoppable_run(run: Awaitable[None], stop: Awaitable[None]) -> None:
    async def group():
        run_scope = anyio.CancelScope(shield=True)
        stop_scope = anyio.CancelScope()

        async def waiter():
            with stop_scope:
                await stop
                run_scope.cancel()

        async def runner():
            with run_scope:
                await run
                stop_scope.cancel()

        async with anyio.create_task_group() as tg:
            tg.start_soon(runner)
            tg.start_soon(waiter)

    try:
        await group()
    except ExceptionGroup as eg:
        # since we only have two tasks, we can be sure that the first exception is the one we want to raise
        # in case of debugging, one might want to log all exceptions
        raise eg.exceptions[0]
