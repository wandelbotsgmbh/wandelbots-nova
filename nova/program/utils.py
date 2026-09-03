import inspect
import io
from collections.abc import Awaitable
from pathlib import Path
from typing import TextIO

import anyio
from exceptiongroup import ExceptionGroup


class Tee(io.StringIO):
    def __init__(self, stream: TextIO, *args, **kwargs) -> None:
        self.stream = stream
        super().__init__(*args, **kwargs)

    def write(self, string: str) -> int:
        self.stream.write(string)
        return super().write(string)


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


def ensure_trailing_newline(s: str):
    if not s:
        return s
    return s if s[-1] == "\n" else s + "\n"


def executing_program_dir() -> Path | None:
    """Directory of the first @nova.program file executing on the call stack."""

    from nova.program.function import Program

    frame = inspect.currentframe()
    try:
        frame = frame.f_back if frame is not None else None  # skip this helper's own frame
        while frame is not None:
            if frame.f_code is Program.__call__.__code__:
                program = frame.f_locals.get("self")
                if isinstance(program, Program):
                    code = getattr(inspect.unwrap(program._wrapped), "__code__", None)
                    return Path(code.co_filename).resolve().parent if code is not None else None
            frame = frame.f_back
    finally:
        del frame
