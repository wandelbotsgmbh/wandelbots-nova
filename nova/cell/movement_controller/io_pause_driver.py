"""Drive a movement to its end through controller-side IO pauses.

Shared by the one-shot movement controller (:func:`move_forward`, one motion group) and
:meth:`TrajectoryExecutor.execute` (several groups through the start barrier). Both own a
cursor-like object whose ``forward()`` starts or resumes the movement and whose ``pause()``
stops it on path; the driver only needs those as callables, plus the IO waits from
:class:`nova.cell.io_condition.IOConditionWatcher`.

What the driver does, per stop of the movement:

- **finished** (the operation resolved without an IO pause): return.
- **IO pause** (the controller paused on ``pause_on_io``): wait for the release, start
  again. Without a release waiter (``resume_on_io_release=False``) the pause is a stop
  condition: stop and return.
- **signal source lost** (the bus-IO service went away, so the controller stopped
  evaluating the condition and keeps moving): pause the robot from the SDK, then handle
  it like an IO pause; without a release waiter raise
  :class:`~nova.cell.io_condition.IOConditionUnavailable`, since a stop condition can no
  longer be watched.
- **condition holds again after a resume**: the controller accepts a start received while
  the condition holds but leaves the robot paused at the same location, which looks
  exactly like the pause before it, so the operation may never resolve. Waiting for the
  release and starting again covers that and the ordinary second pause alike.
- **a watcher failed** (e.g. the NATS subscription for the bus-loss guard): pause the
  robot and raise the error at once, instead of moving on unguarded.
"""

import asyncio
import contextlib
import logging
from collections.abc import Awaitable, Callable
from enum import Enum
from typing import Generic, TypeVar

from nova.cell.io_condition import IOConditionUnavailable

logger = logging.getLogger(__name__)

R = TypeVar("R")

# Upper bound for the pause the driver sends itself to settle; the robot ramps down
# on path in well under a second.
_PAUSE_SETTLE_TIMEOUT_SECS = 10.0


class _Stop(Enum):
    FINISHED = "finished"
    IO_PAUSE = "IO condition"
    SIGNAL_LOST = "signal source lost"


class IOPauseDriver(Generic[R]):
    """Run one movement to its end through IO pauses; see the module docstring.

    Args:
        operation: The movement already started (the first ``forward()``).
        start: Starts the movement again from where it stands; returns its operation.
        pause: Pauses the movement on path; returns the pause's operation, or None when
            nothing is moving.
        paused_on_io: Whether a resolved operation stopped on the IO condition.
        location: The current trajectory location, for logging.
        wait_for_release: Returns once the condition no longer holds; None makes the
            condition a stop condition.
        wait_for_hold: Returns once the condition holds; watched after every resume.
        wait_for_signal_loss: Returns once the condition's source is gone; watched while
            the robot moves.
    """

    def __init__(
        self,
        operation: Awaitable[R],
        *,
        start: Callable[[], Awaitable[R]],
        pause: Callable[[], Awaitable[object] | None],
        paused_on_io: Callable[[R], bool],
        location: Callable[[], float],
        wait_for_release: Callable[[], Awaitable[None]] | None,
        wait_for_hold: Callable[[], Awaitable[None]] | None = None,
        wait_for_signal_loss: Callable[[], Awaitable[None]] | None = None,
    ):
        self._operation = asyncio.ensure_future(operation)
        self._start = start
        self._pause = pause
        self._paused_on_io = paused_on_io
        self._location = location
        self._wait_for_release = wait_for_release
        self._wait_for_hold = wait_for_hold
        self._wait_for_signal_loss = wait_for_signal_loss

    async def run(self) -> bool:
        """Drive until the movement finished or stopped for good.

        Returns:
            False when it stopped on the IO condition without resuming (a stop
            condition), True otherwise — including an operation that ended with an
            error or was cancelled, which its owner reports.

        Raises:
            IOConditionUnavailable: The signal source went away under a stop condition,
                or a watcher could not observe it; the robot was paused first.
        """
        operation = self._operation
        resumed = False
        while True:
            stop = await self._next_stop(operation, watch_hold=resumed)
            if stop is _Stop.FINISHED:
                return True
            if self._wait_for_release is None:
                if stop is _Stop.SIGNAL_LOST:
                    raise IOConditionUnavailable(
                        f"The pause signal's source went away; the robot was paused at "
                        f"location {self._location()} because the stop condition can no "
                        f"longer be watched"
                    )
                logger.info("Movement stopped on the IO condition at location %s", self._location())
                return False
            logger.info(
                "Movement paused (%s) at location %s — waiting for the signal to allow motion",
                stop.value,
                self._location(),
            )
            await self._wait_for_release()
            logger.info("Pause signal cleared — resuming movement")
            # The start re-carries pause_on_io and the IO overlay, so the pause is
            # re-armed and remaining path outputs stay attached.
            operation = asyncio.ensure_future(self._start())
            resumed = True

    async def _next_stop(self, operation: asyncio.Future[R], *, watch_hold: bool) -> _Stop:
        watchers: dict[_Stop, asyncio.Task[None]] = {}
        if self._wait_for_signal_loss is not None:
            watchers[_Stop.SIGNAL_LOST] = asyncio.create_task(
                _call(self._wait_for_signal_loss), name="io-pause-signal-guard"
            )
        if watch_hold and self._wait_for_hold is not None:
            watchers[_Stop.IO_PAUSE] = asyncio.create_task(
                _call(self._wait_for_hold), name="io-pause-hold-watch"
            )
        try:
            done, _ = await asyncio.wait(
                [operation, *watchers.values()], return_when=asyncio.FIRST_COMPLETED
            )
        finally:
            for task in watchers.values():
                task.cancel()
            # return_exceptions: a watcher's own error is read below; only the
            # cancellation of this task itself propagates from here.
            await asyncio.gather(*watchers.values(), return_exceptions=True)

        if operation in done:
            if operation.cancelled():
                return _Stop.FINISHED  # detached or torn down by its owner
            error = operation.exception()
            if error is not None:
                logger.debug(f"Movement operation ended with an error: {error!r}")
                return _Stop.FINISHED  # the owner surfaces it
            return _Stop.IO_PAUSE if self._paused_on_io(operation.result()) else _Stop.FINISHED

        # A watcher fired while the operation is still open. The next start or pause
        # supersedes it; nobody awaits it any more.
        _retire(operation)
        for stop, task in watchers.items():
            if task not in done:
                continue
            error = task.exception()
            if error is not None:
                logger.error(f"Cannot watch the pause signal, pausing the movement: {error!r}")
                await self._pause_and_settle()
                raise error
            if stop is _Stop.SIGNAL_LOST:
                logger.warning("Pause signal source lost — pausing the movement from the SDK")
                await self._pause_and_settle()
            return stop
        raise AssertionError("asyncio.wait returned without a completed awaitable")

    async def _pause_and_settle(self) -> None:
        pause = self._pause()
        if pause is None:
            return
        with contextlib.suppress(Exception):
            await asyncio.wait_for(pause, _PAUSE_SETTLE_TIMEOUT_SECS)


async def _call(wait: Callable[[], Awaitable[None]]) -> None:
    await wait()


def _retire(operation: asyncio.Future) -> None:
    """Retrieve a superseded operation's outcome so asyncio does not log it as lost."""

    def consume(future: asyncio.Future) -> None:
        if not future.cancelled():
            future.exception()

    operation.add_done_callback(consume)
