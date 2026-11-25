"""Off-thread writer for Rerun entries produced inside a control loop.

Rerun logging is not free. One tick's worth of tracking and action-chunk entries
costs a meaningful fraction of a 10ms control tick, and occasionally a multiple of
one. Paid on the control loop that also has to produce the next waypoint chunk,
that lengthens ticks well past their budget and costs motion smoothness — observed
on a UR10e jogging run, though any loop this tight will have the same problem.

None of that work needs the caller to wait for it, so it is handed to a worker
thread. The Rerun SDK does its serialisation in Rust and releases the GIL for
it, so this genuinely gets the cost off the producing thread rather than just
moving it around. Entries carry the timestamp measured when they were submitted
(see :mod:`novapolicy.rerun.logtime`), so deferring the write does not move them
on the viewer's timeline.

The queue is bounded and drops its oldest entries when it overflows: this is
visualisation, and showing the present matters more than showing everything.
"""

from __future__ import annotations

import contextlib
import logging
import queue
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable

logger = logging.getLogger(__name__)

# Entries the writer may fall behind by. At ~90 entries/sec this is several
# seconds of slack — far more than a transient hiccup needs, and small enough
# that a writer which has genuinely stopped keeping up cannot grow without
# bound.
_DEFAULT_DEPTH = 512

# How often an idle worker checks whether a close is waiting on it. Short enough
# that shutdown is not perceptibly delayed, long enough not to spin.
_STOP_POLL_S = 0.05

# Errors producing or writing a log entry can raise. A bad entry must not take
# the writer down, nor the control loop that submitted it: reading a target off a
# half-populated state is as much a visualisation-only fault as a failed write.
LOG_ERRORS = (OSError, RuntimeError, ValueError, TypeError, AttributeError)


class AsyncLogSink:
    """Runs submitted Rerun writes on a dedicated worker thread."""

    def __init__(self, *, depth: int = _DEFAULT_DEPTH) -> None:
        self._queue: queue.Queue[Callable[[], None] | None] = queue.Queue(maxsize=depth)
        self._thread: threading.Thread | None = None
        self._stopping = threading.Event()
        self._dropped = 0
        # Serialises the writes themselves. :meth:`close` hands the worker its
        # sentinel and clears ``_thread`` before the worker has finished
        # draining, so a submit landing in that window runs inline on the
        # caller's thread while the worker is still mid-write. Both would then
        # mutate the same unsynchronised visualisation state (TCP trails, for
        # one) at once, which the Rerun writes were never written to survive.
        self._write_lock = threading.Lock()

    @property
    def dropped(self) -> int:
        """Entries discarded because the writer could not keep up."""
        return self._dropped

    def start(self) -> None:
        """Begin writing in the background. Idempotent."""
        if self._thread is not None:
            return
        self._stopping.clear()
        self._thread = threading.Thread(target=self._run, name="policy-rerun-sink", daemon=True)
        self._thread.start()

    def submit(self, write: Callable[[], None]) -> None:
        """Hand a write to the worker. Never blocks the caller.

        Runs inline when no worker is running, so logging still works before
        :meth:`start` and after :meth:`close` rather than silently vanishing.
        """
        if self._thread is None:
            self._run_once(write)
            return
        try:
            self._queue.put_nowait(write)
        except queue.Full:
            self._drop_oldest_for(write)

    def _drop_oldest_for(self, write: Callable[[], None]) -> None:
        """Make room by discarding the stalest entry, never the newest."""
        self._dropped += 1
        try:
            _ = self._queue.get_nowait()
            self._queue.put_nowait(write)
        except (queue.Empty, queue.Full):
            # Another thread moved the queue underneath us; the entry is
            # dropped rather than retried, which is the point of the bound.
            pass

    def close(self, timeout_s: float = 2.0) -> None:
        """Flush what is queued and stop the worker.

        Must run before the recording it writes to is disconnected, or the tail
        of the run is written to a closed stream and lost.
        """
        thread = self._thread
        if thread is None:
            return
        self._thread = None
        # The flag, not the sentinel, is what stops the worker. A sentinel alone
        # cannot: the queue is bounded, so on a backed-up sink there is no room
        # to put it, and a blocking put would just spend the whole timeout
        # failing to place it — leaving a worker that never learns to stop and
        # goes on writing to a recording that is about to be disconnected.
        self._stopping.set()
        # Still offered when there is room: it lets an idle worker stop at once
        # instead of waiting out its poll interval.
        with contextlib.suppress(queue.Full):
            self._queue.put_nowait(None)
        thread.join(timeout_s)
        if thread.is_alive():
            logger.debug("Rerun log sink did not drain within %.1fs", timeout_s)
        if self._dropped:
            logger.debug(
                "Rerun log sink dropped %d entries to keep up with the control loop",
                self._dropped,
            )

    def _run(self) -> None:
        while True:
            try:
                write = self._queue.get(timeout=_STOP_POLL_S)
            except queue.Empty:
                # Nothing left to write. Stop only if a close is waiting on us,
                # so the queue is drained first and shutdown never depends on a
                # sentinel finding room in a full queue.
                if self._stopping.is_set():
                    return
                continue
            if write is None:
                return
            self._run_once(write)

    def _run_once(self, write: Callable[[], None]) -> None:
        # Uncontended in steady state: only the worker ever writes. The lock is
        # there for the inline path, which can otherwise overlap the worker
        # during shutdown.
        with self._write_lock:
            try:
                write()
            except LOG_ERRORS as e:
                # Visualisation is best-effort; never let it break execution.
                logger.debug("Rerun log write failed: %s", e)
