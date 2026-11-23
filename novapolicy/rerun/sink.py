"""Off-thread writer for Rerun entries produced inside a control loop (ADR 0020).

A bounded queue that drops its oldest entries on overflow; entries keep the time they were
submitted (:mod:`novapolicy.rerun.logtime`).
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

# Entries the writer may fall behind by.
_DEFAULT_DEPTH = 512

# How often an idle worker checks whether a close is waiting on it [s].
_STOP_POLL_S = 0.05

# Errors producing or writing a log entry can raise; such an entry is skipped.
LOG_ERRORS = (OSError, RuntimeError, ValueError, TypeError, AttributeError)


class AsyncLogSink:
    """Runs submitted Rerun writes on a dedicated worker thread."""

    def __init__(self, *, depth: int = _DEFAULT_DEPTH) -> None:
        self._queue: queue.Queue[Callable[[], None] | None] = queue.Queue(maxsize=depth)
        self._thread: threading.Thread | None = None
        self._stopping = threading.Event()
        self._dropped = 0
        # Serialises the writes: after close() a submit runs inline while the worker may still
        # be draining onto the same state (TCP trails).
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
        """Hand a write to the worker without blocking; inline when no worker runs (before
        :meth:`start`, after :meth:`close`)."""
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
            pass  # another thread moved the queue: the entry is dropped, not retried

    def close(self, timeout_s: float = 2.0) -> None:
        """Flush what is queued and stop the worker; call it before the recording disconnects,
        or the tail of the run is lost."""
        thread = self._thread
        if thread is None:
            return
        self._thread = None
        # The flag stops the worker: a full queue may have no room for the sentinel (ADR 0020).
        self._stopping.set()
        # The sentinel, where it fits, stops an idle worker without waiting out its poll.
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
                # Drained: stop if a close is waiting.
                if self._stopping.is_set():
                    return
                continue
            if write is None:
                return
            self._run_once(write)

    def _run_once(self, write: Callable[[], None]) -> None:
        with self._write_lock:
            try:
                write()
            except LOG_ERRORS as e:
                logger.debug("Rerun log write failed: %s", e)
