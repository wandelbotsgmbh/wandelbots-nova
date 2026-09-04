"""Timestamping for Rerun entries that are written off the producing thread.

Rerun entries are stamped with ``time.monotonic() - start_time``. Read at the
moment the entry is written, that is the right answer only when the writer is
also the producer. When a worker thread does the writing the read happens
whenever the worker got round to it, so a backlog stretches the timeline and
shows up in the viewer as data drifting later than it happened.

The producer therefore pins the elapsed time it measured, and the writer stamps
with that instead of re-reading the clock. The pin is thread-local, so it can
only ever affect the writer that set it.
"""

from __future__ import annotations

from contextlib import contextmanager
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Iterator

_pinned = threading.local()


@contextmanager
def pinned_elapsed(elapsed: float) -> Iterator[None]:
    """Stamp everything logged in this block at ``elapsed``, not at "now"."""
    previous = getattr(_pinned, "value", None)
    _pinned.value = elapsed
    try:
        yield
    finally:
        _pinned.value = previous


def elapsed_since(start_time: float) -> float:
    """Elapsed time to stamp an entry with: the pinned value, else the clock."""
    pinned = getattr(_pinned, "value", None)
    if pinned is not None:
        return pinned
    return time.monotonic() - start_time
