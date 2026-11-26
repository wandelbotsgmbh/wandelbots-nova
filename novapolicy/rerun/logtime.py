"""Timestamping for Rerun entries written off the producing thread (ADR 0020).

Entries are stamped with ``time.monotonic() - start_time``, or with the elapsed time the producer
pinned; the pin is thread-local.
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
