"""Off-thread Rerun writing: ordering, timestamps, and back-pressure."""

from __future__ import annotations

from itertools import pairwise
import threading
import time
from typing import Any, cast

import pytest

from novapolicy.rerun.logtime import elapsed_since, pinned_elapsed
from novapolicy.rerun.sink import AsyncLogSink


def _drain(sink: AsyncLogSink) -> None:
    sink.close(timeout_s=5.0)


def test_submitted_writes_run_off_the_calling_thread() -> None:
    """The whole point: the producer must not execute the write itself."""
    sink = AsyncLogSink()
    sink.start()
    seen: list[int] = []
    done = threading.Event()
    caller = threading.get_ident()

    def record() -> None:
        seen.append(threading.get_ident())
        done.set()

    sink.submit(record)
    assert done.wait(5.0)
    _drain(sink)

    assert seen and seen[0] != caller


def test_submit_does_not_wait_for_a_slow_write() -> None:
    """A slow entry must not stall the producer."""
    sink = AsyncLogSink()
    sink.start()
    started = threading.Event()

    def slow() -> None:
        started.set()
        time.sleep(0.2)

    sink.submit(slow)
    assert started.wait(5.0)
    t0 = time.perf_counter()
    for _ in range(50):
        sink.submit(lambda: None)
    submit_cost = time.perf_counter() - t0
    _drain(sink)

    assert submit_cost < 0.05, submit_cost


def test_writes_keep_their_order() -> None:
    sink = AsyncLogSink()
    sink.start()
    seen: list[int] = []
    for i in range(200):
        sink.submit(lambda i=i: seen.append(i))
    _drain(sink)

    assert seen == list(range(200))


def test_overflow_drops_the_oldest_not_the_newest() -> None:
    """Back-pressure keeps the present, because that is what a viewer shows."""
    sink = AsyncLogSink(depth=4)
    seen: list[int] = []
    # No worker started, so submissions run inline; start one that is blocked
    # to force the queue to fill.
    sink.start()
    release = threading.Event()
    sink.submit(release.wait)
    for i in range(50):
        sink.submit(lambda i=i: seen.append(i))
    release.set()
    _drain(sink)

    assert sink.dropped > 0
    # whatever survived, the last entry submitted must be among it
    assert seen and seen[-1] == 49


def test_submit_runs_inline_when_no_worker_is_running() -> None:
    """Logging before start or after close still happens, it just blocks."""
    sink = AsyncLogSink()
    seen: list[str] = []

    sink.submit(lambda: seen.append("before-start"))
    assert seen == ["before-start"]

    sink.start()
    sink.close(timeout_s=5.0)
    sink.submit(lambda: seen.append("after-close"))
    assert seen == ["before-start", "after-close"]


def test_an_inline_write_never_overlaps_the_in_flight_worker() -> None:
    """Shutdown must not put two threads inside a write at once.

    ``close`` clears the worker before it has finished draining, so a submit
    landing in that window runs inline on the caller's thread while the worker is
    still mid-write. Both then mutate the same unsynchronised visualisation state
    — the TCP trails, among others — from two threads.
    """
    sink = AsyncLogSink()
    sink.start()
    guard = threading.Lock()
    active = 0
    overlapped = False
    in_write = threading.Event()
    release = threading.Event()

    def tracked(*, block: bool) -> None:
        nonlocal active, overlapped
        with guard:
            active += 1
            overlapped = overlapped or active > 1
        if block:
            in_write.set()
            release.wait(5.0)
        with guard:
            active -= 1

    sink.submit(lambda: tracked(block=True))
    assert in_write.wait(5.0)
    # Exactly what close() does before the worker has drained.
    sink._thread = None
    submitted = threading.Event()

    def submit_inline() -> None:
        sink.submit(lambda: tracked(block=False))
        submitted.set()

    threading.Thread(target=submit_inline, name="racing-producer").start()
    assert not submitted.wait(0.1)  # it has to wait for the in-flight write
    release.set()
    assert submitted.wait(5.0)

    assert overlapped is False


def test_close_stops_the_worker_even_with_a_full_queue() -> None:
    """Shutdown must not depend on a sentinel finding room in the queue.

    The queue is bounded, so on a backed-up sink there is none, and a close that
    can only signal by enqueueing a sentinel gives up once its put times out.
    The worker is then left parked on an empty queue forever — still holding the
    recording that teardown is about to disconnect.
    """
    sink = AsyncLogSink(depth=2)
    release = threading.Event()
    sink.start()
    worker = sink._thread
    assert worker is not None

    # Park the worker inside a write, then fill the queue behind it so there is
    # no room for a sentinel for the whole of the close below.
    sink.submit(lambda: release.wait(5.0))
    time.sleep(0.05)
    for _ in range(10):
        sink.submit(lambda: None)

    sink.close(timeout_s=0.2)  # cannot drain yet: the worker is still parked

    # Once the parked write returns, the worker has to notice the close on its
    # own — nothing will hand it a sentinel after the fact.
    release.set()
    worker.join(5.0)
    assert not worker.is_alive()


def test_a_failing_write_does_not_kill_the_worker() -> None:
    sink = AsyncLogSink()
    sink.start()
    seen: list[str] = []

    def boom() -> None:
        raise RuntimeError("bad entry")

    sink.submit(boom)
    sink.submit(lambda: seen.append("still alive"))
    _drain(sink)

    assert seen == ["still alive"]


def test_pinned_elapsed_overrides_the_clock() -> None:
    """A deferred write is stamped when it was produced, not when it is written."""
    start = time.monotonic() - 10.0

    assert elapsed_since(start) > 9.0
    with pinned_elapsed(1.25):
        assert elapsed_since(start) == 1.25
    assert elapsed_since(start) > 9.0


def test_pinned_elapsed_is_per_thread() -> None:
    """One thread's pin must never stamp another thread's entries."""
    start = time.monotonic() - 10.0
    other: list[float] = []
    ready = threading.Event()

    def observe() -> None:
        other.append(elapsed_since(start))
        ready.set()

    with pinned_elapsed(1.25):
        thread = threading.Thread(target=observe)
        thread.start()
        thread.join(5.0)

    assert ready.is_set()
    assert other[0] > 9.0


class _Logger:
    """Just enough PolicyRerunLogger to exercise _defer's timestamping."""

    def __init__(self) -> None:
        from novapolicy.rerun.logger import PolicyRerunLogger

        self.impl = PolicyRerunLogger.__new__(PolicyRerunLogger)
        self.impl._start_time = time.monotonic()
        self.impl._start_wall = None
        self.impl._recording = None
        self.impl._sink = AsyncLogSink()
        self.stamps: list[float] = []

    def defer(self, at: float | None) -> None:
        from novapolicy.rerun.logtime import elapsed_since

        self.impl._defer(lambda: self.stamps.append(elapsed_since(self.impl._start_time)), at=at)


def test_a_pre_session_instant_never_stamps_the_timeline_days_in_the_past() -> None:
    """Guard the whole class of "monotonic zero means machine boot" bug.

    A zero-initialised instant is not a moment in this session; stamping with
    it puts the entry a machine-uptime ago and drags every plot's axis with it.
    """
    log = _Logger()

    log.defer(at=0.0)

    assert log.stamps and log.stamps[0] >= 0.0
    assert log.stamps[0] < 1.0


def test_an_earlier_instant_is_honoured_when_it_is_within_the_session() -> None:
    """The normal case must still be able to backdate."""
    log = _Logger()
    produced = log.impl._start_time + 0.25

    log.defer(at=produced)

    assert log.stamps[0] == pytest.approx(0.25, abs=1e-6)


def test_a_pre_session_instant_does_not_backdate_log_time_either() -> None:
    """The clamp has to correct the instant, not just the derived elapsed value.

    ``log_time`` is stamped from the same moment. Clamping only ``policy_time``
    leaves that axis a machine-uptime in the past — the very corruption the
    clamp exists to prevent, just on the other timeline.
    """
    from novapolicy.rerun.logger import PolicyRerunLogger
    import rerun as rr

    stamps: list[float] = []
    original = rr.set_time

    def spy(timeline: str, **kw: object) -> None:
        if timeline == "log_time" and kw.get("timestamp") is not None:
            stamps.append(kw["timestamp"].timestamp())  # type: ignore[union-attr]

    log = PolicyRerunLogger.__new__(PolicyRerunLogger)
    log._start_time = time.monotonic()
    log._start_wall = None
    log._recording = None
    log._sink = AsyncLogSink()

    monkeypatched = cast("Any", rr)
    monkeypatched.set_time = spy
    try:
        # Zero in the monotonic domain is machine boot, i.e. days ago.
        log._defer(lambda: None, at=0.0)
    finally:
        monkeypatched.set_time = original

    assert len(stamps) == 1
    assert stamps[0] == pytest.approx(time.time(), abs=1.0)


def test_log_time_is_stamped_when_the_data_was_produced() -> None:
    """A burst written together must not collapse onto the wall-clock timeline.

    Rerun stamps its built-in ``log_time`` at write time. With writes handed to
    a worker, a burst of state packets is written within a millisecond of each
    other, so on that timeline they pile into a vertical step followed by a flat
    run — the staircase this exists to prevent.
    """
    from novapolicy.rerun.logger import PolicyRerunLogger
    import rerun as rr

    stamps: list[float] = []
    original = rr.set_time

    def spy(timeline: str, **kw: object) -> None:
        if timeline == "log_time" and kw.get("timestamp") is not None:
            stamps.append(kw["timestamp"].timestamp())  # type: ignore[union-attr]

    log = PolicyRerunLogger.__new__(PolicyRerunLogger)
    log._start_time = time.monotonic()
    log._start_wall = None
    log._recording = None
    log._sink = AsyncLogSink()
    log._sink.start()

    monkeypatched = cast("Any", rr)
    monkeypatched.set_time = spy
    try:
        for offset in (1.000, 1.008, 1.016):
            log._defer(lambda: None, at=log._start_time + offset)
        log._sink.close(timeout_s=5.0)
    finally:
        monkeypatched.set_time = original

    assert len(stamps) == 3
    gaps = [b - a for a, b in pairwise(stamps)]
    assert all(g == pytest.approx(0.008, abs=1e-4) for g in gaps), gaps
