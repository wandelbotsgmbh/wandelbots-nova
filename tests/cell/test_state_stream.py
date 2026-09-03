"""Unit tests for the shared motion-group state stream.

The upstream websocket is faked by :class:`FakeUpstream`, which counts opens
and acloses and can be fed states, ended gracefully, or failed — everything
the pump-owns-the-socket contract is asserted against.
"""

import asyncio
import logging

import pytest

from nova.cell.state_stream import (
    _MAX_QUEUED_STATES,
    MotionGroupStateStreamRegistry,
    SharedMotionGroupStateStream,
)

pytestmark = pytest.mark.asyncio


class FakeUpstream:
    """Stands in for the generated client's ``stream_motion_group_state``."""

    def __init__(self):
        self.open_rates: list[int | None] = []
        self.aclose_count = 0
        self.closed = asyncio.Event()
        self._feed: asyncio.Queue = asyncio.Queue()

    def feed(self, state) -> None:
        self._feed.put_nowait(("state", state))

    def end(self) -> None:
        self._feed.put_nowait(("end", None))

    def fail(self, error: Exception) -> None:
        self._feed.put_nowait(("error", error))

    def open(self, response_rate_msecs):
        self.open_rates.append(response_rate_msecs)
        return self._stream()

    async def _stream(self):
        try:
            while True:
                kind, value = await self._feed.get()
                if kind == "state":
                    yield value
                elif kind == "end":
                    return
                else:
                    raise value
        finally:
            self.aclose_count += 1
            self.closed.set()


@pytest.fixture
def upstream():
    return FakeUpstream()


@pytest.fixture
def shared(upstream):
    return SharedMotionGroupStateStream(open_stream=upstream.open, name="cell/ctrl/0@ctrl")


async def next_state(subscription, timeout: float = 1.0):
    return await asyncio.wait_for(subscription.__anext__(), timeout)


async def test_no_socket_before_first_subscribe(shared, upstream):
    await asyncio.sleep(0)
    assert upstream.open_rates == []


async def test_one_socket_serves_many_subscribers(shared, upstream):
    subscriptions = [shared.subscribe() for _ in range(3)]
    upstream.feed("s1")
    for subscription in subscriptions:
        assert await next_state(subscription) == "s1"
    assert upstream.open_rates == [None]
    assert upstream.aclose_count == 0


async def test_last_aclose_closes_upstream_in_the_pump(shared, upstream):
    first = shared.subscribe()
    second = shared.subscribe()
    upstream.feed("s1")
    assert await next_state(first) == "s1"

    await first.aclose()
    await asyncio.sleep(0)
    assert upstream.aclose_count == 0, "socket must stay open while a subscriber remains"

    await second.aclose()
    await asyncio.wait_for(upstream.closed.wait(), 1.0)
    assert upstream.aclose_count == 1


async def test_aclose_is_idempotent_and_wakes_a_blocked_anext(shared, upstream):
    subscription = shared.subscribe()
    blocked = asyncio.ensure_future(next_state(subscription))
    await asyncio.sleep(0)

    await subscription.aclose()
    await subscription.aclose()
    with pytest.raises(StopAsyncIteration):
        await blocked


async def test_states_buffered_before_a_cross_task_aclose_still_drain(shared, upstream):
    """The execute relay's subscription is aclosed by the execution task while
    the relay may still be draining: frames buffered before the close must be
    delivered, with the end marker behind them ending the stream — the old
    sentinel-queue design's guarantee."""
    subscription = shared.subscribe()
    upstream.feed("s1")
    upstream.feed("s2")
    while not upstream._feed.empty():
        await asyncio.sleep(0)
    for _ in range(3):
        await asyncio.sleep(0)  # both states are broadcast into the queue now

    await subscription.aclose()  # another task closes; nothing consumed yet
    assert [state async for state in subscription] == ["s1", "s2"]


async def test_reopen_after_full_close(shared, upstream):
    first = shared.subscribe()
    upstream.feed("s1")
    assert await next_state(first) == "s1"
    await first.aclose()
    await asyncio.wait_for(upstream.closed.wait(), 1.0)

    second = shared.subscribe()
    upstream.feed("s2")
    assert await next_state(second) == "s2"
    assert upstream.open_rates == [None, None]
    await second.aclose()


async def test_upstream_error_reaches_every_subscriber(shared, upstream):
    subscriptions = [shared.subscribe() for _ in range(2)]
    error = RuntimeError("connection lost")
    upstream.fail(error)
    for subscription in subscriptions:
        with pytest.raises(RuntimeError, match="connection lost"):
            await next_state(subscription)


async def test_graceful_upstream_end_ends_subscriptions(shared, upstream):
    subscription = shared.subscribe()
    upstream.feed("s1")
    upstream.end()
    received = [state async for state in subscription]
    assert received == ["s1"]


async def test_subscribe_after_upstream_end_opens_a_fresh_socket(shared, upstream):
    first = shared.subscribe()
    upstream.end()
    assert [state async for state in first] == []

    second = shared.subscribe()
    upstream.feed("s2")
    assert await next_state(second) == "s2"
    assert len(upstream.open_rates) == 2
    await second.aclose()


async def test_socket_rate_is_fixed_by_the_first_subscriber(shared, upstream):
    first = shared.subscribe(500)
    upstream.feed("s1")
    assert await next_state(first) == "s1"
    assert upstream.open_rates == [500]
    await first.aclose()


async def test_later_faster_subscriber_warns_and_keeps_the_socket(shared, upstream, caplog):
    first = shared.subscribe(1000)
    with caplog.at_level(logging.WARNING, logger="nova.cell.state_stream"):
        second = shared.subscribe(100)
    assert any("cannot make it faster" in message for message in caplog.messages)

    upstream.feed("s1")
    assert await next_state(second) == "s1"
    assert upstream.open_rates == [1000]
    await first.aclose()
    await second.aclose()


async def test_none_subscriber_is_never_downsampled(shared, upstream, caplog):
    """``None`` means the controller step rate — the fastest — so a ``None``
    subscriber joining an explicit-rate socket wants MORE than the socket
    delivers: it gets the warning and the socket's full feed, never a
    downsample to some assumed 200 ms default."""
    explicit = shared.subscribe(100)
    with caplog.at_level(logging.WARNING, logger="nova.cell.state_stream"):
        unrated = shared.subscribe()
    assert any("cannot make it faster" in message for message in caplog.messages)

    for index in range(3):
        upstream.feed(f"s{index}")
    assert [await next_state(unrated) for _ in range(3)] == ["s0", "s1", "s2"]
    await explicit.aclose()
    await unrated.aclose()


async def test_later_slower_subscriber_is_downsampled(shared, upstream):
    fast = shared.subscribe()  # opens the socket at the controller step rate (full rate)
    slow = shared.subscribe(60_000)  # one state a minute: a burst passes only its first state

    for index in range(3):
        upstream.feed(f"s{index}")
    assert [await next_state(fast) for _ in range(3)] == ["s0", "s1", "s2"]

    assert await next_state(slow) == "s0"
    with pytest.raises(asyncio.TimeoutError):
        await next_state(slow, timeout=0.05)

    await fast.aclose()
    await slow.aclose()


async def test_cancelled_subscriber_leaves_the_socket_open_for_others(shared, upstream):
    doomed = shared.subscribe()
    survivor = shared.subscribe()

    blocked = asyncio.ensure_future(next_state(doomed))
    await asyncio.sleep(0)
    blocked.cancel()
    await asyncio.gather(blocked, return_exceptions=True)
    await doomed.aclose()  # what a consumer's finally does after cancellation
    await asyncio.sleep(0)

    assert upstream.aclose_count == 0
    upstream.feed("s1")
    assert await next_state(survivor) == "s1"
    await survivor.aclose()


async def test_queue_is_bounded_dropping_oldest(shared, upstream):
    subscription = shared.subscribe()
    total = _MAX_QUEUED_STATES + 7
    for index in range(total):
        upstream.feed(index)
    upstream.feed("last")
    # Let the pump broadcast everything before consuming, so the subscriber
    # queue actually hits its bound instead of being drained concurrently.
    while not upstream._feed.empty():
        await asyncio.sleep(0)
    for _ in range(5):
        await asyncio.sleep(0)

    received = []
    while (state := await next_state(subscription)) != "last":
        received.append(state)
    assert len(received) < _MAX_QUEUED_STATES  # "last" occupies one of the bounded slots
    assert received[-1] == total - 1
    assert received[0] > 0, "oldest states must have been dropped"
    await subscription.aclose()


async def test_stream_aclose_ends_subscriptions_and_closes_the_socket(shared, upstream):
    subscription = shared.subscribe()
    upstream.feed("s1")
    assert await next_state(subscription) == "s1"

    await shared.aclose()
    assert upstream.aclose_count == 1
    with pytest.raises(StopAsyncIteration):
        await next_state(subscription)


async def test_subscribe_during_pump_teardown_gets_a_fresh_socket(upstream):
    """A subscribe landing while the pump is closing the old websocket must not
    join the dying generation (it would end after zero states, with no reopen)."""
    close_started = asyncio.Event()
    release_close = asyncio.Event()

    class HoldOnClose:
        """First socket only: closing it suspends until the test releases it."""

        def __init__(self, inner):
            self._inner = inner

        def __aiter__(self):
            return self

        async def __anext__(self):
            return await self._inner.__anext__()

        async def aclose(self):
            close_started.set()
            await release_close.wait()
            await self._inner.aclose()

    def open_stream(rate):
        stream = upstream.open(rate)
        return HoldOnClose(stream) if len(upstream.open_rates) == 1 else stream

    shared = SharedMotionGroupStateStream(open_stream=open_stream, name="cell/ctrl/0@ctrl")
    first = shared.subscribe()
    upstream.end()
    await asyncio.wait_for(close_started.wait(), 1.0)  # pump suspended in aclose

    second = shared.subscribe()
    upstream.feed("s2")
    assert await next_state(second) == "s2"
    assert len(upstream.open_rates) == 2, "the race-losing subscriber must get a fresh socket"

    release_close.set()
    with pytest.raises(StopAsyncIteration):
        await next_state(first)
    await second.aclose()


async def test_linger_is_measured_from_the_latest_departure(upstream):
    """A stale linger countdown from an earlier leave/rejoin cycle must not
    close the socket early relative to the most recent departure."""
    shared = SharedMotionGroupStateStream(
        open_stream=upstream.open, name="cell/ctrl/0@ctrl", linger_secs=0.4
    )
    first = shared.subscribe()
    upstream.feed("s1")
    assert await next_state(first) == "s1"
    await first.aclose()  # countdown 1: would fire 0.4 s from now

    await asyncio.sleep(0.1)
    second = shared.subscribe()  # rejoin within the window: same socket
    await asyncio.sleep(0.1)
    await second.aclose()  # countdown must restart from here (fires at ~0.6)

    await asyncio.sleep(0.25)  # past countdown 1's stale deadline (~0.4)
    assert not upstream.closed.is_set(), "the stale countdown must not close the socket"

    await asyncio.wait_for(upstream.closed.wait(), 2.0)
    assert upstream.open_rates == [None]


async def test_linger_keeps_the_socket_open_for_a_resubscriber(upstream):
    shared = SharedMotionGroupStateStream(
        open_stream=upstream.open, name="cell/ctrl/0@ctrl", linger_secs=0.05
    )
    first = shared.subscribe()
    upstream.feed("s1")
    assert await next_state(first) == "s1"
    await first.aclose()

    second = shared.subscribe()  # within the linger window: same socket
    upstream.feed("s2")
    assert await next_state(second) == "s2"
    assert upstream.open_rates == [None]

    await second.aclose()
    await asyncio.wait_for(upstream.closed.wait(), 1.0)


async def test_registry_shares_streams_per_motion_group():
    upstream = FakeUpstream()
    other_upstream = FakeUpstream()

    def open_stream(cell, controller_id, motion_group_id, rate):
        return (upstream if motion_group_id == "0@ctrl" else other_upstream).open(rate)

    registry = MotionGroupStateStreamRegistry(open_stream=open_stream)
    assert registry.stream("cell", "ctrl", "0@ctrl") is registry.stream("cell", "ctrl", "0@ctrl")
    assert registry.stream("cell", "ctrl", "0@ctrl") is not registry.stream(
        "cell", "ctrl", "1@ctrl"
    )

    subscription = registry.stream("cell", "ctrl", "0@ctrl").subscribe()
    other = registry.stream("cell", "ctrl", "1@ctrl").subscribe()
    upstream.feed("s1")
    assert await next_state(subscription) == "s1"

    await registry.aclose()
    assert upstream.aclose_count == 1
    assert other_upstream.aclose_count == 1
    with pytest.raises(StopAsyncIteration):
        await next_state(subscription)
    with pytest.raises(StopAsyncIteration):
        await next_state(other)


async def test_gateway_close_drains_its_registry(monkeypatch):
    from nova.config import NovaConfig
    from nova.core.gateway import ApiGateway

    gateway = ApiGateway(NovaConfig(host="http://localhost"))
    upstream = FakeUpstream()
    monkeypatch.setattr(
        gateway,
        "_open_motion_group_state_stream",
        lambda cell, controller_id, motion_group_id, rate: upstream.open(rate),
    )
    monkeypatch.setattr(
        gateway._motion_group_state_streams,
        "_open_stream",
        lambda cell, controller_id, motion_group_id, rate: upstream.open(rate),
    )

    subscription = gateway.motion_group_state_stream("cell", "ctrl", "0@ctrl").subscribe()
    upstream.feed("s1")
    assert await next_state(subscription) == "s1"

    await gateway.close()
    assert upstream.aclose_count == 1
    with pytest.raises(StopAsyncIteration):
        await next_state(subscription)
