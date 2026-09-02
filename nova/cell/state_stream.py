"""One shared motion-group state stream per motion group, fanned out to its subscribers.

The SDK used to open a fresh ``stream_motion_group_state`` websocket for every
consumer (the execute relay, the trajectory cursor, viewers, ...). Every extra
socket is another full-rate stream the client must keep draining: a consumer
that stops draining pauses the websocket transport, a paused transport never
reads the server's close reply, and closing the socket then waits out the
library's ``close_timeout`` — the end-of-motion stall.

Here one pump task owns the websocket per motion group and only ever enqueues
into per-subscriber queues, so the transport is always drained. The websocket
is opened lazily on the first subscription and closed by the pump itself, in a
context that is never externally cancelled — never in a subscriber ``finally``,
a cancelled TaskGroup child, or a GC finalizer.
"""

import asyncio
import contextlib
import functools
import logging
from collections.abc import AsyncGenerator, AsyncIterable, AsyncIterator, Callable
from dataclasses import dataclass, field
from typing import Generic, TypeVar

from nova import api
from nova.utils.downsample import downsample_stream

logger = logging.getLogger(__name__)

T = TypeVar("T")

# What ``response_rate=None`` means server-side; used to order subscriber rates.
_SERVER_DEFAULT_RATE_MSECS = 200

# Bound per subscriber queue: a subscriber that is never consumed must not grow
# without limit; oldest states are dropped first.
_MAX_QUEUED_STATES = 1024


class _QueueSentinel:
    """Marker type used only as a sentinel for queue termination."""


_QUEUE_SENTINEL = _QueueSentinel()


class StreamBroadcaster(Generic[T]):
    """Broadcast a single async stream to multiple subscribers.

    A cursor tees its states into one internal queue, so it supports exactly one
    consumer; the session needs several (barrier arm-wait, drift monitors, user
    streams). A subscriber that closes its stream stops being fed; a
    subscriber that never consumes is bounded by dropping its oldest items.
    Subscribing after the source ended yields an immediately-finished stream.
    """

    def __init__(self, source: AsyncIterable[T]):
        self._source = source
        self._queues: list[asyncio.Queue[T | _QueueSentinel]] = []
        self._is_finished = False

    def subscribe(self) -> AsyncGenerator[T, None]:
        """Subscribe to every item emitted from now on."""
        if self._is_finished:

            async def ended_stream() -> AsyncGenerator[T, None]:
                return
                yield  # unreachable; makes this function an async generator

            return ended_stream()
        queue: asyncio.Queue[T | _QueueSentinel] = asyncio.Queue()
        self._queues.append(queue)
        return self._queue_iterator(queue)

    async def run(self) -> None:
        try:
            async for item in self._source:
                for queue in self._queues:
                    if queue.qsize() >= _MAX_QUEUED_STATES:
                        with contextlib.suppress(asyncio.QueueEmpty):
                            queue.get_nowait()
                    queue.put_nowait(item)
        finally:
            self._is_finished = True
            for queue in self._queues:
                queue.put_nowait(_QUEUE_SENTINEL)

    async def _queue_iterator(
        self, queue: asyncio.Queue[T | _QueueSentinel]
    ) -> AsyncGenerator[T, None]:
        try:
            while True:
                item = await queue.get()
                if isinstance(item, _QueueSentinel):
                    return
                yield item
        finally:
            if queue in self._queues:
                self._queues.remove(queue)


@dataclass
class _EndOfStream:
    """Terminal queue marker: the shared stream is over for this subscriber.

    Carries the upstream error when there was one, so each subscription can
    re-raise it; ``None`` means a graceful end (or the subscriber's own close).
    """

    error: BaseException | None = None


class StateSubscription:
    """One subscriber's live view of a shared motion-group state stream.

    Async-iterate it for states. ``aclose()`` only deregisters — it is cheap,
    idempotent, and safe while another task is blocked in ``__anext__``; the
    websocket itself is owned by the shared stream's pump and closed there once
    the last subscription is gone.
    """

    def __init__(
        self,
        deregister: Callable[["StateSubscription"], None],
        queue: "asyncio.Queue[api.models.MotionGroupState | _EndOfStream]",
        downsample_to_msecs: int | None = None,
    ):
        self._deregister = deregister
        self._queue = queue
        self._closed = False
        stream: AsyncIterator[api.models.MotionGroupState] = self._drain_queue()
        if downsample_to_msecs is not None:
            stream = downsample_stream(stream, 1000.0 / downsample_to_msecs)
        self._stream = stream

    def __aiter__(self) -> AsyncIterator[api.models.MotionGroupState]:
        return self

    async def __anext__(self) -> api.models.MotionGroupState:
        return await self._stream.__anext__()

    async def aclose(self) -> None:
        """Deregister from the shared stream.

        Deliberately never touches the websocket, so it stays trivially safe to
        call from a ``finally``, a cancelled task, or a GC-driven generator
        finalizer. Contains no await point: it completes even when the calling
        context is already cancelled.

        States already buffered at this point are still delivered to a consumer
        that keeps iterating; the end marker enqueued *behind* them is what ends
        the stream. That preserves at-least-the-buffered delivery for a consumer
        whose subscription is closed by another task — the execute relay's
        trailing frames must not be dropped by its own teardown.
        """
        if self._closed:
            return
        self._closed = True
        self._deregister(self)
        # Wake a __anext__ blocked on the empty queue so it ends instead of
        # hanging; enqueued behind any buffered states so those still drain.
        self._queue.put_nowait(_EndOfStream())

    async def _drain_queue(self) -> AsyncGenerator[api.models.MotionGroupState, None]:
        while True:
            item = await self._queue.get()
            if isinstance(item, _EndOfStream):
                if not self._closed and item.error is not None:
                    raise item.error
                return
            yield item


@dataclass
class _PumpGeneration:
    """One websocket lifetime: the pump task, its socket rate, its subscribers."""

    rate_msecs: int | None
    queues: "dict[StateSubscription, asyncio.Queue[api.models.MotionGroupState | _EndOfStream]]" = (
        field(default_factory=dict)
    )
    close_requested: asyncio.Event = field(default_factory=asyncio.Event)
    task: asyncio.Task | None = None
    # The one pending linger countdown; rescheduled whenever the last
    # subscriber leaves again, so an earlier countdown cannot fire early.
    linger_task: asyncio.Task | None = None


class SharedMotionGroupStateStream:
    """Fan one ``stream_motion_group_state`` websocket out to many subscribers.

    The websocket opens lazily on the first :meth:`subscribe` and closes — in
    the pump task's own, never externally cancelled context — when the last
    subscription deregisters (after ``linger_secs``). A later subscribe reopens
    it. The socket rate is fixed by the subscriber that opened it: a later
    subscriber asking for a slower rate is downsampled client-side, one asking
    for a faster rate gets the socket's slower rate and a warning.

    An upstream error ends every subscription with that error; a graceful
    upstream end just ends them. Either way the next subscribe opens a fresh
    websocket.
    """

    def __init__(
        self,
        open_stream: Callable[[int | None], AsyncGenerator[api.models.MotionGroupState, None]],
        name: str = "",
        linger_secs: float = 0.0,
    ):
        self._open_stream = open_stream
        self._name = name
        self._linger_secs = linger_secs
        self._generation: _PumpGeneration | None = None

    def subscribe(self, response_rate_msecs: int | None = None) -> StateSubscription:
        """Subscribe to every state emitted from now on.

        Args:
            response_rate_msecs: Rate at which this subscriber wants states, in
                milliseconds. ``None`` means the server default of 200 ms.
        """
        generation = self._generation
        if (
            generation is None
            or generation.close_requested.is_set()
            or (generation.task is not None and generation.task.done())
        ):
            generation = _PumpGeneration(rate_msecs=response_rate_msecs)
            generation.task = asyncio.create_task(
                self._pump(generation), name=f"motion-group-state-pump-{self._name}"
            )
            self._generation = generation

        socket_rate = (
            generation.rate_msecs
            if generation.rate_msecs is not None
            else _SERVER_DEFAULT_RATE_MSECS
        )
        requested_rate = (
            response_rate_msecs if response_rate_msecs is not None else _SERVER_DEFAULT_RATE_MSECS
        )
        downsample_to_msecs: int | None = None
        if requested_rate < socket_rate:
            logger.warning(
                f"Motion group state stream '{self._name}' is already open at "
                f"{socket_rate} ms; a new subscription asking for {requested_rate} ms "
                f"cannot make it faster and will receive states at {socket_rate} ms."
            )
        elif requested_rate > socket_rate:
            downsample_to_msecs = requested_rate

        queue: asyncio.Queue[api.models.MotionGroupState | _EndOfStream] = asyncio.Queue()
        subscription = StateSubscription(
            deregister=functools.partial(self._deregister, generation),
            queue=queue,
            downsample_to_msecs=downsample_to_msecs,
        )
        generation.queues[subscription] = queue
        return subscription

    async def aclose(self) -> None:
        """End all subscriptions and wait for the pump to close the websocket."""
        generation = self._generation
        if generation is None:
            return
        generation.close_requested.set()
        if generation.task is not None:
            await generation.task

    def _deregister(self, generation: _PumpGeneration, subscription: StateSubscription) -> None:
        generation.queues.pop(subscription, None)
        if generation.queues or generation.close_requested.is_set():
            return
        if self._linger_secs > 0:
            # One countdown per generation, measured from the *latest* departure:
            # a stale countdown from an earlier leave/rejoin cycle would fire at
            # the earlier deadline and cut the keep-alive short.
            if generation.linger_task is not None:
                generation.linger_task.cancel()
            generation.linger_task = asyncio.create_task(
                self._linger_then_close(generation), name=f"motion-group-state-linger-{self._name}"
            )
        else:
            generation.close_requested.set()

    async def _linger_then_close(self, generation: _PumpGeneration) -> None:
        await asyncio.sleep(self._linger_secs)
        if not generation.queues:
            generation.close_requested.set()

    async def _pump(self, generation: _PumpGeneration) -> None:
        """Drain the websocket into the subscriber queues, then close it.

        The pump is the only place the websocket is closed, and the pump is
        never cancelled from outside — closing therefore never runs in a
        subscriber ``finally``, a cancelled TaskGroup child, or a GC finalizer,
        where a paused transport would turn it into a ``close_timeout`` wait
        for the caller.
        """
        error: BaseException | None = None
        try:
            stream = self._open_stream(generation.rate_msecs)
            try:
                iterator = stream.__aiter__()
                close_wait = asyncio.ensure_future(generation.close_requested.wait())
                try:
                    while True:
                        next_frame = asyncio.ensure_future(iterator.__anext__())
                        await asyncio.wait(
                            {next_frame, close_wait}, return_when=asyncio.FIRST_COMPLETED
                        )
                        if generation.close_requested.is_set():
                            next_frame.cancel()
                            await asyncio.gather(next_frame, return_exceptions=True)
                            break
                        self._broadcast(generation, next_frame.result())
                finally:
                    close_wait.cancel()
                    # This generation stops serving here, but closing the
                    # websocket below is an await point: a subscribe() landing
                    # in that window must open a fresh generation instead of
                    # joining this one moments before its queues are flushed.
                    generation.close_requested.set()
            finally:
                await stream.aclose()
        except StopAsyncIteration:
            pass  # upstream ended gracefully
        except Exception as e:
            error = e
        finally:
            self._finish(generation, error)

    def _broadcast(self, generation: _PumpGeneration, state: api.models.MotionGroupState) -> None:
        for queue in generation.queues.values():
            if queue.qsize() >= _MAX_QUEUED_STATES:
                with contextlib.suppress(asyncio.QueueEmpty):
                    queue.get_nowait()
            queue.put_nowait(state)

    def _finish(self, generation: _PumpGeneration, error: BaseException | None) -> None:
        # Covers the paths that never reach the pump loop (open_stream raising):
        # a finished generation must never be joinable.
        generation.close_requested.set()
        if generation.linger_task is not None:
            generation.linger_task.cancel()
        if error is not None:
            logger.warning(f"Motion group state stream '{self._name}' failed: {error!r}")
        for queue in generation.queues.values():
            queue.put_nowait(_EndOfStream(error=error))
        generation.queues.clear()
        if self._generation is generation:
            self._generation = None


class MotionGroupStateStreamRegistry:
    """The shared state streams of one gateway, keyed by (cell, controller, motion group).

    Owned by the :class:`~nova.core.gateway.ApiGateway`, so every consumer that
    reaches the API through one gateway converges on one websocket per motion
    group. Keyed below ``MotionGroup`` because ``Controller.motion_group()``
    constructs a fresh instance per call.
    """

    def __init__(
        self,
        open_stream: Callable[
            [str, str, str, int | None], AsyncGenerator[api.models.MotionGroupState, None]
        ],
        linger_secs: float = 0.0,
    ):
        self._open_stream = open_stream
        self._linger_secs = linger_secs
        self._streams: dict[tuple[str, str, str], SharedMotionGroupStateStream] = {}

    def stream(
        self, cell: str, controller_id: str, motion_group_id: str
    ) -> SharedMotionGroupStateStream:
        """The shared stream for one motion group, created on first use."""
        key = (cell, controller_id, motion_group_id)
        shared = self._streams.get(key)
        if shared is None:
            shared = SharedMotionGroupStateStream(
                open_stream=functools.partial(
                    self._open_stream, cell, controller_id, motion_group_id
                ),
                name="/".join(key),
                linger_secs=self._linger_secs,
            )
            self._streams[key] = shared
        return shared

    async def aclose(self) -> None:
        """Close every shared stream; used by ``ApiGateway.close()``."""
        for shared in self._streams.values():
            await shared.aclose()
