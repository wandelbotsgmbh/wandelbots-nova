"""The synchronized execution session: the barrier handle and its sync strategy."""

import asyncio
import contextlib
import logging
from collections.abc import AsyncGenerator, AsyncIterable, Mapping
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar

from nova import api
from nova.actions.io import WriteAction
from nova.cell.movement_controller.trajectory_cursor import OperationResult, TrajectoryCursor
from nova.core.gateway import ApiGateway

logger = logging.getLogger(__name__)

# An unconfirmed sync write leaves the groups armed on an unknown IO image, so
# the barrier gives up rather than waiting for a controller that never answers.
_TRIGGER_CONFIRM_TIMEOUT = 5.0
_IO_POLL_INTERVAL = 0.05

# Same bound the cursor applies to its own state queue: a subscriber that is
# never consumed must not grow without limit.
_MAX_QUEUED_STATES = 1024

T = TypeVar("T")


class SyncDriver(Protocol):
    """The topology-specific pieces of the start barrier.

    :class:`IOSyncDriver` is the IO-based implementation; a custom driver can
    signal a start any other way.
    """

    def start_conditions(self) -> Mapping[str, api.models.StartOnIO]:
        """The condition each group's start is gated on, keyed by group.

        Returned whole so a session can check up front that all its groups are
        covered.
        """
        pass

    async def clear(self) -> None:
        """Put the cell into the not-released state, before the groups are armed."""
        pass

    async def release(self) -> None:
        """Start the groups; called once every group waits on its condition."""
        pass


class IOSyncDriver:
    """IO-based :class:`SyncDriver`: the start barrier as three IO pieces run
    through one cell's gateway.

    Three independent pieces describe the barrier for any topology: the IO write
    that puts the cell into the not-released state (``clear``), the IO write that
    releases it (``release``), and the condition each group's start *watches*
    (``watch``). They are deliberately not derived from one another — clear and
    release may target different values, IOs, or even controllers, and only the
    wiring's owner knows which.

    Topologies covered:

    - **Same controller / cell-wide bus IO**: one boolean trigger; every group
      watches it, clearing writes its inverse — derived by the builder's
      :meth:`TrajectoryExecutorBuilder.sync_on_io`.
    - **Physically wired controllers**: the writes go to one controller's
      output; each group watches its own input the wire lands on. Spelled out
      explicitly — the executor cannot know how a cell is wired.

    For ``CONTROLLER``-origin writes, ``device_id`` names the controller owning
    the IO; it may be omitted when all motion groups share one controller. It is
    ignored for ``BUS_IO`` origin.

    Args:
        clear: Written before the groups are armed, so no stale release value
            can start them early.
        release: Written once every group waits on its start condition.
        watch: The start condition per group, keyed like the executor's motion
            groups; every group needs one.
        api_client: Gateway of the cell that carries the sync IO.
        cell: The cell the writes and watches address.
    """

    def __init__(
        self,
        clear: WriteAction,
        release: WriteAction,
        watch: Mapping[str, api.models.StartOnIO],
        api_client: ApiGateway,
        cell: str,
    ):
        for action in (clear, release):
            if action.origin is not api.models.IOOrigin.BUS_IO and action.device_id is None:
                raise ValueError(
                    f"Sync IO write '{action.key}' needs a device_id naming its controller"
                )
        self._clear = clear
        self._release = release
        self._watch = watch
        self._api_client = api_client
        self._cell = cell

    def start_conditions(self) -> Mapping[str, api.models.StartOnIO]:
        return self._watch

    async def clear(self) -> None:
        await self._write(self._clear)

    async def release(self) -> None:
        await self._write(self._release)

    async def _write(self, action: WriteAction) -> None:
        """Write the IO and wait until the write is observable.

        Without the confirmation a group could arm its start condition while the
        IO image still carries the previous barrier's value and start immediately,
        desynchronizing the run.
        """
        io_value = action.to_api_model()
        async with asyncio.timeout(_TRIGGER_CONFIRM_TIMEOUT):
            if action.origin is api.models.IOOrigin.BUS_IO:
                await self._api_client.bus_ios_api.set_bus_io_values(
                    cell=self._cell, io_value=[io_value]
                )
                while True:
                    values = await self._api_client.bus_ios_api.get_bus_io_values(
                        cell=self._cell, ios=[action.key]
                    )
                    if values and values[0] == io_value:
                        break
                    await asyncio.sleep(_IO_POLL_INTERVAL)
            else:
                assert action.device_id is not None
                await self._api_client.controller_ios_api.set_output_values(
                    cell=self._cell, controller=action.device_id, io_value=[io_value]
                )
                await self._api_client.controller_ios_api.wait_for_io_event(
                    cell=self._cell,
                    controller=action.device_id,
                    wait_for_io_event_request=api.models.WaitForIOEventRequest(
                        io=io_value, comparator=api.models.Comparator.COMPARATOR_EQUALS
                    ),
                )
        logger.debug(f"Sync IO '{action.key}' set to {action.value}")


class _QueueSentinel:
    """Marker type used only as a sentinel for queue termination."""


_QUEUE_SENTINEL = _QueueSentinel()


class _StreamBroadcaster(Generic[T]):
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


async def _wait_until_waiting_for_io(states: AsyncIterable[api.models.MotionGroupState]) -> None:
    async for state in states:
        if state.execute is not None and isinstance(
            state.execute.details, api.models.TrajectoryDetails
        ):
            if isinstance(state.execute.details.state, api.models.TrajectoryWaitForIO):
                return
    raise RuntimeError("State stream ended before the group armed its start-on-IO trigger")


async def _cancel_and_wait(task: asyncio.Task | None) -> None:
    if task is None or task.done():
        return
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)


async def _gather_named(
    futures: dict[str, asyncio.Future[OperationResult]],
) -> dict[str, OperationResult]:
    results = await asyncio.gather(*futures.values())
    return dict(zip(futures.keys(), results))


@dataclass
class _ForwardIntent:
    """A forward request posted for ``control()`` to run as a barrier."""

    future: asyncio.Future[dict[str, OperationResult]]
    target_location: float | None
    playback_speed_in_percent: int | None


class MultiTrajectoryCursor:
    """Cursor-compatible handle over one synchronized execution session.

    Obtained from :meth:`TrajectoryExecutor.attach`; drives all groups together
    through the start barrier with the single cursor's interactive contract —
    ``forward`` / ``forward_to`` / ``pause``, ``current_location`` and
    ``stream_state``.

    Backward movement is not exposed: driving the barrier in reverse is
    mechanically symmetric but unverified against real controllers.
    """

    def __init__(self, cursors: dict[str, TrajectoryCursor], sync: SyncDriver):
        self._cursors = cursors
        self._sync = sync
        # One fan-out per group: a cursor tees its states into a single queue, but
        # the session has several consumers (barrier arm-wait, drift monitors,
        # user streams).
        self._broadcasters = {name: _StreamBroadcaster(cursor) for name, cursor in cursors.items()}
        # A pending intent not yet consumed is silently overwritten — only the
        # most recent one matters — and pause() may void it before it ever runs.
        self._pending_intent: _ForwardIntent | None = None
        self._intent_event = asyncio.Event()
        self._stop_event = asyncio.Event()
        # The in-flight barrier, owned by control()'s TaskGroup. pause() and a
        # superseding forward() cancel it; a real failure aborts the session.
        self._current: asyncio.Task[None] | None = None

    async def control(self) -> None:
        """Run the session loop; must be running for ``forward``/``pause`` to make
        progress (:meth:`TrajectoryExecutor.attach` drives it).

        Owns the TaskGroup, spawns the per-group state pumps into it, then serves
        the intent mailbox: a forward runs a barrier as a child task (so
        pause/supersede can cancel it), a stop ends the loop. A barrier that fails
        for real propagates out and aborts the session.
        """
        try:
            async with asyncio.TaskGroup() as task_group:
                for name in self._broadcasters:
                    task_group.create_task(
                        self._broadcasters[name].run(), name=f"state-broadcaster-{name}"
                    )
                while True:
                    await self._intent_event.wait()
                    # Take-and-clear with no await in between, so an intent
                    # posted after this read re-sets the event and cannot be lost.
                    intent, self._pending_intent = self._pending_intent, None
                    self._intent_event.clear()
                    if self._stop_event.is_set():
                        if intent is not None and not intent.future.done():
                            intent.future.cancel()
                        await _cancel_and_wait(self._current)
                        break
                    if intent is None:
                        continue  # woken by a pause that voided the mailbox
                    # A superseded barrier must be gone before the next one arms
                    # the groups: left running, its still-open subscriptions
                    # would match the new barrier's WaitForIO states and fire
                    # the trigger a second time.
                    await _cancel_and_wait(self._current)
                    self._current = task_group.create_task(
                        self._run_barrier(intent), name="multi-trajectory-cursor-forward"
                    )
        except BaseExceptionGroup as error_group:
            # Unwrap our own TaskGroup so a barrier's error reaches attach as
            # itself, not nested a level deeper by this extra group.
            if len(error_group.exceptions) == 1 and isinstance(
                error_group.exceptions[0], Exception
            ):
                raise error_group.exceptions[0] from error_group
            raise
        finally:
            # A session that died on its own must also fail late forward() calls.
            self._stop_event.set()

    def monitor_streams(self) -> dict[str, AsyncGenerator[api.models.MotionGroupState, None]]:
        """Per-group future-only subscriptions for the session monitors.

        Called during ``attach`` setup, before ``control`` starts the pumps, so
        no early state is missed."""
        return {name: broadcaster.subscribe() for name, broadcaster in self._broadcasters.items()}

    def request_stop(self) -> None:
        """Ask ``control`` to end its loop — called as the session tears down.

        The session-teardown counterpart of the single cursor's ``detach()``,
        with its exact shape: set the stop signal, wake the loop so it sees it.
        """
        self._stop_event.set()
        self._intent_event.set()

    def forward(
        self, target_location: float | None = None, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[dict[str, OperationResult]]:
        """Move all groups forward, synchronized through the start barrier.

        Posts an intent to :meth:`control` and returns immediately, the way the
        single cursor's ``forward`` feeds its ``cntrl``. Starting a new operation
        cancels the previous one — a pending intent that never ran included.

        Returns:
            Future resolving with the per-group results when every group stops
            (at the target, the end of the trajectory, or a pause). Cancelling
            the future does not disarm groups already waiting on the trigger —
            use :meth:`pause` for that.
        """
        future: asyncio.Future[dict[str, OperationResult]] = asyncio.Future()
        if self._stop_event.is_set():
            future.set_exception(RuntimeError("The execution session has already been closed"))
            return future
        superseded = self._pending_intent
        if superseded is not None and not superseded.future.done():
            superseded.future.cancel()
        self._pending_intent = _ForwardIntent(future, target_location, playback_speed_in_percent)
        self._intent_event.set()
        return future

    def forward_to(
        self, location: float, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[dict[str, OperationResult]]:
        """Move all groups forward to ``location`` — meaningful on every path
        because of the shared parameterization."""
        if location < self.current_location:
            future: asyncio.Future[dict[str, OperationResult]] = asyncio.Future()
            future.set_exception(
                ValueError("Cannot move forward to a location before the current location")
            )
            return future
        return self.forward(
            target_location=location, playback_speed_in_percent=playback_speed_in_percent
        )

    async def _run_barrier(self, intent: _ForwardIntent) -> None:
        """Run one barrier and resolve the caller's future with its outcome.

        A cancel (pause or a superseding forward) cancels the future. A real
        failure is left off the future: it propagates to abort the session, and
        the awaiter learns the cause from the ``attach`` exit instead.
        """
        try:
            results = await self._forward(intent.target_location, intent.playback_speed_in_percent)
        except asyncio.CancelledError:
            if not intent.future.done():
                intent.future.cancel()
            raise
        # A real failure is deliberately not caught here: setting it on the
        # future too would surface the same error twice in the session group.
        if not intent.future.done():
            intent.future.set_result(results)

    async def _forward(
        self, target_location: float | None, playback_speed_in_percent: int | None
    ) -> dict[str, OperationResult]:
        await self._sync.clear()

        # Live subscriptions taken before arming so the TrajectoryWaitForIO
        # transition cannot be missed, and future-only so a repeated start never
        # matches a stale WaitForIO buffered from an earlier barrier.
        subscriptions = {
            name: broadcaster.subscribe() for name, broadcaster in self._broadcasters.items()
        }
        start_conditions = self._sync.start_conditions()
        futures: dict[str, asyncio.Future[OperationResult]] = {}
        try:
            futures = {
                name: cursor.forward(
                    target_location=target_location,
                    playback_speed_in_percent=playback_speed_in_percent,
                    start_on_io=start_conditions[name],
                )
                for name, cursor in self._cursors.items()
            }
            await asyncio.gather(
                *(_wait_until_waiting_for_io(states) for states in subscriptions.values())
            )
            await self._sync.release()
        except BaseException:
            for future in futures.values():
                if not future.done():
                    future.cancel()
                elif not future.cancelled():
                    # Nobody awaits a superseded operation's future; reading its
                    # exception here keeps asyncio from logging it as never
                    # retrieved when the future is garbage-collected.
                    future.exception()
            raise
        finally:
            for states in subscriptions.values():
                # aclose() raises when the generator is still suspended in a
                # sibling of a failed gather; that must not mask the real error.
                with contextlib.suppress(RuntimeError):
                    await states.aclose()

        results = await asyncio.gather(*futures.values())
        return dict(zip(futures.keys(), results))

    def pause(self) -> asyncio.Future[dict[str, OperationResult]] | None:
        """Pause every group. Returns None when no group had an active operation."""
        # A forward posted but not yet consumed by control() must not run after
        # the pause — void the mailbox first.
        pending, self._pending_intent = self._pending_intent, None
        if pending is not None and not pending.future.done():
            pending.future.cancel()
        futures = {
            name: future
            for name, future in ((name, cursor.pause()) for name, cursor in self._cursors.items())
            if future is not None
        }
        # A barrier still waiting for its groups has no operation left to
        # complete once the cursors pause; cancel it so it cannot linger and
        # release a later barrier's trigger. Cancelled only after the cursors
        # paused: cancelling first would cascade into the cursors' operation
        # futures and make pause() a no-op.
        if self._current is not None and not self._current.done():
            self._current.cancel()
        if not futures:
            return None
        return asyncio.ensure_future(_gather_named(futures))

    @property
    def current_location(self) -> float:
        """Location of the first motion group; with the shared parameterization
        all groups are at (drift-bounded) the same location."""
        return next(iter(self._cursors.values())).current_location

    def stream_state(
        self, group: str | None = None
    ) -> AsyncGenerator[api.models.MotionGroupState, None]:
        """Live motion-group states emitted from now on (first group by default)."""
        if group is None:
            group = next(iter(self._broadcasters))
        return self._broadcasters[group].subscribe()
