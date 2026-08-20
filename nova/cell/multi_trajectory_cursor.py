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


@dataclass(frozen=True)
class IOSyncConfig:
    """IO-based start synchronization.

    Three independent pieces describe the barrier for any topology: the IO
    write that puts the cell into the not-released state (``clear``), the IO
    write that releases it (``release``), and the condition each group's start
    *watches* (``watch``). They are deliberately not derived from one another —
    clear and release may target different values, IOs, or even controllers,
    and only the wiring's owner knows which.

    Topologies covered:

    - **Same controller / cell-wide bus IO**: one boolean trigger; every group
      watches it, clearing writes its inverse — derived by the builder's
      :meth:`TrajectoryExecutorBuilder.sync_on_io`.
    - **Physically wired controllers**: the writes go to one controller's
      output; each group watches its own input the wire lands on. Spelled out
      explicitly — the executor cannot know how a cell is wired.

    For ``CONTROLLER``-origin writes, ``device_id`` names the controller
    owning the IO; it may be omitted when all motion groups share one
    controller. It is ignored for ``BUS_IO`` origin.

    Attributes:
        clear: Written before the groups are armed, so no stale release value
            can start them early.
        release: Written once every group waits on its start condition.
        watch: The start condition per group, keyed like the executor's motion
            groups; every group needs one.
    """

    clear: WriteAction
    release: WriteAction
    watch: Mapping[str, api.models.StartOnIO]


class IOSyncDriver:
    """IO-based :class:`SyncDriver`: runs :class:`IOSyncConfig`'s writes and
    serves its per-group start conditions."""

    def __init__(self, config: IOSyncConfig, api_client: ApiGateway, cell: str):
        for action in (config.clear, config.release):
            if action.origin is not api.models.IOOrigin.BUS_IO and action.device_id is None:
                raise ValueError(
                    f"Sync IO write '{action.key}' needs a device_id naming its controller"
                )
        self._config = config
        self._api_client = api_client
        self._cell = cell

    def start_conditions(self) -> Mapping[str, api.models.StartOnIO]:
        return self._config.watch

    async def clear(self) -> None:
        await self._write(self._config.clear)

    async def release(self) -> None:
        await self._write(self._config.release)

    async def _write(self, action: WriteAction) -> None:
        """Write the IO and wait until the write is observable.

        Without the confirmation a group could arm its start condition while the
        IO image still carries the previous barrier's value and start immediately,
        desynchronizing the run.
        """
        io_value = api.models.IOValue(action.to_api_model())
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
    consumer; the session needs several (barrier, end monitor, drift monitor,
    user streams). A subscriber that closes its stream stops being fed; a
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


class MultiTrajectoryCursor:
    """Cursor-compatible handle over one synchronized execution session.

    Obtained from :meth:`TrajectoryExecutor.attach`; never free-standing. Holds
    the per-run state — the per-group cursors, their state broadcasters and the
    barrier — and cannot outlive the ``attach`` context that supervises it.

    Backward movement is not exposed: driving the barrier in reverse is
    mechanically symmetric but unverified against real controllers.
    """

    def __init__(
        self,
        cursors: dict[str, TrajectoryCursor],
        broadcasters: dict[str, _StreamBroadcaster[api.models.MotionGroupState]],
        sync: SyncDriver,
        task_group: asyncio.TaskGroup,
    ):
        self._cursors = cursors
        self._broadcasters = broadcasters
        self._sync = sync
        # The barrier's failure coupling, in both directions: a forward task
        # spawned here failing aborts the whole session (sockets close, the
        # groups stop), and a dying session cancels a barrier still waiting for
        # its groups. Once ``attach`` exits, the finished TaskGroup rejects new
        # tasks, so a kept handle fails fast.
        self._task_group = task_group
        self._forward_task: asyncio.Task[dict[str, OperationResult]] | None = None

    def forward(
        self, target_location: float | None = None, playback_speed_in_percent: int | None = None
    ) -> asyncio.Future[dict[str, OperationResult]]:
        """Move all groups forward, synchronized through the start barrier.

        Starting a new operation cancels the previous one, as on the single
        cursor.

        Returns:
            Future resolving with the per-group results when every group stops
            (at the target, the end of the trajectory, or a pause). Cancelling
            the future does not disarm groups already waiting on the trigger —
            use :meth:`pause` for that.
        """
        previous = self._forward_task
        try:
            self._forward_task = self._task_group.create_task(
                self._forward(previous, target_location, playback_speed_in_percent),
                name="multi-trajectory-cursor-forward",
            )
        except RuntimeError:
            # A finished TaskGroup rejects new tasks — the session is closed.
            future: asyncio.Future[dict[str, OperationResult]] = asyncio.Future()
            future.set_exception(RuntimeError("The execution session has already been closed"))
            return future
        return self._forward_task

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

    async def _forward(
        self,
        previous: "asyncio.Task[dict[str, OperationResult]] | None",
        target_location: float | None,
        playback_speed_in_percent: int | None,
    ) -> dict[str, OperationResult]:
        # A superseded barrier must be gone before a new one arms the groups:
        # left running, its still-open subscriptions would be satisfied by the
        # *new* barrier's WaitForIO states and fire the trigger a second time.
        await _cancel_and_wait(previous)

        await self._sync.clear()

        # Live subscriptions taken before forward() so the TrajectoryWaitForIO
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
        if self._forward_task is not None and not self._forward_task.done():
            self._forward_task.cancel()
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
