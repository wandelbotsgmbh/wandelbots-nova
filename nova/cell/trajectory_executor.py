"""Synchronized execution of one trajectory across multiple motion groups.

Each motion group keeps its own :class:`TrajectoryCursor` and
``executeTrajectory`` websocket; the executor owns what spans them — task
supervision, the start barrier and the session monitors.

Synchronization rests on one invariant, carried structurally by the trajectory
type: all groups of an :class:`api.models.MultiJointTrajectory` share a single
``times`` and a single ``locations`` array, so equal location means the same
instant on every path. The multi-motion-group RRT endpoint produces this type
directly; externally planned trajectories construct it explicitly.

Movement starts run a barrier so the groups start on their controllers' own IO
image rather than on N network round-trips:

1. the sync IO is cleared,
2. every cursor is armed with ``forward(start_on_io=...)``,
3. the executor waits until *every* group reports ``TrajectoryWaitForIO``
   (on a live, future-only subscription, so a stale state from an earlier
   start can never satisfy the barrier),
4. the sync IO is set.

Example:
    ```python
    executor = (
        TrajectoryExecutor.builder({"0@robot": robot_mg, "0@positioner": positioner_mg})
        .sync_on_io("sync_out", controller="robot")
        .build()
    )
    await executor.execute(multi_joint_trajectory)

    # Or interactively:
    async with executor.attach(multi_joint_trajectory) as cursor:
        operation = cursor.forward()
        cursor.pause()
        await cursor.forward()  # resumes through the barrier again
    ```
"""

import asyncio
from collections.abc import AsyncGenerator, AsyncIterable, Iterable, Mapping, Sequence
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Self

from nova import api
from nova.actions.container import located_writes, write_to_set_io
from nova.actions.io import WriteAction, io_write
from nova.cell.motion_group import MotionGroup
from nova.cell.movement_controller.trajectory_cursor import MovementOption, TrajectoryCursor
from nova.cell.multi_trajectory_cursor import (
    IOSyncConfig,
    IOSyncDriver,
    MultiTrajectoryCursor,
    SyncDriver,
    _StreamBroadcaster,
)
from nova.cell.robot_cell import ActionsLike, _normalize_actions
from nova.cell.session_monitor import SessionMonitor, SyncDriftMonitor


@dataclass(frozen=True)
class GroupArgs:
    """Per-group arguments for one execution.

    Actions are not part of this: the ensemble action list is one shared list,
    not per group, and is passed straight to :meth:`TrajectoryExecutor.execute`
    /:meth:`~TrajectoryExecutor.attach`. The executor routes each write to the
    owning group and anchors it on the shared ``locations`` parameterization,
    so no per-group action metadata is needed here.

    Attributes:
        tcp: TCP the trajectory was planned for; passed to trajectory loading.
        ignore_controller_limits: Skip the executing side's limit check when
            initializing the movement, on by default.

            The executing side (RAE) rescales a trajectory it finds to be over
            the controller's limits — including trajectories that are within
            them, which is a defect on that side. Either way it rescales each
            motion group on its own, so the groups stop sharing one time
            parameterization and equal location no longer means the same
            instant: exactly the invariant synchronization rests on. The
            trajectory reaching the executor was planned against the same
            limits, so skipping the check costs nothing here and is the only
            way to keep it as planned. Turn it off for a group whose trajectory
            was not planned by Nova.
    """

    tcp: str | None = None
    ignore_controller_limits: bool = True


async def _detach_when_finished(
    cursor: TrajectoryCursor, states: AsyncIterable[api.models.MotionGroupState]
) -> None:
    """Detach the cursor at the end of its trajectory.

    ``detach_on_standstill`` cannot be used for this: the controller reports
    ``TrajectoryEnded`` at *every* commanded stop, an intermediate
    ``forward_to`` target included, so it would end the session on the first
    interactive step. Only the location says whether the trajectory is through.
    """
    async for state in states:
        if (
            MovementOption.CAN_MOVE_FORWARD not in cursor.get_movement_options()
            and state.standstill
        ):
            cursor.detach()
            return


class TrajectoryExecutor:
    """Long-lived owner of synchronized multi-motion-group execution.

    Created once per topology — which motion groups, how they synchronize — and
    holds no per-run state; trajectories are per-call arguments and each run
    gets its own session (:meth:`attach` / :meth:`execute`).

    Every execution goes through the barrier, so a single group with a
    hardware-gated start is a valid topology; plain single-group execution is
    :meth:`MotionGroup.execute`'s job. Groups need not share a cell or gateway
    — each is executed through its own — but a :class:`SyncDriver` may require
    it (:class:`IOSyncDriver` writes within one cell).

    ``monitors`` observe every session's state streams; one that raises aborts
    the session.
    """

    def __init__(
        self,
        motion_groups: Mapping[str, MotionGroup],
        sync: SyncDriver,
        monitors: Sequence[SessionMonitor] = (),
    ):
        if not motion_groups:
            raise ValueError("At least one motion group is required")
        groups_without_condition = set(motion_groups) - set(sync.start_conditions())
        if groups_without_condition:
            raise ValueError(
                f"The sync driver has no start condition for groups: "
                f"{sorted(groups_without_condition)}"
            )

        self._motion_groups = dict(motion_groups)
        self._monitors = tuple(monitors)
        self._sync = sync

    @property
    def motion_groups(self) -> dict[str, MotionGroup]:
        """The motion groups this executor drives, keyed by name."""
        return dict(self._motion_groups)

    async def execute(
        self,
        trajectory: api.models.MultiJointTrajectory,
        actions: ActionsLike | None = None,
        groups: Mapping[str, GroupArgs] | None = None,
    ) -> None:
        """Execute the trajectory front to end, synchronized through the barrier.

        ``actions`` is the same ensemble action list passed to
        :meth:`MultiMotionGroupPlanner.plan`; its :class:`WriteAction` entries
        become the location-anchored IO overlay fired during execution (see
        :meth:`attach`)."""
        async with self.attach(trajectory, actions=actions, groups=groups) as cursor:
            await cursor.forward()

    @asynccontextmanager
    async def attach(
        self,
        trajectory: api.models.MultiJointTrajectory,
        actions: ActionsLike | None = None,
        groups: Mapping[str, GroupArgs] | None = None,
    ) -> AsyncGenerator[MultiTrajectoryCursor, None]:
        """Open a session for interactive control: load the trajectory, open the
        sockets, initialize — but do not move. Starting is the caller's call
        (:meth:`MultiTrajectoryCursor.forward`); exiting the context detaches
        every cursor and closes every socket.

        ``actions`` mirrors the list given to
        :meth:`MultiMotionGroupPlanner.plan`. It is not baked into the loaded
        trajectory: each :class:`WriteAction` is turned into a
        :class:`api.models.SetIO` anchored at its motion index (the count of
        motions before it, waits skipped — the same convention as single-group
        execution) and attached to the owning group's cursor, which re-emits the
        overlay on every start. Every write is fired once, by the single group
        it routes to; since all groups share one ``locations`` array that instant
        is synchronized across the ensemble."""
        per_group = self._split_for_groups(trajectory)
        unknown_group_args = set(groups or {}) - set(self._motion_groups)
        if unknown_group_args:
            raise ValueError(f"Group args given for unknown groups: {sorted(unknown_group_args)}")

        overlay = self._build_io_overlay(actions)

        cursors: dict[str, TrajectoryCursor] = {}
        broadcasters: dict[str, _StreamBroadcaster[api.models.MotionGroupState]] = {}
        for name, joint_trajectory in per_group.items():
            motion_group = self._motion_groups[name]
            group_args = (groups or {}).get(name) or GroupArgs()
            trajectory_id = await motion_group._load_planned_motion(
                joint_trajectory, group_args.tcp
            )
            cursor = TrajectoryCursor(
                motion_id=trajectory_id,
                motion_group_state_stream=motion_group.stream_state,
                joint_trajectory=joint_trajectory,
                detach_on_standstill=False,
                emit_motion_events=False,
                ignore_controller_limits=group_args.ignore_controller_limits,
                set_outputs=overlay[name] or None,
            )
            cursors[name] = cursor
            broadcasters[name] = _StreamBroadcaster(cursor)

        try:
            async with asyncio.TaskGroup() as task_group:
                for name, cursor in cursors.items():
                    task_group.create_task(
                        broadcasters[name].run(), name=f"state-broadcaster-{name}"
                    )
                    task_group.create_task(
                        _detach_when_finished(cursor, broadcasters[name].subscribe()),
                        name=f"end-monitor-{name}",
                    )
                    task_group.create_task(
                        self._run_execute_trajectory(name, cursor),
                        name=f"execute-trajectory-{name}",
                    )
                for monitor in self._monitors:
                    task_group.create_task(
                        monitor.run(
                            {
                                name: broadcaster.subscribe()
                                for name, broadcaster in broadcasters.items()
                            }
                        ),
                        name=f"session-monitor-{type(monitor).__name__}",
                    )
                try:
                    yield MultiTrajectoryCursor(cursors, broadcasters, self._sync, task_group)
                finally:
                    for cursor in cursors.values():
                        cursor.detach()
        except BaseExceptionGroup as error_group:
            # Callers expect the underlying error (e.g. SyncDriftError), not the
            # TaskGroup wrapper — same unwrapping the cursor itself applies.
            if len(error_group.exceptions) == 1 and isinstance(
                error_group.exceptions[0], Exception
            ):
                raise error_group.exceptions[0] from error_group
            raise

    def _split_for_groups(
        self, trajectory: api.models.MultiJointTrajectory
    ) -> dict[str, api.models.JointTrajectory]:
        """Split into per-group ``JointTrajectory`` for loading; every part
        carries the trajectory's single ``times``/``locations`` parameterization."""
        joint_positions_by_group = trajectory.joint_positions_by_motion_group_key.root
        if set(joint_positions_by_group) != set(self._motion_groups):
            raise ValueError(
                f"Trajectory keys {sorted(joint_positions_by_group)} must match the executor's "
                f"motion groups {sorted(self._motion_groups)}"
            )
        return {
            key: api.models.JointTrajectory(
                joint_positions=joint_samples.root,
                times=trajectory.times,
                locations=trajectory.locations,
            )
            for key, joint_samples in joint_positions_by_group.items()
        }

    def _build_io_overlay(self, actions: ActionsLike | None) -> dict[str, list[api.models.SetIO]]:
        """Turn the action list's writes into a per-group ``SetIO`` overlay.

        Each write is routed to exactly one group and anchored at its motion
        index; all other groups get an empty list."""
        overlay: dict[str, list[api.models.SetIO]] = {name: [] for name in self._motion_groups}
        for motion_index, write in located_writes(_normalize_actions(actions or [])):
            group = self._route_write(write)
            overlay[group].append(write_to_set_io(write, motion_index))
        return overlay

    def _route_write(self, write: WriteAction) -> str:
        """Pick the single group whose execute stream fires this write.

        A controller output goes to the group on that controller; a cell-wide
        bus variable goes to one deterministic group (all groups share the
        ``locations`` array, so which one carries it does not change the instant).
        """
        if write.origin is api.models.IOOrigin.BUS_IO:
            return min(self._motion_groups)

        controllers = {
            name: motion_group._controller_id for name, motion_group in self._motion_groups.items()
        }
        if write.device_id is None:
            distinct = set(controllers.values())
            if len(distinct) > 1:
                raise ValueError(
                    f"The controller for IO '{write.key}' is ambiguous: the motion groups span "
                    f"controllers {sorted(distinct)}. Set the write's device_id explicitly."
                )
            return min(self._motion_groups)

        matching = sorted(name for name, ctrl in controllers.items() if ctrl == write.device_id)
        if not matching:
            raise ValueError(
                f"No motion group is on controller '{write.device_id}' for IO '{write.key}'; "
                f"known controllers are {sorted(set(controllers.values()))}."
            )
        return matching[0]

    async def _run_execute_trajectory(self, name: str, cursor: TrajectoryCursor) -> None:
        # Addressed through the group's own cell and gateway: executing does not
        # assume the groups share either, only the IO barrier does.
        motion_group = self._motion_groups[name]
        await motion_group._api_client.trajectory_execution_api.execute_trajectory(
            cell=motion_group._cell,
            controller=motion_group._controller_id,
            client_request_generator=cursor.cntrl,  # ty: ignore[invalid-argument-type]
        )

    @classmethod
    def builder(
        cls, motion_groups: Mapping[str, MotionGroup] | Iterable[MotionGroup]
    ) -> "TrajectoryExecutorBuilder":
        """Start fluent assembly: ``TrajectoryExecutor.builder(...)...build()``.

        ``motion_groups`` is either a mapping of names to motion groups, or
        just the motion groups — then their ids are the names.
        """
        return TrajectoryExecutorBuilder(motion_groups)


class TrajectoryExecutorBuilder:
    """Fluent assembly of a :class:`TrajectoryExecutor`.

    The basic scenario is two calls — the motion groups and the sync IO:

    ```python
    executor = (
        TrajectoryExecutor.builder([robot_mg, positioner_mg])
        .sync_on_io("sync_out", controller="robot")
        .build()
    )
    ```

    :meth:`sync_on_io` states all three pieces of the barrier — the release
    write, the clear write and every group's start condition — so nothing is
    inferred later. :meth:`release_io`, :meth:`clear_io` and :meth:`watch`
    state the same pieces one at a time, for topologies where they are not one
    boolean IO; called after :meth:`sync_on_io` they replace what it set. A
    drift monitor runs by default — :meth:`monitors` replaces it. A custom
    :class:`SyncDriver` goes through the plain constructor instead.
    """

    def __init__(self, motion_groups: Mapping[str, MotionGroup] | Iterable[MotionGroup]):
        self._motion_groups = (
            dict(motion_groups)
            if isinstance(motion_groups, Mapping)
            else {motion_group.id: motion_group for motion_group in motion_groups}
        )
        if not self._motion_groups:
            raise ValueError("At least one motion group is required")
        self._release: WriteAction | None = None
        self._clear: WriteAction | None = None
        self._watch: dict[str, api.models.StartOnIO] = {}
        self._monitors: tuple[SessionMonitor, ...] | None = None

    def sync_on_io(
        self,
        io: str,
        controller: str | None = None,
        *,
        origin: api.models.IOOrigin = api.models.IOOrigin.CONTROLLER,
        release_value: bool = True,
    ) -> Self:
        """Synchronize on one boolean IO: releasing the barrier writes
        ``release_value`` to it, clearing writes the other value, and every
        group watches it for ``release_value``.

        ``controller`` may be omitted when all motion groups share one; pass
        ``origin=IOOrigin.BUS_IO`` for a cell-wide bus variable.
        """
        self._release = self._write(io, release_value, controller, origin)
        self._clear = self._write(io, not release_value, controller, origin)
        condition = api.models.StartOnIO(
            io=api.models.IOValue(api.models.IOBooleanValue(io=io, value=release_value)),
            comparator=api.models.Comparator.COMPARATOR_EQUALS,
            io_origin=origin,
        )
        self._watch = {group: condition for group in self._motion_groups}
        return self

    def release_io(
        self,
        io: str,
        value: bool,
        controller: str | None = None,
        *,
        origin: api.models.IOOrigin = api.models.IOOrigin.CONTROLLER,
    ) -> Self:
        """Release the barrier with this write — the counterpart of
        :meth:`clear_io` when the two are not one IO's two values."""
        self._release = self._write(io, value, controller, origin)
        return self

    def clear_io(
        self,
        io: str,
        value: bool,
        controller: str | None = None,
        *,
        origin: api.models.IOOrigin = api.models.IOOrigin.CONTROLLER,
    ) -> Self:
        """Clear the barrier with this write — the counterpart of
        :meth:`release_io`."""
        self._clear = self._write(io, value, controller, origin)
        return self

    def watch(self, group: str, condition: api.models.StartOnIO) -> Self:
        """Gate one group's start on this condition instead of on the release
        write itself (e.g. the wired input the sync IO physically lands on)."""
        if group not in self._motion_groups:
            raise ValueError(
                f"Unknown motion group '{group}'; known groups are {sorted(self._motion_groups)}"
            )
        self._watch[group] = condition
        return self

    def monitors(self, *monitors: SessionMonitor) -> Self:
        """Replace the default session monitors (a :class:`SyncDriftMonitor`);
        call with no arguments to run without any."""
        self._monitors = monitors
        return self

    def build(self) -> TrajectoryExecutor:
        if self._release is None:
            raise ValueError("The barrier has no release write: call sync_on_io() or release_io()")
        if self._clear is None:
            raise ValueError("The barrier has no clear write: call sync_on_io() or clear_io()")
        config = IOSyncConfig(clear=self._clear, release=self._release, watch=self._watch)
        barrier_group = self._barrier_group()
        return TrajectoryExecutor(
            self._motion_groups,
            IOSyncDriver(config, barrier_group._api_client, barrier_group._cell),
            self._monitors if self._monitors is not None else (SyncDriftMonitor(),),
        )

    def _barrier_group(self) -> MotionGroup:
        """The motion group whose cell and gateway carry the IO barrier.

        The IO barrier writes into a single cell — bus IO is cell-scoped, and a
        controller output is addressed as (cell, controller) — so groups spanning
        cells need a driver built for the cell that carries the sync IO.
        """
        cells = {motion_group._cell for motion_group in self._motion_groups.values()}
        if len(cells) > 1:
            raise ValueError(
                f"The IO barrier writes into one cell but the motion groups span {sorted(cells)}. "
                "Build an IOSyncDriver for the cell carrying the sync IO and pass it to "
                "TrajectoryExecutor directly."
            )
        return next(iter(self._motion_groups.values()))

    def _write(
        self, io: str, value: bool, controller: str | None, origin: api.models.IOOrigin
    ) -> WriteAction:
        """Build a sync write, resolving its controller when it is unambiguous."""
        if origin is not api.models.IOOrigin.BUS_IO and controller is None:
            controllers = {
                motion_group._controller_id for motion_group in self._motion_groups.values()
            }
            if len(controllers) > 1:
                raise ValueError(
                    f"The controller for sync IO '{io}' is ambiguous: the motion groups span "
                    f"controllers {sorted(controllers)}. Pass the controller explicitly."
                )
            controller = next(iter(controllers))
        return io_write(io, value, device_id=controller, origin=origin)
