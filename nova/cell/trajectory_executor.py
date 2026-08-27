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
from collections.abc import AsyncGenerator, Iterable, Mapping, Sequence
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Self

from nova import api
from nova.actions.io import WriteAction, io_write
from nova.cell.motion_group import MotionGroup
from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor
from nova.cell.multi_trajectory_cursor import IOSyncDriver, MultiTrajectoryCursor, SyncDriver
from nova.cell.session_monitor import SessionMonitor, SyncDriftMonitor


@dataclass(frozen=True)
class GroupArgs:
    """Per-group arguments for one execution.

    Actions are not part of this: an action list belongs to one motion group's
    trajectory, whose integer locations are *its* action boundaries, and the
    ensemble has one shared parameterization instead. Synchronized execution
    therefore carries no action metadata and no IO overlay derived from it.

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

    async def execute(
        self,
        trajectory: api.models.MultiJointTrajectory,
        groups: Mapping[str, GroupArgs] | None = None,
    ) -> None:
        """Execute the trajectory front to end, synchronized through the barrier."""
        async with self.attach(trajectory, groups) as cursor:
            await cursor.forward()

    @asynccontextmanager
    async def attach(
        self,
        trajectory: api.models.MultiJointTrajectory,
        groups: Mapping[str, GroupArgs] | None = None,
    ) -> AsyncGenerator[MultiTrajectoryCursor, None]:
        """Open a session for interactive control: load the trajectory, open the
        sockets, initialize — but do not move. Starting is the caller's call
        (:meth:`MultiTrajectoryCursor.forward`); exiting the context detaches
        every cursor and closes every socket."""
        per_group = self._split_for_groups(trajectory)
        unknown_group_args = set(groups or {}) - set(self._motion_groups)
        if unknown_group_args:
            raise ValueError(f"Group args given for unknown groups: {sorted(unknown_group_args)}")

        cursors: dict[str, TrajectoryCursor] = {}
        for name, joint_trajectory in per_group.items():
            motion_group = self._motion_groups[name]
            group_args = (groups or {}).get(name) or GroupArgs()
            trajectory_id = await motion_group._load_planned_motion(
                joint_trajectory, group_args.tcp
            )
            cursors[name] = TrajectoryCursor(
                motion_id=trajectory_id,
                motion_group_state_stream=motion_group.stream_state,
                joint_trajectory=joint_trajectory,
                detach_on_standstill=False,
                emit_motion_events=False,
                ignore_controller_limits=group_args.ignore_controller_limits,
            )

        cursor = MultiTrajectoryCursor(cursors, self._sync)
        try:
            async with asyncio.TaskGroup() as task_group:
                # control() owns the barrier and the state fan-out in its own
                # TaskGroup; the executor only adds what needs its motion groups —
                # the executeTrajectory sockets — and the session monitors.
                for name in cursors:
                    task_group.create_task(
                        self._run_execute_trajectory(name, cursors[name]),
                        name=f"execute-trajectory-{name}",
                    )
                for monitor in self._monitors:
                    task_group.create_task(
                        monitor.run(cursor.monitor_streams()),
                        name=f"session-monitor-{type(monitor).__name__}",
                    )
                task_group.create_task(cursor.control(), name="multi-trajectory-cursor-control")
                try:
                    yield cursor
                finally:
                    cursor.request_stop()
                    for group_cursor in cursors.values():
                        group_cursor.detach()
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
        joint_positions_by_group = trajectory.joint_positions_by_motion_group_key
        if set(joint_positions_by_group) != set(self._motion_groups):
            raise ValueError(
                f"Trajectory keys {sorted(joint_positions_by_group)} must match the executor's "
                f"motion groups {sorted(self._motion_groups)}"
            )
        return {
            key: api.models.JointTrajectory(
                joint_positions=joint_samples,
                times=trajectory.times,
                locations=trajectory.locations,
            )
            for key, joint_samples in joint_positions_by_group.items()
        }

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
            io=api.models.IOBooleanValue(io=io, value=release_value),
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
        barrier_group = self._barrier_group()
        return TrajectoryExecutor(
            self._motion_groups,
            IOSyncDriver(
                clear=self._clear,
                release=self._release,
                watch=self._watch,
                api_client=barrier_group._api_client,
                cell=barrier_group._cell,
            ),
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
