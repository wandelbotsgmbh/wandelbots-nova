"""One synchronized ensemble of motion groups, planned and executed as a unit.

:class:`MultiMotionGroup` is to several motion groups what :class:`MotionGroup`
is to one: a single object exposing ``plan`` / ``execute`` / ``plan_and_execute``.
It composes the two halves that do the work —
:class:`~nova.cell.multi_motion_group_planner.MultiMotionGroupPlanner` for
planning and :class:`~nova.cell.trajectory_executor.TrajectoryExecutor` for
synchronized execution — sharing one action list across both, exactly as
:meth:`MotionGroup.plan_and_execute` passes ``actions`` to both its own halves.

    ```python
    ensemble = (
        MultiMotionGroup.builder({"0@robot": robot_mg, "0@positioner": positioner_mg})
        .sync_on_io("sync_out", controller="robot")
        .build()
    )
    await ensemble.plan_and_execute(
        [
            multi_collision_free({"0@robot": Pose(...), "0@positioner": (j1, ..., j6)}),
            io_write("OUT#1", True, device_id="robot"),
        ],
        tcp={"0@robot": "flange", "0@positioner": None},
    )

    # Execute-only, mirroring MotionGroup.execute — the planner stays dormant:
    await ensemble.execute(recorded_trajectory)
    ```

The ensemble is cell-scoped: the multi-motion-group RRT endpoint reasons about
the groups' mutual geometry in one cell-scoped request, and the sync barrier
gates them on one cell's IO.
"""

from collections.abc import AsyncGenerator, Iterable, Mapping
from contextlib import asynccontextmanager
from typing import Self

from nova import api
from nova.actions.io import WriteAction, io_write
from nova.actions.mock import WaitAction
from nova.cell.motion_group import MotionGroup
from nova.cell.multi_motion_group_planner import MultiMotionGroupPlanner
from nova.cell.multi_trajectory_cursor import IOSyncDriver, MultiTrajectoryCursor
from nova.cell.robot_cell import ActionsLike, _normalize_actions
from nova.cell.session_monitor import SessionMonitor, SyncDriftMonitor
from nova.cell.trajectory_executor import GroupArgs, TrajectoryExecutor


class MultiMotionGroup:
    """A synchronized ensemble of motion groups planned and executed as a unit.

    Assembled with :meth:`builder`, which states the sync barrier. ``plan`` and
    ``execute`` take the same ensemble action list; ``plan_and_execute`` chains
    them, mirroring :meth:`MotionGroup.plan_and_execute`. A pre-planned
    trajectory can be run with :meth:`execute` alone, leaving the planner idle.

    Wraps two collaborators over the same motion groups — a
    :class:`~nova.cell.multi_motion_group_planner.MultiMotionGroupPlanner` and a
    :class:`~nova.cell.trajectory_executor.TrajectoryExecutor`; :meth:`builder`
    wires them, or pass your own to the constructor.
    """

    def __init__(self, executor: TrajectoryExecutor, planner: MultiMotionGroupPlanner):
        if set(executor.motion_groups) != set(planner.motion_groups):
            raise ValueError(
                "executor and planner must cover the same motion groups: "
                f"{sorted(executor.motion_groups)} vs {sorted(planner.motion_groups)}"
            )
        self._executor = executor
        self._planner = planner

    @property
    def motion_groups(self) -> dict[str, MotionGroup]:
        """The motion groups in this ensemble, keyed by name."""
        return self._executor.motion_groups

    async def plan(
        self,
        actions: ActionsLike,
        tcp: Mapping[str, str | None] | str | None = None,
        start_joint_position: Mapping[str, tuple[float, ...]] | None = None,
    ) -> api.models.MultiJointTrajectory:
        """Plan one synchronized collision-free trajectory for the action list.

        Delegates to :meth:`MultiMotionGroupPlanner.plan`; see it for the action
        types, the per-group ``tcp``/``start_joint_position`` and the raised
        errors. Returns a pure trajectory — :class:`WriteAction` entries carry no
        joint samples and become the IO overlay only at :meth:`execute` time.
        """
        return await self._planner.plan(actions, tcp, start_joint_position)

    async def execute(
        self,
        trajectory: api.models.MultiJointTrajectory,
        groups: Mapping[str, GroupArgs] | None = None,
        actions: ActionsLike | None = None,
    ) -> None:
        """Execute the trajectory front to end, synchronized through the barrier.

        Pass the same ``actions`` list given to :meth:`plan` to fire its
        location-anchored IO. Delegates to :meth:`TrajectoryExecutor.execute`.
        """
        await self._executor.execute(trajectory, actions=actions, groups=groups)

    @asynccontextmanager
    async def attach(
        self,
        trajectory: api.models.MultiJointTrajectory,
        groups: Mapping[str, GroupArgs] | None = None,
        actions: ActionsLike | None = None,
    ) -> AsyncGenerator[MultiTrajectoryCursor, None]:
        """Open an interactive session over the trajectory, delegating to
        :meth:`TrajectoryExecutor.attach`."""
        async with self._executor.attach(trajectory, actions=actions, groups=groups) as cursor:
            yield cursor

    async def plan_and_execute(
        self,
        actions: ActionsLike,
        tcp: Mapping[str, str | None] | str | None = None,
        start_joint_position: Mapping[str, tuple[float, ...]] | None = None,
        groups: Mapping[str, GroupArgs] | None = None,
    ) -> None:
        """Plan and execute in one call, passing ``actions`` to both halves — the
        multi-group counterpart of :meth:`MotionGroup.plan_and_execute`.

        An all-non-motion list (only writes/waits) is applied directly, without
        planning — the same short-circuit :meth:`MotionGroup.plan_and_execute` has.
        """
        actions_list = _normalize_actions(actions)
        if actions_list and all(isinstance(a, (WaitAction, WriteAction)) for a in actions_list):
            await self._executor.apply_non_motion_actions(actions_list)
            return
        trajectory = await self.plan(actions_list, tcp, start_joint_position)
        await self.execute(trajectory, actions=actions_list, groups=groups)

    @classmethod
    def builder(
        cls, motion_groups: Mapping[str, MotionGroup] | Iterable[MotionGroup]
    ) -> "MultiMotionGroupBuilder":
        """Start fluent assembly: ``MultiMotionGroup.builder(...)...build()``.

        ``motion_groups`` is either a mapping of names to motion groups, or just
        the motion groups — then their ids are the names.
        """
        return MultiMotionGroupBuilder(motion_groups)


class MultiMotionGroupBuilder:
    """Fluent assembly of a :class:`MultiMotionGroup`.

    The basic scenario is two calls — the motion groups and the sync IO:

    ```python
    ensemble = (
        MultiMotionGroup.builder([robot_mg, positioner_mg])
        .sync_on_io("sync_out", controller="robot")
        .build()
    )
    ```

    :meth:`sync_on_io` states all three pieces of the barrier — the release
    write, the clear write and every group's start condition — so nothing is
    inferred later. :meth:`release_io`, :meth:`clear_io` and :meth:`watch` state
    the same pieces one at a time, for topologies where they are not one boolean
    IO; called after :meth:`sync_on_io` they replace what it set. A drift monitor
    runs by default — :meth:`monitors` replaces it.
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
        self._watch = {motion_group: condition for motion_group in self._motion_groups}
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

    def watch(self, motion_group: str, condition: api.models.StartOnIO) -> Self:
        """Gate one group's start on this condition instead of on the release
        write itself (e.g. the wired input the sync IO physically lands on)."""
        if motion_group not in self._motion_groups:
            raise ValueError(
                f"Unknown motion group '{motion_group}'; known groups are {sorted(self._motion_groups)}"
            )
        self._watch[motion_group] = condition
        return self

    def monitors(self, *monitors: SessionMonitor) -> Self:
        """Replace the default session monitors (a :class:`SyncDriftMonitor`);
        call with no arguments to run without any."""
        self._monitors = monitors
        return self

    def build(self) -> MultiMotionGroup:
        if self._release is None:
            raise ValueError("The barrier has no release write: call sync_on_io() or release_io()")
        if self._clear is None:
            raise ValueError("The barrier has no clear write: call sync_on_io() or clear_io()")
        # The IO barrier writes into a single cell — bus IO is cell-scoped, a
        # controller output is addressed as (cell, controller) — and synchronized
        # execution is single-cell anyway, so any group's cell and gateway serve.
        barrier_motion_group = next(iter(self._motion_groups.values()))
        executor = TrajectoryExecutor(
            self._motion_groups,
            IOSyncDriver(
                clear=self._clear,
                release=self._release,
                watch=self._watch,
                api_client=barrier_motion_group._api_client,
                cell=barrier_motion_group._cell,
            ),
            self._monitors if self._monitors is not None else (SyncDriftMonitor(),),
        )
        return MultiMotionGroup(executor, MultiMotionGroupPlanner(self._motion_groups))

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
