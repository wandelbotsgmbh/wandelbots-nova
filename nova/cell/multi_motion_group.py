"""One synchronized ensemble of motion groups, planned and executed as a unit.

:class:`MultiMotionGroup` is to several motion groups what :class:`MotionGroup`
is to one: a single object exposing ``plan`` / ``execute`` / ``plan_and_execute``.
It is a thin facade over the two halves that already do the work —
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
    ```

The ensemble is cell-scoped because planning is: the multi-motion-group RRT
endpoint reasons about the groups' mutual geometry in a single, cell-scoped
request. Execution itself may span cells (only the IO barrier is cell-bound),
so the planner is built lazily — an execute-only ensemble of a pre-planned
trajectory never triggers the single-cell requirement.
"""

from collections.abc import AsyncGenerator, Iterable, Mapping
from contextlib import asynccontextmanager

from nova import api
from nova.cell.motion_group import MotionGroup
from nova.cell.multi_motion_group_planner import MultiMotionGroupPlanner
from nova.cell.multi_trajectory_cursor import MultiTrajectoryCursor
from nova.cell.robot_cell import ActionsLike
from nova.cell.trajectory_executor import GroupArgs, TrajectoryExecutor, TrajectoryExecutorBuilder


class MultiMotionGroup:
    """A synchronized ensemble of motion groups planned and executed as a unit.

    Built with :meth:`builder`, which states the sync barrier the same way
    :class:`TrajectoryExecutor` does. ``plan`` and ``execute`` take the same
    ensemble action list; ``plan_and_execute`` chains them, mirroring
    :meth:`MotionGroup.plan_and_execute`.
    """

    def __init__(self, executor: TrajectoryExecutor):
        self._executor = executor
        # Planning is cell-scoped, execution need not be — so the planner is
        # only constructed when a plan is actually requested.
        self._planner: MultiMotionGroupPlanner | None = None

    @property
    def motion_groups(self) -> dict[str, MotionGroup]:
        """The motion groups in this ensemble, keyed by name."""
        return self._executor.motion_groups

    def _get_planner(self) -> MultiMotionGroupPlanner:
        if self._planner is None:
            self._planner = MultiMotionGroupPlanner(self._executor.motion_groups)
        return self._planner

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
        return await self._get_planner().plan(actions, tcp, start_joint_position)

    async def execute(
        self,
        trajectory: api.models.MultiJointTrajectory,
        actions: ActionsLike | None = None,
        groups: Mapping[str, GroupArgs] | None = None,
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
        actions: ActionsLike | None = None,
        groups: Mapping[str, GroupArgs] | None = None,
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
        multi-group counterpart of :meth:`MotionGroup.plan_and_execute`."""
        trajectory = await self.plan(actions, tcp, start_joint_position)
        await self.execute(trajectory, actions=actions, groups=groups)

    @classmethod
    def builder(
        cls, motion_groups: Mapping[str, MotionGroup] | Iterable[MotionGroup]
    ) -> "MultiMotionGroupBuilder":
        """Start fluent assembly: ``MultiMotionGroup.builder(...)...build()``.

        Takes the motion groups the same way :meth:`TrajectoryExecutor.builder`
        does — a mapping of names, or just the groups keyed by their ids.
        """
        return MultiMotionGroupBuilder(motion_groups)


class MultiMotionGroupBuilder(TrajectoryExecutorBuilder):
    """Fluent assembly of a :class:`MultiMotionGroup`.

    Identical to :class:`TrajectoryExecutorBuilder` — same ``sync_on_io`` /
    ``release_io`` / ``clear_io`` / ``watch`` / ``monitors`` — but :meth:`build`
    returns a :class:`MultiMotionGroup` wrapping the executor it assembles.
    """

    # Narrows the return type past the base builder's TrajectoryExecutor: a
    # MultiMotionGroupBuilder is only ever used as itself, never through a
    # TrajectoryExecutorBuilder reference expecting build() -> TrajectoryExecutor.
    def build(self) -> MultiMotionGroup:  # ty: ignore[invalid-method-override]
        return MultiMotionGroup(super().build())
