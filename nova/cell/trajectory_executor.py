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

This is the execution machinery behind :class:`~nova.cell.multi_motion_group.MultiMotionGroup`;
assemble it through :meth:`MultiMotionGroup.builder` (the user-facing entry) or construct it
directly with an :class:`IOSyncDriver`. Then::

    await executor.execute(multi_joint_trajectory)

    # Or interactively:
    async with executor.attach(multi_joint_trajectory) as cursor:
        operation = cursor.forward()
        cursor.pause()
        await cursor.forward()  # resumes through the barrier again
"""

import asyncio
import functools
from collections.abc import AsyncGenerator, Mapping, Sequence
from contextlib import asynccontextmanager
from dataclasses import dataclass

from nova import api
from nova.actions.base import Action
from nova.actions.container import located_writes, write_to_set_io
from nova.actions.io import WriteAction
from nova.actions.mock import WaitAction
from nova.cell.motion_group import MotionGroup
from nova.cell.movement_controller.trajectory_cursor import TrajectoryCursor
from nova.cell.multi_trajectory_cursor import MultiTrajectoryCursor, SyncDriver
from nova.cell.robot_cell import ActionsLike, _normalize_actions
from nova.cell.session_monitor import SessionMonitor


@dataclass(frozen=True)
class GroupArgs:
    """Per-group arguments for one execution.

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
        state_stream_rate_msecs: Rate of this group's motion-group state
            stream, in milliseconds. None means the controller's own step
            rate — the fastest the server emits. The shared stream is opened
            at the rate of its first subscriber; see
            :meth:`MotionGroup.stream_state`.
    """

    tcp: str | None = None
    ignore_controller_limits: bool = True
    state_stream_rate_msecs: int | None = None


class TrajectoryExecutor:
    """Long-lived owner of synchronized multi-motion-group execution.

    Created once per topology — which motion groups, how they synchronize — and
    holds no per-run state; trajectories are per-call arguments and each run
    gets its own session (:meth:`attach` / :meth:`execute`).

    Every execution goes through the barrier, so a single group with a
    hardware-gated start is a valid topology; plain single-group execution is
    :meth:`MotionGroup.execute`'s job. All groups must live in one cell — the
    sync barrier gates them on that cell's IO (:class:`IOSyncDriver` writes
    within one cell) — though each is still executed through its own gateway.

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

        cells = {motion_group._cell for motion_group in motion_groups.values()}
        if len(cells) > 1:
            raise ValueError(
                f"Synchronized execution is cell-scoped but the motion groups span "
                f"{sorted(cells)}. The sync barrier gates them on one cell's IO."
            )

        motion_groups_without_condition = set(motion_groups) - set(sync.start_conditions())
        if motion_groups_without_condition:
            raise ValueError(
                f"The sync driver has no start condition for groups: "
                f"{sorted(motion_groups_without_condition)}"
            )

        self._motion_groups = dict(motion_groups)
        self._monitors = tuple(monitors)
        self._sync = sync
        # Controller id -> the groups on it, name-ordered so the first is the
        # deterministic pick; fixed for the executor's lifetime.
        self._motion_groups_by_controller: dict[str, list[str]] = {}
        for name in sorted(self._motion_groups):
            self._motion_groups_by_controller.setdefault(
                self._motion_groups[name]._controller_id, []
            ).append(name)

    @property
    def motion_groups(self) -> dict[str, MotionGroup]:
        """The motion groups this executor drives, keyed by name."""
        return dict(self._motion_groups)

    async def execute(
        self,
        trajectory: api.models.MultiJointTrajectory,
        groups: Mapping[str, GroupArgs] | None = None,
        actions: ActionsLike | None = None,
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
        groups: Mapping[str, GroupArgs] | None = None,
        actions: ActionsLike | None = None,
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

        action_list = _normalize_actions(actions or [])
        overlay = self._build_io_overlay(action_list)

        cursors: dict[str, TrajectoryCursor] = {}
        for name, joint_trajectory in per_group.items():
            motion_group = self._motion_groups[name]
            group_args = (groups or {}).get(name) or GroupArgs()
            trajectory_id = await motion_group._load_planned_motion(
                joint_trajectory, group_args.tcp
            )
            # Bind the rate so the cursor still receives a zero-arg stream
            # factory; with no rate the bare bound method already is one.
            state_stream_source = (
                functools.partial(motion_group.stream_state, group_args.state_stream_rate_msecs)
                if group_args.state_stream_rate_msecs is not None
                else motion_group.stream_state
            )
            cursors[name] = TrajectoryCursor(
                motion_id=trajectory_id,
                motion_group_state_stream=state_stream_source,
                joint_trajectory=joint_trajectory,
                detach_on_standstill=False,
                emit_motion_events=False,
                ignore_controller_limits=group_args.ignore_controller_limits,
                set_outputs=overlay[name] or None,
            )

        cursor = MultiTrajectoryCursor(cursors, self._sync, actions=action_list)
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

    def _build_io_overlay(self, actions: Sequence[Action]) -> dict[str, list[api.models.SetIO]]:
        """Turn the action list's writes into a per-group ``SetIO`` overlay.

        Each write is routed to exactly one group and anchored at its motion
        index; all other groups get an empty list. A controller output goes to
        the group on that controller; a cell-wide bus variable goes to one
        deterministic group (all groups share the ``locations`` array, so which
        one carries it does not change the instant).
        """
        overlay: dict[str, list[api.models.SetIO]] = {name: [] for name in self._motion_groups}
        for location, write in located_writes(actions):
            overlay[self._motion_group_for_write(write)].append(write_to_set_io(write, location))
        return overlay

    async def apply_non_motion_actions(self, actions: Sequence[Action]) -> None:
        """Run an all-non-motion action list directly, without a trajectory.

        The multi-group counterpart of ``MotionGroup``'s direct non-motion path:
        a wait sleeps, a write is routed to its owning motion group and set on that
        group's controller. Used by :meth:`MultiMotionGroup.plan_and_execute` when
        there is nothing to plan.
        """
        for action in actions:
            if isinstance(action, WaitAction):
                await asyncio.sleep(action.wait_for_in_seconds)
            elif isinstance(action, WriteAction):
                motion_group = self._motion_group_for_write(action)
                await self._motion_groups[motion_group]._execute_direct_non_motion_actions([action])
            else:
                raise ValueError(f"Not a non-motion action: {type(action).__name__}")

    def _motion_group_for_write(self, write: WriteAction) -> str:
        """The single motion group whose stream fires this write, from the controller
        index built in ``__init__``: the group on the write's controller, or the
        first motion group for a cell-wide bus IO (all groups share the ``locations``
        array, so which one carries it does not change the instant)."""
        if write.origin is api.models.IOOrigin.BUS_IO:
            return min(self._motion_groups)
        if write.device_id is None:
            if len(self._motion_groups_by_controller) > 1:
                raise ValueError(
                    f"The controller for IO '{write.key}' is ambiguous: the motion groups span "
                    f"controllers {sorted(self._motion_groups_by_controller)}. Set the write's "
                    "device_id explicitly."
                )
            return min(self._motion_groups)
        on_controller = self._motion_groups_by_controller.get(write.device_id)
        if not on_controller:
            raise ValueError(
                f"No motion group is on controller '{write.device_id}' for IO '{write.key}'; "
                f"known controllers are {sorted(self._motion_groups_by_controller)}."
            )
        return on_controller[0]

    async def _run_execute_trajectory(self, name: str, cursor: TrajectoryCursor) -> None:
        motion_group = self._motion_groups[name]
        await motion_group._api_client.trajectory_execution_api.execute_trajectory(
            cell=motion_group._cell,
            controller=motion_group._controller_id,
            client_request_generator=cursor.cntrl,  # ty: ignore[invalid-argument-type]
        )
