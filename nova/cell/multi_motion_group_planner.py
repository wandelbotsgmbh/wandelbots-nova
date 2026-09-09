"""Collision-free planning of one synchronized trajectory across motion groups.

:class:`MultiMotionGroupPlanner` is to :class:`MotionGroup.plan` what
:class:`TrajectoryExecutor` is to :meth:`MotionGroup.execute`: the multi-group
counterpart. ``plan`` takes one ensemble action list — the same shape
:meth:`MotionGroup.plan` accepts — and returns the
:class:`api.models.MultiJointTrajectory` the executor consumes, so the two
compose end to end:

    ```python
    planner = MultiMotionGroupPlanner({"0@kuka": robot_mg, "1@kuka": positioner_mg})
    trajectory = await planner.plan(
        [
            multi_collision_free({"0@kuka": Pose(...), "1@kuka": (j1, j2, j3, j4, j5, j6)}),
            wait(0.5),
            multi_collision_free({"0@kuka": (...), "1@kuka": (...)}),
        ],
        tcp={"0@kuka": "1", "1@kuka": "0"},
    )

    ensemble = MultiMotionGroup.builder(planner.motion_groups).sync_on_io("OUT#1").build()
    await ensemble.execute(trajectory)
    ```

The batch logic mirrors :meth:`MotionGroup.plan`: a
:class:`~nova.actions.motions.MultiCollisionFreeMotion` is planned by the
multi-motion-group RRT endpoint (``search_collision_free_multi_motion_group``)
into a synchronized segment; a :class:`~nova.actions.mock.WaitAction` becomes an
ensemble hold; segments are concatenated into one shared ``times``/``locations``
— the invariant synchronized execution rests on.

Like :meth:`MotionGroup.plan`, ``plan`` returns a pure trajectory: a
:class:`~nova.actions.io.WriteAction` in the action list contributes no joint
samples and is ignored here. Its location-anchored IO overlay is derived from
the same action list at execute time, by
:meth:`~nova.cell.trajectory_executor.TrajectoryExecutor.execute` — so the
``actions`` list is passed to both, exactly as in the single-motion-group flow.
"""

from collections.abc import Mapping

from nova import api
from nova.actions.io import WriteAction
from nova.actions.mock import WaitAction
from nova.actions.motions import MultiCollisionFreeMotion
from nova.cell.motion_group import MotionGroup, _find_and_sort_best_joint_solutions
from nova.cell.robot_cell import ActionsLike, _normalize_actions
from nova.exceptions import NoInverseKinematicsSolutionFound, PlanTrajectoryFailed
from nova.types import Pose
from nova.utils.joint_trajectory import combine_multi_trajectories

_WAIT_TIMESTEP = 0.050  # 50 ms, matching MotionGroup._build_wait_trajectory


class MultiMotionGroupPlanner:
    """Plan one synchronized, collision-free trajectory across several motion groups.

    Created once for the motion groups to coordinate; ``plan`` is called per
    action list. All groups must live in one cell — the RRT endpoint is
    cell-scoped and reasons about their mutual geometry in a single request.
    """

    def __init__(self, motion_groups: Mapping[str, MotionGroup]):
        self._motion_groups = dict(motion_groups)
        if not self._motion_groups:
            raise ValueError("At least one motion group is required")

        cells = {motion_group._cell for motion_group in self._motion_groups.values()}
        if len(cells) > 1:
            raise ValueError(
                f"Multi-motion-group planning is cell-scoped but the motion groups span "
                f"{sorted(cells)}. Plan one cell at a time."
            )
        self._cell = next(iter(cells))
        # Controllers in a cell share the cell's gateway; the single RRT request
        # goes to one, so pin it here.
        self._api_client = next(iter(self._motion_groups.values()))._api_client

    @property
    def motion_groups(self) -> dict[str, MotionGroup]:
        """The motion groups this planner coordinates, keyed by name.

        Handy to hand straight to :meth:`MultiMotionGroup.builder`."""
        return dict(self._motion_groups)

    async def plan(
        self,
        actions: ActionsLike,
        tcp: Mapping[str, str | None] | str | None = None,
        start_joint_position: Mapping[str, tuple[float, ...]] | None = None,
    ) -> api.models.MultiJointTrajectory:
        """Plan a synchronized collision-free trajectory for the action list.

        Mirrors :meth:`MotionGroup.plan`: an ensemble action list is batched and
        each batch planned in turn, every segment continuing from the previous
        one's end, then all concatenated into a single shared parameterization.

        Args:
            actions: The ensemble actions — one or a sequence of
                :class:`~nova.actions.motions.MultiCollisionFreeMotion` (each
                planned by RRT into a synchronized segment),
                :class:`~nova.actions.mock.WaitAction` (an ensemble hold) and
                :class:`~nova.actions.io.WriteAction` (ignored here — it carries
                no joint samples; pass the same list to the executor for its IO
                overlay). Every motion's ``targets`` must cover exactly the
                planner's groups.
            tcp: The TCP per group, or a single TCP shared by all. Required for
                any group whose target is a pose (for inverse kinematics); also
                selects the motion group setup. ``None`` for a group means no TCP.
            start_joint_position: The start joint position per group. A group
                left out starts from its current joints.

        Returns:
            The synchronized trajectory, ready for :meth:`TrajectoryExecutor.execute`.

        Raises:
            ValueError: An action type has no place in a collision-free plan, or
                a motion's targets do not match the planner's motion groups.
            PlanTrajectoryFailed: The RRT search found no coordinated path.
            NoInverseKinematicsSolutionFound: A pose target has no IK solution.
        """
        actions_list = _normalize_actions(actions)
        if not actions_list:
            raise ValueError("No actions provided")

        setups = {
            name: await motion_group.get_setup(tcp_name=self._tcp_for(name, tcp))
            for name, motion_group in self._motion_groups.items()
        }
        current: dict[str, tuple[float, ...]] = {
            name: (start_joint_position or {}).get(name) or await motion_group.joints()
            for name, motion_group in self._motion_groups.items()
        }

        segments: list[api.models.MultiJointTrajectory] = []
        for action in actions_list:
            if isinstance(action, MultiCollisionFreeMotion):
                segment = await self._plan_multi_collision_free(action, tcp, current, setups)
            elif isinstance(action, WaitAction):
                segment = self._build_wait_segment(current, action.wait_for_in_seconds)
            elif isinstance(action, WriteAction):
                # Writes carry no joint samples; their IO overlay is derived at
                # execute time (see the module docstring).
                continue
            else:
                raise ValueError(
                    f"Action type '{type(action).__name__}' is not supported by synchronized "
                    "collision-free planning; use multi_collision_free(), wait() or io_write()."
                )
            # The segment's last sample is the next batch's start, per group.
            for name, samples in segment.joint_positions_by_motion_group_key.items():
                current[name] = tuple(samples[-1])
            segments.append(segment)

        if not segments:
            raise ValueError(
                "The action list has no motion or wait to plan; it carries only writes."
            )
        return combine_multi_trajectories(segments)

    def _tcp_for(self, name: str, tcp: Mapping[str, str | None] | str | None) -> str | None:
        return tcp.get(name) if isinstance(tcp, Mapping) else tcp

    async def _plan_multi_collision_free(
        self,
        action: MultiCollisionFreeMotion,
        tcp: Mapping[str, str | None] | str | None,
        current: Mapping[str, tuple[float, ...]],
        setups: Mapping[str, api.models.MotionGroupSetup],
    ) -> api.models.MultiJointTrajectory:
        if set(action.targets) != set(self._motion_groups):
            raise ValueError(
                f"Motion targets {sorted(action.targets)} must match the planner's "
                f"motion groups {sorted(self._motion_groups)}"
            )

        path_definitions: dict[str, api.models.JointPTPMotion] = {}
        for name, target in action.targets.items():
            motion_group = self._motion_groups[name]
            group_tcp = self._tcp_for(name, tcp)
            start = current[name]
            target_joints = await self._resolve_target(
                target, group_tcp, start, motion_group, setups[name]
            )
            settings = action.settings.get(name)
            path_definitions[name] = api.models.JointPTPMotion(
                start_joint_position=list(start),
                target_joint_position=list(target_joints),
                limits_override=(
                    settings.as_limits_settings()
                    if settings is not None and settings.has_limits_override()
                    else None
                ),
            )

        request = api.models.MultiSearchCollisionFreeRequest(
            motion_group_setups_by_motion_group_key=dict(setups),
            path_definitions_by_motion_group_key=path_definitions,
            collision_setups=(
                dict(action.collision_setups) if action.collision_setups is not None else None
            ),
            algorithm_settings=action.algorithm,
        )

        response = (
            await self._api_client.trajectory_planning_api.search_collision_free_multi_motion_group(
                cell=self._cell, multi_search_collision_free_request=request
            )
        )

        if isinstance(response.response, api.models.PlanCollisionFreeFailedResponse):
            raise PlanTrajectoryFailed(
                error=response.response, motion_group_id=", ".join(sorted(self._motion_groups))
            )
        if not isinstance(response.response, api.models.MultiJointTrajectory):
            raise ValueError("Multi-motion-group planning returned no trajectory")
        return response.response

    def _build_wait_segment(
        self, current: Mapping[str, tuple[float, ...]], wait_time: float
    ) -> api.models.MultiJointTrajectory:
        """Hold every group at its current joints for ``wait_time`` seconds.

        The multi-group counterpart of ``MotionGroup._build_wait_trajectory``:
        50 ms steps, locations flat at 0 (a hold makes no path progress)."""
        num_steps = max(2, int(wait_time / _WAIT_TIMESTEP) + 1)
        times = [i * _WAIT_TIMESTEP for i in range(num_steps)]
        times[-1] = wait_time
        return api.models.MultiJointTrajectory(
            joint_positions_by_motion_group_key={
                name: [list(joints) for _ in range(num_steps)] for name, joints in current.items()
            },
            times=times,
            locations=[0.0 for _ in range(num_steps)],
        )

    async def _resolve_target(
        self,
        target: Pose | tuple[float, ...],
        tcp: str | None,
        start: tuple[float, ...],
        motion_group: MotionGroup,
        setup: api.models.MotionGroupSetup,
    ) -> tuple[float, ...]:
        """Resolve a target to joints, solving IK for a pose and picking the
        solution closest to the start."""
        if isinstance(target, tuple):
            return target
        if tcp is None:
            raise ValueError(
                f"TCP is required for a pose target on '{motion_group.id}'; pass tcp or use a "
                "joint-space target instead."
            )
        solutions = await motion_group._inverse_kinematics(
            poses=[target], tcp=tcp, motion_group_setup=setup
        )
        if not solutions or not solutions[0]:
            raise NoInverseKinematicsSolutionFound(target)
        joint_limits = setup.global_limits.joints if setup.global_limits is not None else None
        return _find_and_sort_best_joint_solutions(start, solutions[0], joint_limits)[0]
