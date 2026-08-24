"""Tests for the MultiMotionGroup facade: that it assembles the executor through
the shared builder and delegates plan/execute/plan_and_execute to the two halves
(the planner and the executor, each covered end to end in its own suite)."""

from contextlib import asynccontextmanager
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova.actions.io import io_write
from nova.actions.motions import multi_collision_free
from nova.cell.multi_motion_group import MultiMotionGroup, MultiMotionGroupBuilder
from nova.cell.trajectory_executor import GroupArgs, TrajectoryExecutor
from tests.cell.multi_group_doubles import IOGateway, motion_group

pytestmark = pytest.mark.asyncio


class TestBuilder:
    def test_builder_returns_a_multi_motion_group_builder(self):
        builder = MultiMotionGroup.builder({"a": motion_group(IOGateway())})
        assert isinstance(builder, MultiMotionGroupBuilder)

    def test_build_wraps_a_trajectory_executor_with_the_same_groups(self):
        gateway = IOGateway()
        groups = {"a": motion_group(gateway), "b": motion_group(gateway)}

        ensemble = (
            MultiMotionGroup.builder(groups)
            .sync_on_io("sync-io", controller="controller-a")
            .build()
        )

        assert isinstance(ensemble, MultiMotionGroup)
        assert isinstance(ensemble._executor, TrajectoryExecutor)
        assert set(ensemble.motion_groups) == {"a", "b"}


def _fake_executor() -> MagicMock:
    executor = MagicMock(spec=TrajectoryExecutor)
    executor.execute = AsyncMock()
    executor.motion_groups = {"a": object(), "b": object()}
    return executor


class TestDelegation:
    async def test_plan_delegates_to_the_planner(self):
        ensemble = MultiMotionGroup(_fake_executor())
        ensemble._planner = MagicMock(plan=AsyncMock(return_value="TRAJ"))
        actions = [multi_collision_free({"a": (0.0,) * 6})]

        result = await ensemble.plan(actions, tcp="flange", start_joint_position={"a": (0.1,) * 6})

        ensemble._planner.plan.assert_awaited_once_with(actions, "flange", {"a": (0.1,) * 6})
        assert result == "TRAJ"

    async def test_execute_delegates_to_the_executor(self):
        executor = _fake_executor()
        ensemble = MultiMotionGroup(executor)
        actions = [io_write("OUT#1", True, device_id="c")]
        group_args = {"a": GroupArgs()}

        await ensemble.execute("TRAJ", actions=actions, groups=group_args)

        executor.execute.assert_awaited_once_with("TRAJ", actions=actions, groups=group_args)

    async def test_attach_delegates_to_the_executor(self):
        executor = _fake_executor()

        @asynccontextmanager
        async def fake_attach(trajectory, actions=None, groups=None):
            fake_attach.args = (trajectory, actions, groups)
            yield "CURSOR"

        executor.attach = fake_attach
        ensemble = MultiMotionGroup(executor)
        actions = [io_write("OUT#1", True, device_id="c")]

        async with ensemble.attach("TRAJ", actions=actions) as cursor:
            assert cursor == "CURSOR"
        assert fake_attach.args == ("TRAJ", actions, None)

    async def test_plan_and_execute_chains_the_same_actions_into_both_halves(self):
        executor = _fake_executor()
        ensemble = MultiMotionGroup(executor)
        ensemble._planner = MagicMock(plan=AsyncMock(return_value="TRAJ"))
        actions = [multi_collision_free({"a": (0.0,) * 6}), io_write("OUT#1", True, device_id="c")]

        await ensemble.plan_and_execute(actions, tcp="flange")

        ensemble._planner.plan.assert_awaited_once_with(actions, "flange", None)
        # the planned trajectory and the very same actions reach execute
        executor.execute.assert_awaited_once_with("TRAJ", actions=actions, groups=None)


class TestLazyPlanner:
    async def test_planner_is_built_once_from_the_executor_groups_on_first_plan(self, monkeypatch):
        executor = _fake_executor()
        created: list[dict] = []
        fake_planner = MagicMock(plan=AsyncMock(return_value="TRAJ"))

        def fake_ctor(groups):
            created.append(groups)
            return fake_planner

        monkeypatch.setattr("nova.cell.multi_motion_group.MultiMotionGroupPlanner", fake_ctor)
        ensemble = MultiMotionGroup(executor)

        await ensemble.plan([multi_collision_free({"a": (0.0,) * 6})])
        await ensemble.plan([multi_collision_free({"a": (1.0,) * 6})])

        # built exactly once, from the executor's motion groups, then cached
        assert created == [executor.motion_groups]

    async def test_execute_only_never_builds_the_planner(self):
        # An execute-only ensemble must not trigger the planner's single-cell
        # requirement (execution may span cells; planning may not).
        executor = _fake_executor()
        ensemble = MultiMotionGroup(executor)

        await ensemble.execute("TRAJ")

        assert ensemble._planner is None
