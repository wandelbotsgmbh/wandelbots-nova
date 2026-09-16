"""Tests for MultiMotionGroup: that its builder wires a planner and an executor
over the same groups, and that it delegates plan/execute/plan_and_execute to the
two halves (each covered end to end in its own suite)."""

from contextlib import asynccontextmanager
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova.actions.io import io_write
from nova.actions.motions import multi_collision_free
from nova.cell.multi_motion_group import MultiMotionGroup, MultiMotionGroupBuilder
from nova.cell.trajectory_executor import GroupArgs, TrajectoryExecutor
from tests.cell.multi_group_doubles import IOGateway, motion_group

pytestmark = pytest.mark.asyncio


def _half(keys: tuple[str, ...] = ("a", "b")) -> MagicMock:
    half = MagicMock()
    half.motion_groups = {key: MagicMock() for key in keys}
    return half


def _fake_executor(keys: tuple[str, ...] = ("a", "b")) -> MagicMock:
    executor = _half(keys)
    executor.execute = AsyncMock()
    return executor


def _fake_planner(keys: tuple[str, ...] = ("a", "b")) -> MagicMock:
    planner = _half(keys)
    planner.plan = AsyncMock(return_value="TRAJ")
    return planner


class TestBuilder:
    def test_builder_returns_a_multi_motion_group_builder(self):
        builder = MultiMotionGroup.builder({"a": motion_group(IOGateway())})
        assert isinstance(builder, MultiMotionGroupBuilder)

    def test_build_wires_a_planner_and_executor_over_the_same_groups(self):
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
        assert set(ensemble._planner.motion_groups) == {"a", "b"}


class TestConstruction:
    def test_planner_and_executor_over_different_groups__raises(self):
        with pytest.raises(ValueError, match="same motion groups"):
            MultiMotionGroup(_fake_executor(("a", "b")), _fake_planner(("a", "c")))


class TestDelegation:
    async def test_plan_delegates_to_the_planner(self):
        planner = _fake_planner()
        ensemble = MultiMotionGroup(_fake_executor(), planner)
        actions = [multi_collision_free({"a": (0.0,) * 6})]

        result = await ensemble.plan(
            actions, tcp={"a": "flange"}, start_joint_position={"a": (0.1,) * 6}
        )

        planner.plan.assert_awaited_once_with(actions, {"a": "flange"}, {"a": (0.1,) * 6})
        assert result == "TRAJ"

    async def test_execute_delegates_to_the_executor(self):
        executor = _fake_executor()
        ensemble = MultiMotionGroup(executor, _fake_planner())
        actions = [io_write("OUT#1", True, device_id="c")]
        group_args = {"a": GroupArgs()}

        await ensemble.execute("TRAJ", tcp={"a": "flange"}, actions=actions, groups=group_args)

        executor.execute.assert_awaited_once_with(
            "TRAJ", tcp={"a": "flange"}, actions=actions, groups=group_args
        )

    async def test_attach_delegates_to_the_executor(self):
        executor = _fake_executor()

        @asynccontextmanager
        async def fake_attach(trajectory, tcp=None, actions=None, groups=None):
            fake_attach.args = (trajectory, tcp, actions, groups)
            yield "CURSOR"

        executor.attach = fake_attach
        ensemble = MultiMotionGroup(executor, _fake_planner())
        actions = [io_write("OUT#1", True, device_id="c")]

        async with ensemble.attach("TRAJ", tcp={"a": "flange"}, actions=actions) as cursor:
            assert cursor == "CURSOR"
        assert fake_attach.args == ("TRAJ", {"a": "flange"}, actions, None)

    async def test_plan_and_execute_chains_the_same_actions_into_both_halves(self):
        executor = _fake_executor()
        planner = _fake_planner()
        ensemble = MultiMotionGroup(executor, planner)
        actions = [multi_collision_free({"a": (0.0,) * 6}), io_write("OUT#1", True, device_id="c")]

        await ensemble.plan_and_execute(actions, tcp={"a": "flange"})

        planner.plan.assert_awaited_once_with(actions, {"a": "flange"}, None)
        executor.execute.assert_awaited_once_with(
            "TRAJ", tcp={"a": "flange"}, groups=None, actions=actions
        )

    async def test_plan_and_execute_forwards_tcp_and_groups_orthogonally(self):
        executor = _fake_executor()
        planner = _fake_planner()
        ensemble = MultiMotionGroup(executor, planner)
        actions = [multi_collision_free({"a": (0.0,) * 6})]
        # tcp is the planning input, groups holds only execution knobs: both reach
        # execute untouched, neither derived from the other.
        tcp = {"a": "flange", "b": None}
        groups = {"a": GroupArgs(state_stream_rate_msecs=50)}

        await ensemble.plan_and_execute(actions, tcp=tcp, groups=groups)

        planner.plan.assert_awaited_once_with(actions, tcp, None)
        executor.execute.assert_awaited_once_with("TRAJ", tcp=tcp, groups=groups, actions=actions)

    async def test_execute_does_not_touch_the_planner(self):
        # An execute-only workflow drives the executor without planning.
        planner = _fake_planner()
        ensemble = MultiMotionGroup(_fake_executor(), planner)

        await ensemble.execute("TRAJ")

        planner.plan.assert_not_awaited()

    async def test_plan_and_execute_only_writes__applies_directly_without_planning(self):
        # An all-non-motion list sets IO directly, mirroring MotionGroup.
        executor = _fake_executor()
        executor.apply_non_motion_actions = AsyncMock()
        planner = _fake_planner()
        ensemble = MultiMotionGroup(executor, planner)
        actions = [io_write("OUT#1", True, device_id="c")]

        await ensemble.plan_and_execute(actions)

        executor.apply_non_motion_actions.assert_awaited_once_with(actions)
        planner.plan.assert_not_awaited()
        executor.execute.assert_not_awaited()
