"""Acceptance tests for ``Action.tcp`` modes against a real NOVA instance.

Each test provisions a throwaway virtual UR10e (deleted on teardown), moves it to a
tool-down start, lets a scripted policy send one TCP chunk through the executor in
sequential mode, and measures where the TCP actually came to rest. That covers the
whole path a policy's TCP action takes: unit operators, relative composition, the
executor's waypoint jogging and NOVA's inverse kinematics. They require NOVA_API /
NOVA_ACCESS_TOKEN and are skipped unless run with ``-m integration``.
"""

from __future__ import annotations

import contextlib
import math
from typing import TYPE_CHECKING
import uuid

import pytest
from scipy.spatial.transform import Rotation

from novapolicy.executor import PolicyExecutor
from novapolicy.ops import Scale
from novapolicy.policy_client import CallbackPolicyClient
from novapolicy.schema import Action, Observation, PolicySchema
from novapolicy.types import ActionChunk, SequentialExecution, StopContext

if TYPE_CHECKING:
    from collections.abc import Callable
    from typing import Any

    from novapolicy.schema import _ActTcp

# Tool pointing straight down, well inside the UR10e's reach.
START_DEG = (0.0, -90.0, 90.0, -90.0, -90.0, 0.0)
DT_MS = 50.0
STEPS = 10
POSITION_TOL_MM = 1.0
ROTATION_TOL_DEG = 0.5


async def _virtual_ur10e(cell, stack: contextlib.AsyncExitStack):
    from nova import api
    from nova.cell import virtual_controller

    name = f"tcp-itest-{uuid.uuid4().hex[:8]}"
    controller = await cell.ensure_controller(
        virtual_controller(
            name=name,
            manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
            type="universalrobots-ur10e",
        )
    )

    async def _cleanup() -> None:
        with contextlib.suppress(Exception):
            await cell.delete_robot_controller(name, timeout=25)

    stack.push_async_callback(_cleanup)
    return controller


def _pose(state) -> list[float]:
    return [*state.pose.position, *state.pose.orientation]


async def _run_one_chunk(
    action_for: Callable[[Any], _ActTcp],
    steps: list[list[float]] | Callable[[list[float]], list[list[float]]],
) -> tuple[list[float], list[float]]:
    """Run one TCP chunk on a fresh virtual UR10e; return (start, end) TCP poses.

    ``action_for(mg)`` builds the ``Action.tcp`` entry; ``steps`` may depend on the start
    pose, so it may also be a callable taking it.
    """
    from nova import Nova

    async with Nova() as nova_instance, contextlib.AsyncExitStack() as stack:
        controller = await _virtual_ur10e(nova_instance.cell(), stack)
        mg = await stack.enter_async_context(controller[0])
        tcp = (await mg.tcp_names())[0]
        calls = 0
        start: list[float] = []

        async def policy(_obs: object) -> ActionChunk:
            nonlocal calls, start
            calls += 1
            if not start:
                start = _pose(await mg.get_state(tcp))
            chunk = steps if isinstance(steps, list) else steps(start)
            return ActionChunk(tcp={mg.id: chunk}, dt_ms=DT_MS)

        def after_first_chunk(_ctx: StopContext) -> bool:
            return calls >= 2  # the first chunk ran to standstill

        action: _ActTcp = action_for(mg)
        schema = PolicySchema(
            observations=[Observation.tcp(action.key, source=mg, tcp=tcp)], actions=[action]
        )
        result = await PolicyExecutor(
            schema,
            CallbackPolicyClient(policy),
            execution=SequentialExecution(),
            stop_conditions=[after_first_chunk],
            start_joint_position={mg: [math.radians(d) for d in START_DEG]},
            timeout_s=30.0,
        ).run()
        assert result.reason == "stop condition: after_first_chunk", result.reason
        end = _pose(await mg.get_state(tcp))
    return start, end


def _assert_pose(end: list[float], position: list[float], rotation: Rotation) -> None:
    assert math.dist(end[:3], position) < POSITION_TOL_MM, (end[:3], position)
    angle = (Rotation.from_rotvec(end[3:]) * rotation.inv()).magnitude()
    assert math.degrees(angle) < ROTATION_TOL_DEG, math.degrees(angle)


def _ramp(total: list[float]) -> list[list[float]]:
    """Offsets reaching ``total`` in STEPS equal steps, each from the start pose."""
    return [[v * (i + 1) / STEPS for v in total] for i in range(STEPS)]


@pytest.mark.integration
@pytest.mark.asyncio
async def test_absolute_tcp_targets_are_reached():
    def down_30(start: list[float]) -> list[list[float]]:
        return [[*start[:2], start[2] - 3.0 * (i + 1), *start[3:]] for i in range(STEPS)]

    start, end = await _run_one_chunk(lambda mg: Action.tcp("eef", target=mg), down_30)
    _assert_pose(end, [*start[:2], start[2] - 30.0], Rotation.from_rotvec(start[3:]))


@pytest.mark.integration
@pytest.mark.asyncio
async def test_relative_offsets_are_taken_from_the_pose_at_inference():
    start, end = await _run_one_chunk(
        lambda mg: Action.tcp("eef", target=mg, mode="relative"),
        _ramp([20.0, -10.0, -30.0, 0.0, 0.0, 0.0]),
    )
    _assert_pose(
        end, [start[0] + 20.0, start[1] - 10.0, start[2] - 30.0], Rotation.from_rotvec(start[3:])
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_accumulated_offsets_chain_from_step_to_step():
    start, end = await _run_one_chunk(
        lambda mg: Action.tcp("eef", target=mg, mode="relative", accumulate=True),
        [[2.0, -1.0, -3.0, 0.0, 0.0, 0.0]] * STEPS,
    )
    _assert_pose(
        end, [start[0] + 20.0, start[1] - 10.0, start[2] - 30.0], Rotation.from_rotvec(start[3:])
    )


@pytest.mark.integration
@pytest.mark.asyncio
async def test_tool_coordinate_system_offsets_follow_the_tool_axes():
    start, end = await _run_one_chunk(
        lambda mg: Action.tcp("eef", target=mg, mode="relative", use_tool_coordinate_system=True),
        _ramp([0.0, 0.0, 30.0, 0.0, 0.0, 0.0]),  # 30 mm along the tool's z axis
    )
    r0 = Rotation.from_rotvec(start[3:])
    expected = [s + d for s, d in zip(start[:3], r0.apply([0.0, 0.0, 30.0]), strict=True)]
    _assert_pose(end, expected, r0)
    assert end[2] < start[2] - 25.0  # tool-down: along the tool's z is down in the world


@pytest.mark.integration
@pytest.mark.asyncio
async def test_rotation_offsets_turn_the_tool_about_the_tcp():
    yaw = math.radians(15.0)
    start, end = await _run_one_chunk(
        lambda mg: Action.tcp("eef", target=mg, mode="relative"),
        _ramp([0.0, 0.0, 0.0, 0.0, 0.0, yaw]),
    )
    turned = Rotation.from_rotvec([0.0, 0.0, yaw]) * Rotation.from_rotvec(start[3:])
    _assert_pose(end, start[:3], turned)  # the TCP stays where it is


@pytest.mark.integration
@pytest.mark.asyncio
async def test_unit_operators_apply_before_the_offsets():
    start, end = await _run_one_chunk(
        lambda mg: Action.tcp(
            "eef",
            target=mg,
            mode="relative",
            accumulate=True,
            position_ops=[Scale(0.001)],  # the policy speaks metres
        ),
        [[0.0, 0.0, -0.003, 0.0, 0.0, 0.0]] * STEPS,
    )
    _assert_pose(end, [*start[:2], start[2] - 30.0], Rotation.from_rotvec(start[3:]))
