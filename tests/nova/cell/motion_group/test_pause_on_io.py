"""Tests for pause-on-io functionality in movement controller and motion group execution."""

import asyncio
from datetime import datetime, timezone
from math import pi

import pytest

from nova import Nova, api
from nova.actions import jnt
from nova.actions.container import CombinedActions, MovementControllerContext
from nova.cell import virtual_controller
from nova.cell.movement_controller import move_forward
from nova.types.motion_settings import MotionSettings


@pytest.mark.asyncio
async def test_pause_on_io_in_context_initialization():
    """Tests that MovementControllerContext correctly stores the pause_on_io parameter."""
    pause_io = api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="OUT#900", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )

    async def mock_stream_state():
        yield api.models.MotionGroupState(
            timestamp=datetime.now(timezone.utc),
            sequence_number=0,
            description_revision=0,
            motion_group="mg0",
            controller="test-controller",
            joint_position=[0.0] * 6,
            joint_limit_reached={"values": [False] * 6, "limit_reached": [False] * 6},
            standstill=True,
        )

    context = MovementControllerContext(
        combined_actions=CombinedActions(items=tuple([])),
        motion_id="test-motion-id",
        start_on_io=None,
        pause_on_io=pause_io,
        motion_group_state_stream_gen=mock_stream_state,
    )

    assert context.pause_on_io is not None
    assert context.pause_on_io.io.io == "OUT#900"
    assert context.pause_on_io.comparator == api.models.Comparator.COMPARATOR_EQUALS


@pytest.mark.asyncio
async def test_move_forward_controller_includes_pause_on_io_in_start_request():
    """Tests that move_forward controller includes pause_on_io in StartMovementRequest."""
    pause_io = api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io="OUT#900", value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )

    async def mock_stream_state():
        yield api.models.MotionGroupState(
            timestamp=datetime.now(timezone.utc),
            sequence_number=0,
            description_revision=0,
            motion_group="mg0",
            controller="test-controller",
            joint_position=[0.0] * 6,
            joint_limit_reached={"values": [False] * 6, "limit_reached": [False] * 6},
            standstill=True,
        )

    context = MovementControllerContext(
        combined_actions=CombinedActions(items=tuple([])),
        motion_id="test-motion-id",
        start_on_io=None,
        pause_on_io=pause_io,
        motion_group_state_stream_gen=mock_stream_state,
    )

    controller_fn = move_forward(context)

    async def mock_response_stream():
        yield api.models.InitializeMovementResponse(message=None, add_trajectory_error=None)
        yield api.models.StartMovementResponse()

    start_request = None
    async for request in controller_fn(mock_response_stream()):
        if isinstance(request, api.models.StartMovementRequest):
            start_request = request
            break

    assert start_request is not None
    assert start_request.pause_on_io is not None
    assert start_request.pause_on_io.io.io == "OUT#900"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_pause_on_io_parameter_accepted_by_execution_api():
    """
    Tests that pause_on_io parameter can be passed through the execution API.

    This validates API integration but does not test the actual pause behavior.
    The IO condition is not met, so motion completes normally to target.
    """
    initial_joint_positions = [0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0]
    controller_name = "kuka-pause-on-io-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr6_r700_sixx",
                position=initial_joint_positions,
            )
        )

        kuka = await cell.controller(controller_name)

        async with kuka[0] as mg:
            await kuka.write("OUT#900", False)

            current_joints = await mg.joints()
            target_joints = list(current_joints)
            target_joints[0] += 0.1

            pause_io = api.models.PauseOnIO(
                io=api.models.IOBooleanValue(io="OUT#900", value=True),
                comparator=api.models.Comparator.COMPARATOR_EQUALS,
                io_origin=api.models.IOOrigin.CONTROLLER,
            )

            actions = [jnt(target_joints, settings=MotionSettings(tcp_velocity_limit=100))]

            await asyncio.wait_for(
                mg.plan_and_execute(actions=actions, tcp="Flange", pause_on_io=pause_io),
                timeout=30.0,
            )

            final_joints = await mg.joints()
            assert abs(final_joints[0] - target_joints[0]) < 0.01


@pytest.mark.asyncio
@pytest.mark.integration
async def test_pause_on_io_pauses_the_motion_and_resumes_when_the_signal_clears():
    """
    plan_and_execute(pause_on_io=...) must block through a controller-side IO pause:

    - once moving, setting the IO pauses the robot on path (PAUSED_ON_IO)
    - execute() does not return while the signal holds (ADR 002)
    - clearing the IO resumes the motion; execute() returns at the target
    """
    initial_joint_positions = [0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0]
    controller_name = "kuka-pause-behavior-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr6_r700_sixx",
                position=initial_joint_positions,
            )
        )

        kuka = await cell.controller(controller_name)

        async with kuka[0] as mg:
            await kuka.write("OUT#900", False)
            # The run traverses the full 1.5 rad; start from home so repeated runs on
            # the shared virtual controller stay inside the joint limits.
            await mg.plan_and_execute([jnt(initial_joint_positions[:6])], tcp="Flange")

            current_joints = await mg.joints()
            target_joints = list(current_joints)
            target_joints[0] += 1.5

            pause_io = api.models.PauseOnIO(
                io=api.models.IOBooleanValue(io="OUT#900", value=True),
                comparator=api.models.Comparator.COMPARATOR_EQUALS,
                io_origin=api.models.IOOrigin.CONTROLLER,
            )
            actions = [jnt(target_joints, settings=MotionSettings(tcp_velocity_limit=30))]

            async def wait_for_trajectory_state(kind: type, *, standstill: bool | None = None):
                # Observe the pause on the state stream instead of polling joints —
                # the level-based PAUSED_ON_IO is re-published every step.
                async for state in mg.stream_state(None):
                    details = state.execute.details if state.execute else None
                    if isinstance(details, api.models.TrajectoryDetails) and isinstance(
                        details.state, kind
                    ):
                        if standstill is None or state.standstill == standstill:
                            return state

            motion_task = asyncio.create_task(
                mg.plan_and_execute(actions=actions, tcp="Flange", pause_on_io=pause_io)
            )

            async def observe(kind: type, *, standstill: bool, timeout: float):
                """Wait for a trajectory state, surfacing a failed execute() instead of
                timing out on a state that will never come."""
                waiter = asyncio.ensure_future(
                    wait_for_trajectory_state(kind, standstill=standstill)
                )
                done, _ = await asyncio.wait(
                    {waiter, motion_task}, timeout=timeout, return_when=asyncio.FIRST_COMPLETED
                )
                if waiter not in done:
                    waiter.cancel()
                    if motion_task in done:
                        motion_task.result()  # raises the execute() error
                    raise TimeoutError(
                        f"no {kind.__name__} (standstill={standstill}) in {timeout}s"
                    )
                return waiter.result()

            try:
                await observe(api.models.TrajectoryRunning, standstill=False, timeout=20.0)
                await asyncio.sleep(0.5)
                await kuka.write("OUT#900", True)

                paused = await observe(
                    api.models.TrajectoryPausedOnIO, standstill=True, timeout=10.0
                )
                paused_joints = list(paused.joint_position)
                assert abs(paused_joints[0] - current_joints[0]) > 0.01, "robot didn't move"
                assert abs(paused_joints[0] - target_joints[0]) > 0.5, "motion wasn't paused early"

                # The pause holds: execute() must still be pending.
                await asyncio.sleep(1.0)
                assert not motion_task.done(), "execute() returned on a resumable IO pause"

                await kuka.write("OUT#900", False)
                await asyncio.wait_for(motion_task, timeout=60.0)

                final_joints = await mg.joints()
                assert abs(final_joints[0] - target_joints[0]) < 0.01
            finally:
                await kuka.write("OUT#900", False)
                if not motion_task.done():
                    motion_task.cancel()
