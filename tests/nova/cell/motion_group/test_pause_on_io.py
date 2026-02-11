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
    assert context.pause_on_io.io.root.io == "OUT#900"
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
        yield api.models.ExecuteTrajectoryResponse(
            root=api.models.InitializeMovementResponse(message=None, add_trajectory_error=None)
        )
        yield api.models.ExecuteTrajectoryResponse(root=api.models.StartMovementResponse())

    start_request = None
    async for request in controller_fn(mock_response_stream()):
        if isinstance(request, api.models.StartMovementRequest):
            start_request = request
            break

    assert start_request is not None
    assert start_request.pause_on_io is not None
    assert start_request.pause_on_io.io.root.io == "OUT#900"


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
                type=api.models.VirtualControllerTypes.KUKA_KR6_R700_SIXX,
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
async def test_pause_on_io_stops_motion_early_when_triggered():
    """
    Tests that pause_on_io stops motion early when IO condition is met during execution.

    Verifies:
    - Motion starts and robot begins moving
    - Triggering IO during motion stops trajectory early
    - No exception is raised (TrajectoryPausedOnIO treated as successful completion)
    - Robot stops between start and target positions
    """
    initial_joint_positions = [0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0]
    controller_name = "kuka-pause-behavior-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_KR6_R700_SIXX,
                position=initial_joint_positions,
            )
        )

        kuka = await cell.controller(controller_name)

        async with kuka[0] as mg:
            await kuka.write("OUT#900", False)

            start_time = datetime.now()
            while True:
                io_value = await kuka.read("OUT#900")
                if not io_value:
                    break
                if (datetime.now() - start_time).total_seconds() > 2.0:
                    raise TimeoutError("Timed out waiting for OUT#900 to become False")
                await asyncio.sleep(0.05)

            current_joints = await mg.joints()
            target_joints = list(current_joints)
            target_joints[0] += 1.5

            pause_io = api.models.PauseOnIO(
                io=api.models.IOBooleanValue(io="OUT#900", value=True),
                comparator=api.models.Comparator.COMPARATOR_EQUALS,
                io_origin=api.models.IOOrigin.CONTROLLER,
            )

            actions = [jnt(target_joints, settings=MotionSettings(tcp_velocity_limit=30))]

            async def trigger_io_after_motion_starts():
                start_time = datetime.now()
                while True:
                    current = await mg.joints()
                    movement = abs(current[0] - current_joints[0])
                    if movement > 0.01:
                        await kuka.write("OUT#900", True)
                        break
                    await asyncio.sleep(0.1)
                    if (datetime.now() - start_time).total_seconds() > 5.0:
                        raise TimeoutError("Motion never started")

            try:
                motion_task = asyncio.create_task(
                    mg.plan_and_execute(actions=actions, tcp="Flange", pause_on_io=pause_io)
                )
                trigger_task = asyncio.create_task(trigger_io_after_motion_starts())

                await asyncio.wait_for(asyncio.gather(motion_task, trigger_task), timeout=30.0)

                final_joints = await mg.joints()

                movement_amount = abs(final_joints[0] - current_joints[0])
                distance_to_target = abs(final_joints[0] - target_joints[0])

                assert movement_amount > 0.01, (
                    f"Robot didn't move (moved only {movement_amount:.3f} rad)"
                )
                assert distance_to_target > 0.5, (
                    f"Motion wasn't interrupted (only {distance_to_target:.3f} rad from target)"
                )
            finally:
                await kuka.write("OUT#900", False)
