"""Tests for pause-on-io functionality in movement controller and motion group execution."""

import asyncio

import pytest

from nova import Nova, api
from nova.actions import jnt, lin
from nova.actions.container import CombinedActions, MovementControllerContext
from nova.cell import virtual_controller
from nova.cell.movement_controller import move_forward
from nova.types import Pose
from nova.types.motion_settings import MotionSettings


@pytest.mark.asyncio
async def test_pause_on_io_parameter_forwarded_to_context():
    """
    Tests that pause_on_io parameter is correctly set in MovementControllerContext.
    """
    from datetime import datetime, timezone

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
    """
    Tests that move_forward controller includes pause_on_io in StartMovementRequest.
    """
    from datetime import datetime, timezone

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

    # Capture StartMovementRequest
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
async def test_pause_on_io_with_virtual_controller_terminates_motion():
    """
    Tests that when pause_on_io is configured and the IO signal changes during motion,
    the trajectory stops early and is treated as completed (TrajectoryPausedOnIO).
    """
    initial_joint_positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
    controller_name = "kuka-pause-on-io-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_KR6_R700_SIXX,
            )
        )

        kuka = await cell.controller(controller_name)
        async with kuka[0] as mg:
            # Move to initial position
            await mg.plan_and_execute(
                actions=[
                    jnt(initial_joint_positions, settings=MotionSettings(tcp_velocity_limit=250))
                ],
                tcp="Flange",
            )

            # Ensure the IO is initially False
            await kuka.write("OUT#900", False)
            await asyncio.sleep(0.1)

            # Get current pose and plan a longer motion (1000mm to give time for IO change)
            initial_pose = await mg.tcp_pose("Flange")
            target_pose = initial_pose @ Pose((1000, 0, 0, 0, 0, 0))

            pause_io = api.models.PauseOnIO(
                io=api.models.IOBooleanValue(io="OUT#900", value=True),
                comparator=api.models.Comparator.COMPARATOR_EQUALS,
                io_origin=api.models.IOOrigin.CONTROLLER,
            )

            actions = [lin(target_pose, settings=MotionSettings(tcp_velocity_limit=100))]
            trajectory = await mg.plan(actions=actions, tcp="Flange")

            # Start motion in a task so we can trigger IO change during execution
            movement_task = asyncio.create_task(
                mg.execute(trajectory, "Flange", actions, pause_on_io=pause_io)
            )

            # Wait for motion to start (2 seconds should be enough)
            await asyncio.sleep(2)

            # Trigger the IO change to pause the motion
            await kuka.write("OUT#900", True)

            # Wait for the motion to complete (should pause due to IO)
            try:
                await asyncio.wait_for(movement_task, timeout=5.0)
            except asyncio.TimeoutError:
                pytest.fail("Motion did not complete within timeout after IO trigger")

            # Wait for deceleration
            await asyncio.sleep(1)

            # Verify the robot stopped before reaching the target
            final_pose = await mg.tcp_pose("Flange")
            assert final_pose.position.x > initial_pose.position.x, "Robot did not move at all"
            assert final_pose.position.x < target_pose.position.x, (
                "Robot completed full movement despite pause-on-io trigger"
            )
