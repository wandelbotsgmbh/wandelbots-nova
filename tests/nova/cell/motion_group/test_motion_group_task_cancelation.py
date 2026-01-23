import asyncio
from contextlib import aclosing
from multiprocessing import Process

import pytest

from nova import Nova, api
from nova.actions import jnt, lin
from nova.actions.container import MovementController, MovementControllerContext
from nova.cell.controllers import virtual_controller
from nova.cell.movement_controller import move_forward
from nova.logging import logger
from nova.types import (
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MovementControllerFunction,
)
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose

# logger = logging.getLogger(__name__)


@pytest.fixture
async def ur_mg():
    """
    Fixture that sets up a robot with virtual controller at a specific start position.

    Yields:
        MotionGroup: Motion group ready for task cancellation tests
    """
    initial_joint_positions = [1.8294, -1.4618, -1.8644, -1.1851, 1.5188, 0.2529]
    controller_name = "ur-movement-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
            )
        )

        ur = await cell.controller(controller_name)
        async with ur[0] as mg:
            logger.info("Moving to initial joint positions for test.")
            await mg.plan_and_execute(
                actions=[
                    jnt(initial_joint_positions, settings=MotionSettings(tcp_velocity_limit=250))
                ],
                tcp="Flange",
            )
            try:
                yield mg
            finally:
                logger.info("Moving back to initial joint positions after test.")
                # Move back to the initial position
                await mg.plan_and_execute(
                    actions=[
                        jnt(
                            initial_joint_positions, settings=MotionSettings(tcp_velocity_limit=250)
                        )
                    ],
                    tcp="Flange",
                )


@pytest.mark.asyncio
@pytest.mark.integration
async def test_movement_stops_when_canceling_task_with_execute(ur_mg):
    """
    Test that when you move a robot in an asyncio task running motion_group.execute and cancel the task,
    the robot stops moving.
    """
    movement_in_x = 800
    initial_pose = await ur_mg.tcp_pose()
    final_pose = initial_pose @ Pose((movement_in_x, 0, 0))
    actions = [
        # move 800 mm in X direction with 50 mm/s
        # should give us enough time to cancel the task
        lin(final_pose, settings=MotionSettings(tcp_velocity_limit=50))
    ]
    trajectory = await ur_mg.plan(actions=actions, tcp="Flange")

    movement_task = asyncio.create_task(ur_mg.execute(trajectory, actions=actions, tcp="Flange"))

    await asyncio.sleep(2)

    try:
        result = movement_task.cancel()
        logger.info(f"Cancelation sent: {result}")
        await movement_task
    except asyncio.CancelledError as e:
        logger.info(f"Task was cancelled: {e}")

    # time for deceleration
    await asyncio.sleep(1)
    pose = await ur_mg.tcp_pose()
    assert pose.position.x > initial_pose.position.x, "Robot did not move at all."
    assert pose.position.x < final_pose.position.x, (
        "Robot completed the full movement despite cancelation."
    )

    await asyncio.sleep(2)
    new_pose = await ur_mg.tcp_pose()
    assert pose.position.x == new_pose.position.x, "Robot moved after task was cancelled."


@pytest.mark.asyncio
@pytest.mark.integration
async def test_movement_stops_when_async_generator_raises_exception(ur_mg):
    """
    Test that when you move a robot by motion_group.stream_execute and
    raise an exception in the state consuming async generator,
    the robot stops moving.
    """
    movement_in_x = 800
    initial_pose = await ur_mg.tcp_pose()
    final_pose = initial_pose @ Pose((movement_in_x, 0, 0))
    actions = [
        # move 800 mm in X direction with 50 mm/s
        # should give us enough time to cancel the task
        lin(final_pose, settings=MotionSettings(tcp_velocity_limit=50))
    ]
    trajectory = await ur_mg.plan(actions=actions, tcp="Flange")

    try:
        number_of_states_to_consume = 100
        stream_execute = ur_mg.stream_execute(trajectory, "Flange", actions=actions)
        async with aclosing(stream_execute) as state_stream:
            async for _ in state_stream:
                number_of_states_to_consume -= 1
                if number_of_states_to_consume == 0:
                    raise Exception("Intentional exception to test movement stop on exception.")

    except BaseException as e:
        logger.info(f"Caught expected exception: {e}")

    # time for deceleration
    await asyncio.sleep(1)
    pose = await ur_mg.tcp_pose()
    assert pose.position.x > initial_pose.position.x, "Robot did not move at all."
    assert pose.position.x <= final_pose.position.x, (
        "Robot completed the full movement despite cancelation."
    )

    await asyncio.sleep(2)
    new_pose = await ur_mg.tcp_pose()
    assert pose.position.x == new_pose.position.x, "Robot moved after task was cancelled."


def create_movement_controller(exception: BaseException) -> MovementController:
    def _test_movement_controller(context: MovementControllerContext) -> MovementControllerFunction:
        controller = move_forward(context)

        async def movement_controller(
            response_stream: ExecuteTrajectoryResponseStream,
        ) -> ExecuteTrajectoryRequestStream:
            async def intercepted_response_stream():
                try:
                    async with asyncio.timeout(1):
                        async for response in response_stream:
                            yield response
                except Exception:
                    raise exception

            request_stream = controller(intercepted_response_stream())
            async for request in request_stream:
                yield request

        return movement_controller

    return _test_movement_controller


@pytest.mark.asyncio
@pytest.mark.integration
async def test_movement_stops_when_custom_controller_raises(ur_mg):
    movement_in_x = 800
    initial_pose = await ur_mg.tcp_pose()
    final_pose = initial_pose @ Pose((movement_in_x, 0, 0))
    actions = [
        # move 800 mm in X direction with 50 mm/s
        # should give us enough time to cancel the task
        lin(final_pose, settings=MotionSettings(tcp_velocity_limit=50))
    ]
    trajectory = await ur_mg.plan(actions=actions, tcp="Flange")

    try:
        exception = Exception("Intentional exception to test movement stop on exception.")
        await ur_mg.execute(
            trajectory,
            actions=actions,
            tcp="Flange",
            movement_controller=create_movement_controller(exception),
        )
    except Exception as e:
        logger.info(f"Caught expected exception: {e}")

    # time for deceleration
    await asyncio.sleep(1)
    pose = await ur_mg.tcp_pose()
    assert pose.position.x > initial_pose.position.x, "Robot did not move at all."
    assert pose.position.x <= final_pose.position.x, (
        "Robot completed the full movement despite cancelation."
    )

    await asyncio.sleep(2)
    new_pose = await ur_mg.tcp_pose()
    assert pose.position.x == new_pose.position.x, "Robot moved after task was cancelled."


@pytest.mark.asyncio
@pytest.mark.integration
async def test_task_cancelation_when_movement_controller_cancels_we_should_propagate(ur_mg):
    """
    Tests that when user sends a task cancelation from inside a custom movement controller,
    the cancelation is properly propagated and the robot stops moving.
    """
    movement_in_x = 800
    initial_pose = await ur_mg.tcp_pose()
    final_pose = initial_pose @ Pose((movement_in_x, 0, 0))
    actions = [
        # move 800 mm in X direction with 50 mm/s
        # should give us enough time to cancel the task
        lin(final_pose, settings=MotionSettings(tcp_velocity_limit=50))
    ]
    trajectory = await ur_mg.plan(actions=actions, tcp="Flange")

    try:
        await ur_mg.execute(
            trajectory,
            actions=actions,
            tcp="Flange",
            movement_controller=create_movement_controller(asyncio.CancelledError()),
        )
    except asyncio.CancelledError as e:
        logger.info(f"Caught expected exception: {e}")

    # time for deceleration
    await asyncio.sleep(1)
    pose = await ur_mg.tcp_pose()
    assert pose.position.x > initial_pose.position.x, "Robot did not move at all."
    assert pose.position.x <= final_pose.position.x, (
        "Robot completed the full movement despite cancelation."
    )

    await asyncio.sleep(2)
    new_pose = await ur_mg.tcp_pose()
    assert pose.position.x == new_pose.position.x, "Robot moved after task was cancelled."


def process_worker(controller_name: str):
    async def some_function():
        async with Nova() as nova:
            cell = nova.cell()
            controller = await cell.controller(controller_name)
            async with controller[0] as ur_mg:
                movement_in_x = 800
                initial_pose = await ur_mg.tcp_pose()
                final_pose = initial_pose @ Pose((movement_in_x, 0, 0))
                actions = [
                    # move 800 mm in X direction with 50 mm/s
                    # should give us enough time to cancel the task
                    lin(final_pose, settings=MotionSettings(tcp_velocity_limit=50))
                ]
                trajectory = await ur_mg.plan(actions=actions, tcp="Flange")

                await ur_mg.execute(trajectory, actions=actions, tcp="Flange")

    asyncio.run(some_function())


# this test is added for making RAE integration tests easier for v2
# it is not integrated to ci/cd yet
# you can run it with the command below from the top level directory
# python -m pytest -s ./tests/core/test_task_cancelation.py::test_task_cancelation_when_process_is_killed
async def _test_task_cancelation_when_process_is_killed(ur_mg):
    movement_in_x = 800
    initial_pose = await ur_mg.tcp_pose()
    final_pose = initial_pose @ Pose((movement_in_x, 0, 0))

    # is this same as running a process with python script_name.py?
    # probably not
    p = Process(target=process_worker, args=("ur-movement-test"))
    p.start()

    # let the robot move a little
    await asyncio.sleep(5)

    p.kill()

    # time for deceleration
    await asyncio.sleep(1)
    pose = await ur_mg.tcp_pose()
    assert pose.position.x > initial_pose.position.x, "Robot did not move at all."
    assert pose.position.x <= final_pose.position.x, (
        "Robot completed the full movement despite cancelation."
    )

    # wait a little to and check that position not changed
    await asyncio.sleep(2)
    new_pose = await ur_mg.tcp_pose()
    assert pose.position.x == new_pose.position.x, "Robot moved after task was cancelled."
