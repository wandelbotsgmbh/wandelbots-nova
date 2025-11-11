from nova import Nova
from nova.actions import lin, ptp, jnt
from nova.cell.controllers import virtual_controller
from nova.core.motion_group import MotionGroup
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.api import models
from nova.logging import logger
import asyncio
import pytest

# Testing questions:
# 1. 
# is hardcoding a scene wiwht the positions ok to trigger certain flows?
# e.g move to the start position

# 2. integration test looks move useful than unit test? 
# how to really understand if the robot is moving or not?
# should I use the stream device state to see if any movement is happening?
# 

# 3. cases to test
# when the user's movement controller throws an exception
# cancelation with asyncio task interface
# throwing exception while consuming the state

# 4. The fix looks straightforward but testing all the movement realted API with integration test is quite time consuming
# especiall if we combine the behaviour of the web sockets
# how should we approach this? since it is an important bug to fix

# we have 3 APIs
# plan_and_execute
# stream_execute
# execute
# one user function



# if I use a linear movement or p2p than I can estimate how long the test will take
# using the movement settings as well
# use low velocity -> 

# only slow movements, accelaration and decelaration is less problem when I do slow velocity
# motion_group.stop
# time related bug can be visible with faster tests

# don't put too much stuff on v1 realted stuff


# use two tasks one for monitor and one for execution

@pytest.fixture
async def ur_mg():
    """
    Fixture that sets up a robot with virtual controller at a specific start position.
    
    Yields:
        MotionGroup: Motion group ready for task cancellation tests
    """
    initial_joint_positions = [1.8294, -1.4618, -1.8644, -1.1851, 1.5188, 0.2529, 0.0]
    controller_name = "ur-movement-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type=models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
                position=initial_joint_positions
            )
        )

        ur = await cell.controller(controller_name)
        async with ur[0] as mg:

            yield mg

            print("running clean up")
            # Move back to the initial position
            await mg.plan_and_execute(
                actions=[
                    jnt(initial_joint_positions[:6], settings=MotionSettings(tcp_velocity_limit=250)),
                ],
                tcp="Flange"
            )
            



@pytest.mark.asyncio
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
        lin(final_pose, settings=MotionSettings(tcp_velocity_limit=50)),
    ]
    trajectory = await ur_mg.plan(actions=actions, tcp="Flange")
    
    movement_task = asyncio.create_task(
        ur_mg.execute(trajectory, actions=actions, tcp="Flange")
    )

    await asyncio.sleep(2)

    try:
        result = movement_task.cancel()
        logger.info(f"Cancelation sent: {result}")
        await movement_task
    except asyncio.CancelledError as e:
        logger.info(f"Task was cancelled: {e}")

    # assert that the robot moved but did not complete the full movement
    # note if you remove asyncio.sleep(1) here the test will fail probably 
    # because the robot will decelerate to stop and the final position will be different
    await asyncio.sleep(1)    
    pose = await ur_mg.tcp_pose()
    assert pose.position.x > initial_pose.position.x, "Robot did not move at all."
    assert pose.position.x < final_pose.position.x, "Robot completed the full movement despite cancelation."


    await asyncio.sleep(2)
    new_pose = await ur_mg.tcp_pose()
    assert pose.position.x == new_pose.position.x, "Robot moved after task was cancelled."
    

@pytest.mark.asyncio
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
        lin(final_pose, settings=MotionSettings(tcp_velocity_limit=50)),
    ]
    trajectory = await ur_mg.plan(actions=actions, tcp="Flange")

    try:
        logger.warning("Starting stream_execute...")
        number_of_states_to_consume = 100
        async for state in ur_mg.stream_execute(trajectory, "Flange", actions=actions):
            logger.warning("[Test-stream_execute] Consuming state...")
            number_of_states_to_consume -= 1
            if number_of_states_to_consume == 0:
                raise Exception("Intentional exception to test movement stop on exception.")
    except Exception as e:
        logger.info(f"Caught expected exception: {e}")
    


    # assert that the robot moved but did not complete the full movement
    # note if you remove asyncio.sleep(1) here the test will fail probably 
    # because the robot will decelerate to stop and the final position will be different
    await asyncio.sleep(1)    
    pose = await ur_mg.tcp_pose()
    assert pose.position.x > initial_pose.position.x, "Robot did not move at all."
    assert pose.position.x <= final_pose.position.x, "Robot completed the full movement despite cancelation."


    await asyncio.sleep(2)
    new_pose = await ur_mg.tcp_pose()
    assert pose.position.x == new_pose.position.x, "Robot moved after task was cancelled."