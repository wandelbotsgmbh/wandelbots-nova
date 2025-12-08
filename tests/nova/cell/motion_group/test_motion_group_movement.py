import asyncio
from math import pi

import pytest

from nova.actions import jnt
from nova.api import models
from nova.cell.controllers import virtual_controller
from nova.core.nova import Nova


@pytest.fixture
async def ur_mg():
    """
    Fixture that sets up a robot with virtual controller at a specific start position.
    """
    controller_name = "ur-movement-test-2"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type=models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
                position=[0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0],
            )
        )

        ur = await cell.controller(controller_name)
        async with ur[0] as mg:
            yield mg


@pytest.mark.asyncio
async def test_move_to_current_joint_position(ur_mg):
    """
    Tests that when the robot is in a certain posision and we provide the same joint position as target,
    execution is not stuck.

    """
    joint_position = await ur_mg.joints()

    async with asyncio.timeout(5):
        await ur_mg.plan_and_execute(actions=[jnt(joint_position)], tcp="Flange")


## Test moving robot when it is not in the start position
@pytest.mark.asyncio
async def test_move_to_very_similar_joint_position(ur_mg):
    """
    Tessts that two very similar float numbers for joints are handled correctly and trajectory is executed.
    """
    trajectory = models.JointTrajectory(
        joint_positions=[
            models.Joints(
                [
                    1.8293999433517456,
                    -1.4617999792099,
                    -1.864400029182434,
                    -1.1850999593734741,
                    1.5188000202178955,
                    0.25290000438690186,
                ]
            ),
            models.Joints([1.8294, -1.4618, -1.8644, -1.1851, 1.5188, 0.2529]),
        ],
        locations=[0.0, 1.0],
        times=[0, 0.008],
    )

    # move to the start of the trajectory
    await ur_mg.plan_and_execute(actions=[jnt(trajectory.joint_positions[0].root)], tcp="Flange")
    ## Seems to not work
    # async with Nova() as nova:
    #     await nova.api.virtual_controller_api.set_motion_group_state(
    #         cell="cell",
    #         motion_group_joints=models.MotionGroupJoints(positions=trajectory.joint_positions[0].root),
    #         motion_group=ur_mg.id,
    #         controller=ur_mg._controller_id,
    #     )

    async with asyncio.timeout(5):
        await ur_mg.execute(joint_trajectory=trajectory, actions=[], tcp="Flange")


async def test_multiple_movements_back_to_back(ur_mg):
    """
    Tests that multiple movements back to back are handled correctly.
    """
    joint_position_1 = models.Joints([1.0, -1.0, -1.0, -1.0, 1.0, 0.0])
    joint_position_2 = models.Joints([1.5, -1.5, -1.5, -1.5, 1.5, 0.0])

    async with asyncio.timeout(10):
        await ur_mg.plan_and_execute(actions=[jnt(joint_position_1.root)], tcp="Flange")
        await ur_mg.plan_and_execute(actions=[jnt(joint_position_2.root)], tcp="Flange")
        await ur_mg.plan_and_execute(actions=[jnt(joint_position_1.root)], tcp="Flange")
