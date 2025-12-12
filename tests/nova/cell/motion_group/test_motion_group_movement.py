import asyncio
from math import pi

import pytest

from nova.actions import jnt, ptp
from nova.api import models
from nova.cell.controllers import virtual_controller
from nova.core.nova import Nova
from nova.exceptions import InitMovementFailed
from nova.types.pose import Pose


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
@pytest.mark.integration
async def test_move_to_current_joint_position(ur_mg):
    """
    Tests that when the robot is in a certain posision and we provide the same joint position as target,
    execution is not stuck and finishes correctly.
    """
    joint_position = await ur_mg.joints()

    async with asyncio.timeout(5):
        await ur_mg.plan_and_execute(actions=[jnt(joint_position)], tcp="Flange")


@pytest.mark.asyncio
@pytest.mark.integration
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

    await ur_mg.plan_and_execute(actions=[jnt(trajectory.joint_positions[0].root)], tcp="Flange")

    async with asyncio.timeout(5):
        await ur_mg.execute(joint_trajectory=trajectory, actions=[], tcp="Flange")


@pytest.mark.asyncio
@pytest.mark.integration
async def test_move_when_start_position_is_different_from_current_position(ur_mg):
    """
    Tests that when the start position of the robot is different than the current position of the robot,
    trajectory execution fails.
    """
    # move the robot a little
    joint_position = await ur_mg.joints()
    current_pose = await ur_mg.tcp_pose("Flange")

    await ur_mg.plan_and_execute(
        start_joint_position=joint_position,
        actions=[ptp(current_pose @ Pose((10, 0, 0, 0, 0, 0)))],
        tcp="Flange",
    )

    # use the old joint position as start position, which is now different than the current position
    with pytest.RaisesGroup(InitMovementFailed):
        await ur_mg.plan_and_execute(
            start_joint_position=joint_position,
            actions=[ptp(current_pose @ Pose((10, 0, 0, 0, 0, 0)))],
            tcp="Flange",
        )
