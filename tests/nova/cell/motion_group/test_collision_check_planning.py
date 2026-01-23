from math import pi

import pytest

from nova import Nova
from nova.actions import ptp
from nova.api import models
from nova.cell import virtual_controller
from nova.types.pose import Pose

initial_joint_positions = [pi / 2, -pi / 2, pi / 2, 0, 0, 0]


@pytest.fixture
async def ur_mg():
    """
    Fixture that sets up a robot with virtual controller at a specific start position.

    Yields:
        MotionGroup: Motion group ready for task cancellation tests
    """
    controller_name = "ur-movement-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type=models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
                # create controller API doesn't accept 6 values
                position=[*initial_joint_positions, 0],
            )
        )

        ur = await cell.controller(controller_name)
        async with ur[0] as mg:
            yield mg


@pytest.mark.asyncio
@pytest.mark.integration
async def test_ptp_planning(ur_mg):
    """
    Tests collision-free planning with a joint position as target.
    """
    target_pose = Pose((700, 0, 0, 0, 0, 0))

    trajectory: models.JointTrajectory = await ur_mg.plan(
        start_joint_position=tuple(initial_joint_positions),
        actions=[ptp(target=target_pose)],
        tcp="Flange",
    )

    assert len(trajectory.joint_positions) > 0, (
        "Collision-free planning did not return a valid joint trajectory"
    )
    assert trajectory.joint_positions[0].root == initial_joint_positions, (
        "Initial joint positions do not match start"
    )

    # Verify that the final pose matches the target pose
    found_pose = (await ur_mg.forward_kinematics([trajectory.joint_positions[-1].root], "Flange"))[
        0
    ]
    assert target_pose == found_pose, "Final pose doesn't match target pose"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_ptp_planning_with_collision_setup(ur_mg):
    """
    Tests collision-free planning with a joint position as target.
    """
    plane = models.Collider(
        shape=models.Plane(),
        pose=models.Pose(
            position=models.Vector3d(root=[0, 0, 0]),
            orientation=models.RotationVector(root=[0, 0, 0]),
        ),
    )
    collision_setup = await ur_mg.get_safety_collision_setup("Flange")
    collision_setup.link_chain = await ur_mg.get_default_collision_link_chain()
    collision_setup.colliders = models.ColliderDictionary({"plane": plane})

    target_pose = Pose(700, 0, -10, 0, 0, 0)

    with pytest.raises(Exception):
        await ur_mg.plan(
            start_joint_position=tuple(initial_joint_positions),
            actions=[ptp(target=target_pose, collision_setup=collision_setup)],
            tcp="Flange",
        )
