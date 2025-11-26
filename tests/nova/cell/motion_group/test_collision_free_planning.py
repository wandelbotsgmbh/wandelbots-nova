import pytest
from nova import Nova
from nova.cell import virtual_controller
from nova.api import models
from nova.types import Pose
from nova.actions import collision_free
from math import pi

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
async def test_collision_free_planning_with_joint_position_as_target(ur_mg):
    """
    Tests collision-free planning with a joint position as target.
    """
    trajectory: models.JointTrajectory = await ur_mg.plan(
        start_joint_position=tuple(initial_joint_positions),
        actions=[collision_free(target=(pi / 4, -pi / 2, pi / 2, 0, 0, 0))],
        tcp="Flange",
    )

    assert len(trajectory.joint_positions) > 0, (
        "Collision-free planning did not return a valid joint trajectory"
    )
    assert trajectory.joint_positions[0].root == initial_joint_positions, (
        "Initial joint positions do not match start"
    )
    assert trajectory.joint_positions[-1].root == [pi / 4, -pi / 2, pi / 2, 0, 0, 0], (
        "Final joint positions do not match target" @ pytest.mark.asyncio
    )


@pytest.mark.asyncio
async def test_collision_free_planning_with_pose_as_target(ur_mg):
    """
    Tests collision-free planning with a pose as target.
    """
    target_as_joints = (pi / 4, -pi / 2, pi / 2, 0, 0, 0)
    target_as_pose = (
        await ur_mg.forward_kinematics(joints=[list(target_as_joints)], tcp="Flange")
    )[0]

    trajectory: models.JointTrajectory = await ur_mg.plan(
        start_joint_position=tuple(initial_joint_positions),
        actions=[collision_free(target=target_as_pose)],
        tcp="Flange",
    )

    assert len(trajectory.joint_positions) > 0, (
        "Collision-free planning did not return a valid joint trajectory"
    )
    assert trajectory.joint_positions[0].root == initial_joint_positions, (
        "Initial joint positions do not match start"
    )

    # make sure the final joint position leas to the some pose as target_as_pose
    last_joint_position = trajectory.joint_positions[-1].root
    last_joint_position_as_pose = (
        await ur_mg.forward_kinematics(joints=[list(last_joint_position)], tcp="Flange")
    )[0]

    assert last_joint_position_as_pose == target_as_pose, (
        "Final joint position as pose does not match target pose"
    )


@pytest.mark.asyncio
async def test_collision_free_planning_finds_no_solution(ur_mg):
    """
    Tests that collision-free planning correctly identifies when no solution is possible.
    """
    plane = models.Collider(
        shape=models.Plane(),
        pose=models.Pose(
            position=models.Vector3d(root=[0, 0, 0]),
            orientation=models.RotationVector(root=[0, 0, 0]),
        ),
    )
    collision_setup = models.CollisionSetup(colliders={"plane": plane})

    with pytest.raises(Exception):
        await ur_mg.plan(
            start_joint_position=tuple(initial_joint_positions),
            actions=[
                collision_free(target=Pose(700, 0, -10, 0, 0, 0), collision_setup=collision_setup)
            ],
            tcp="Flange",
        )
