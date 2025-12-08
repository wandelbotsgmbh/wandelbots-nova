from math import pi

import pytest

from nova import Nova, api
from nova.actions import collision_free
from nova.cell import MotionGroup, virtual_controller
from nova.types import Pose

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
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
                # create controller API doesn't accept 6 values
                position=[*initial_joint_positions, 0],
            )
        )

        ur = await cell.controller(controller_name)
        async with ur[0] as mg:
            yield mg


@pytest.mark.asyncio
@pytest.mark.integration
async def test_collision_free_planning_with_joint_position_as_target(ur_mg):
    """
    Tests collision-free planning with a joint position as target.
    """
    trajectory = await ur_mg.plan(
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
        "Final joint positions do not match target"
    )


@pytest.mark.asyncio
@pytest.mark.integration
async def test_collision_free_planning_with_pose_as_target(ur_mg):
    """
    Tests collision-free planning with a pose as target.
    """
    target_as_joints = (pi / 4, -pi / 2, pi / 2, 0, 0, 0)
    target_as_pose = (
        await ur_mg.forward_kinematics(joints=[list(target_as_joints)], tcp="Flange")
    )[0]

    trajectory = await ur_mg.plan(
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
@pytest.mark.integration
async def test_collision_free_planning_finds_no_solution_pose_as_target(ur_mg: MotionGroup):
    """
    Tests that collision-free planning correctly identifies when no solution is possible.
    """
    plane = api.models.Collider(
        shape=api.models.Plane(),
        pose=api.models.Pose(
            position=api.models.Vector3d(root=[0, 0, 0]),
            orientation=api.models.RotationVector(root=[0, 0, 0]),
        ),
    )
    collision_setup = await ur_mg.get_safety_collision_setup("Flange")
    collision_setup.link_chain = await ur_mg.get_default_collision_link_chain()
    collision_setup.colliders = api.models.ColliderDictionary({"plane": plane})

    with pytest.raises(Exception):
        await ur_mg.plan(
            start_joint_position=tuple(initial_joint_positions),
            actions=[
                collision_free(target=Pose(700, 0, -10, 0, 0, 0), collision_setup=collision_setup)
            ],
            tcp="Flange",
        )


@pytest.mark.asyncio
@pytest.mark.integration
async def test_collision_free_planning_finds_no_solution_joints_as_target(ur_mg: MotionGroup):
    """
    Tests that collision-free planning correctly identifies when no solution is possible.
    """
    # calcualte IK to use in collision free planning
    target_pose = Pose(700, 0, -10, 0, 0, 0)
    ik_solutions = await ur_mg._inverse_kinematics([target_pose], tcp="Flange")
    target_as_joints = ik_solutions[0][0]

    # create a collision scene to make the point unreachable
    plane = api.models.Collider(
        shape=api.models.Plane(),
        pose=api.models.Pose(
            position=api.models.Vector3d(root=[0, 0, 0]),
            orientation=api.models.RotationVector(root=[0, 0, 0]),
        ),
    )
    collision_setup = await ur_mg.get_safety_collision_setup("Flange")
    collision_setup.link_chain = await ur_mg.get_default_collision_link_chain()
    collision_setup.colliders = api.models.ColliderDictionary({"plane": plane})

    # plan and expect no solution
    # TODO
    # current API returns 422 when there is a collision
    # this is hard to build login on top, check with core team for a different error mechanism
    with pytest.raises(Exception):
        await ur_mg.plan(
            start_joint_position=tuple(initial_joint_positions),
            actions=[collision_free(target=target_as_joints, collision_setup=collision_setup)],
            tcp="Flange",
        )
