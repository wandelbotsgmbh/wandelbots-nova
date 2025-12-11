from math import pi

import pytest

from nova import Nova
from nova.api import models
from nova.cell import virtual_controller
from nova.cell.motion_group import _find_shortest_distance
from nova.types import Pose


@pytest.fixture
async def ur_mg():
    """
    Fixture that sets up a robot with virtual controller at a specific start position.

    Yields:
        MotionGroup: Motion group ready for task cancellation tests
    """
    initial_joint_positions = [1.8294, -1.4618, -1.8644, -1.1851, 1.5188, 0.2529, 0.0]
    controller_name = "ur-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type=models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
                position=initial_joint_positions,
            )
        )

        ur = await cell.controller(controller_name)
        async with ur[0] as mg:
            yield mg


def test_find_shortest_distance_returns_closest_solution():
    start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    solutions = [
        (3.0, 4.0, 0.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 0.0, 0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ]

    assert _find_shortest_distance(start, solutions) == solutions[1]


@pytest.mark.asyncio
@pytest.mark.integration
async def test_inverse_kinematics_not_reachable_pose(ur_mg):
    """
    Test a pose that is beyond the robot's reach.
    Verify no solution found.
    """
    solutions = await ur_mg._inverse_kinematics(
        poses=[
            Pose(10000, 0, 10000, 0, 0, 0)  # An unreachable pose
        ],
        tcp="Flange",
    )

    assert len(solutions) == 1, (
        "Inverse kinematics did not return a solution for the unreachable pose"
    )
    assert len(solutions[0]) == 0, (
        "Inverse kinematics should return no solutions for unreachable pose"
    )


@pytest.mark.asyncio
@pytest.mark.integration
async def test_inverse_kinematics_reachable_pose(ur_mg):
    """
    Test a pose that is within the robot's reach.
    """
    solutions = await ur_mg._inverse_kinematics(poses=[Pose(700, 0, 700, 0, 0, 0)], tcp="Flange")

    assert len(solutions) == 1, (
        "Inverse kinematics did not return a solution for the unreachable pose"
    )
    assert len(solutions[0]) == 8, (
        "Inverse kinematics should return no solutions for unreachable pose"
    )
    assert len(solutions[0][0]) == 6, "Inverse kinematics solution does not have 6 joint values"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_inverse_kinematics_mixed_pose_list(ur_mg):
    """
    Test sending a list of poses for IK calculation.
    Verify that reachable poses return solutions and unreachable poses do not.
    """
    solutions = await ur_mg._inverse_kinematics(
        poses=[Pose(700, 0, 700, 0, 0, 0), Pose(10000, 0, 10000, 0, 0, 0)], tcp="Flange"
    )

    assert len(solutions) == 2, "Inverse kinematics did not return solutions for all poses"
    assert len(solutions[0]) == 8, "Inverse kinematics should return solutions for reachable pose"
    assert len(solutions[1]) == 0, (
        "Inverse kinematics should return no solutions for unreachable pose"
    )

    for solution in solutions[0]:
        assert len(solution) == 6, "Inverse kinematics solution does not have 6 joint values"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_inverse_kinematics_unreachable_pose_due_to_collision_setup(ur_mg):
    """
    Cut solution space with a plane collider and test that IK respects the collision setup.
    """
    plane = models.Collider(
        shape=models.Plane(),
        pose=models.Pose(
            position=models.Vector3d(root=[0, 0, 0]),
            orientation=models.RotationVector(root=[0, 0, 0]),
        ),
    )
    setup = await ur_mg.get_setup("Flange")

    # create a collsion setup
    # UR doesn't include safety link chain
    collision_setup = await ur_mg.get_safety_collision_setup("Flange")
    collision_setup.colliders = {"plane": plane}
    collision_setup.link_chain = await ur_mg.get_default_collision_link_chain()

    setup.collision_setups.root = {"test": collision_setup}

    # this orientation is important
    # with this orientation, the body of the end effector stays on top of the plane
    # if you change the orientation, IK will find no solution
    orientation = ((120 / 360) * 2 * pi, (-135 / 360) * 2 * pi, 0)
    solutions = await ur_mg._inverse_kinematics(
        poses=[Pose(700, 0, 1, *orientation)], tcp="Flange", motion_group_setup=setup
    )

    assert len(solutions) == 1, "Inverse kinematics did not return solutions for all poses"
    assert len(solutions[0]) != 0, "Inverse kinametics found solutions that should be in collision"

    # we cut the bottom part of the solution space with the plane
    assert len(solutions[0]) == 4, "Inverse kinametics found solutions that should be in collision"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_inverse_kinematics_unreachable_pose_due_to_collision_setup_2(ur_mg):
    """
    Cut solution space with a box collider and test that IK respects the collision setup.
    """
    box = models.Collider(
        shape=models.Box(size_x=100, size_y=100, size_z=100, box_type=models.BoxType.FULL),
        pose=models.Pose(
            position=models.Vector3d(root=[700, 0, 700]),
            orientation=models.RotationVector(root=[0, 0, 0]),
        ),
    )

    setup = await ur_mg.get_setup("Flange")

    # create a collsion setup
    # UR doesn't include safety link chain
    collision_setup = await ur_mg.get_safety_collision_setup("Flange")
    collision_setup.colliders = {"box": box}
    collision_setup.link_chain = await ur_mg.get_default_collision_link_chain()

    setup.collision_setups.root = {"test": collision_setup}

    solutions = await ur_mg._inverse_kinematics(
        poses=[Pose(700, 0, 700, 0, 0, 0)], tcp="Flange", motion_group_setup=setup
    )

    assert len(solutions) == 1, "Inverse kinematics did not return solutions for all poses"
    assert len(solutions[0]) == 0, "Inverse kinametics found solutions that should be in collision"
