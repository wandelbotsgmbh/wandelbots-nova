import pytest
from nova import Nova
from nova.cell import virtual_controller
from nova.cell.motion_group import _update_collision_free_motion_group_setup_with_action_settings
from nova.api import models
from nova.types import Pose
from nova.types.motion_settings import DEFAULT_TCP_VELOCITY_LIMIT, MotionSettings
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

    with pytest.raises(Exception):
        await ur_mg.plan(
            start_joint_position=tuple(initial_joint_positions),
            actions=[collision_free(target=Pose(700, 0, -10, 0, 0, 0), colliders={"plane": plane})],
            tcp="Flange",
        )


def test_tcp_limits_patching_with_none_setup():
    """Test TCP limits patching when setup has no existing TCP limits."""
    # Arrange
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"),
        cycle_time=8,
    )
    settings = MotionSettings(
        tcp_velocity_limit=100.0,
        tcp_acceleration_limit=200.0,
        tcp_orientation_velocity_limit=1.5,
        tcp_orientation_acceleration_limit=3.0,
    )

    # Act
    _update_collision_free_motion_group_setup_with_action_settings(motion_group_setup, settings)

    # Assert
    assert motion_group_setup.global_limits.tcp is not None
    assert motion_group_setup.global_limits.tcp.velocity == 100.0
    assert motion_group_setup.global_limits.tcp.acceleration == 200.0
    assert motion_group_setup.global_limits.tcp.orientation_velocity == 1.5
    assert motion_group_setup.global_limits.tcp.orientation_acceleration == 3.0


def test_tcp_limits_patching_with_existing_setup():
    """Test TCP limits patching when setup has existing TCP limits."""
    # Arrange
    existing_tcp_limits = models.CartesianLimits(
        velocity=50.0,
        acceleration=100.0,
        orientation_velocity=1.0,
        orientation_acceleration=2.0,
    )
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"),
        cycle_time=8,
        global_limits=models.LimitSet(tcp=existing_tcp_limits),
        collision_setups=None,
    )
    settings = MotionSettings(
        tcp_velocity_limit=200.0,
        tcp_orientation_acceleration_limit=5.0,
    )

    # Act
    _update_collision_free_motion_group_setup_with_action_settings(motion_group_setup, settings)

    # Assert
    # these are updated
    assert motion_group_setup.global_limits.tcp.velocity == 200.0
    assert motion_group_setup.global_limits.tcp.orientation_acceleration == 5.0

    # there are not
    assert motion_group_setup.global_limits.tcp.acceleration == 100.0
    assert motion_group_setup.global_limits.tcp.orientation_velocity == 1.0


def test_tcp_limits_patching_all_none_in_settings():
    """Test TCP limits patching when all settings values are None."""
    # Arrange
    existing_tcp_limits = models.CartesianLimits(
        velocity=50.0,
        acceleration=100.0,
        orientation_velocity=1.0,
        orientation_acceleration=2.0,
    )
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(tcp=existing_tcp_limits),
    )
    settings = MotionSettings()

    # Act
    _update_collision_free_motion_group_setup_with_action_settings(motion_group_setup, settings)

    # Updated to default vecolity limit
    assert motion_group_setup.global_limits.tcp.velocity == DEFAULT_TCP_VELOCITY_LIMIT

    # unchanged
    assert motion_group_setup.global_limits.tcp.acceleration == 100.0
    assert motion_group_setup.global_limits.tcp.orientation_velocity == 1.0
    assert motion_group_setup.global_limits.tcp.orientation_acceleration == 2.0


def test_joint_limits_replacement_with_none_setup():
    """Test joint limits replacement when setup has no existing joint limits."""
    # Arrange
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
    )
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0),
        joint_acceleration_limits=(4.0, 5.0, 6.0),
    )

    # Act
    _update_collision_free_motion_group_setup_with_action_settings(motion_group_setup, settings)

    # Assert
    assert motion_group_setup.global_limits.joints is not None
    assert motion_group_setup.global_limits.joints[0].velocity == 1.0
    assert motion_group_setup.global_limits.joints[1].velocity == 2.0
    assert motion_group_setup.global_limits.joints[2].velocity == 3.0

    assert motion_group_setup.global_limits.joints[0].acceleration == 4.0
    assert motion_group_setup.global_limits.joints[1].acceleration == 5.0
    assert motion_group_setup.global_limits.joints[2].acceleration == 6.0


def test_joint_limits_replacement_with_existing_setup():
    """Test joint limits replacement when setup has existing joint limits."""
    # Arrange
    existing_joint_limits = [
        models.JointLimits(velocity=10.0, acceleration=20.0),
        models.JointLimits(velocity=30.0, acceleration=40.0),
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0),
        joint_acceleration_limits=(4.0, 5.0, 6.0),
    )

    # Act
    _update_collision_free_motion_group_setup_with_action_settings(motion_group_setup, settings)

    # Assert - Entire joint limits list should be replaced
    assert motion_group_setup.global_limits.joints is not None

    assert motion_group_setup.global_limits.joints[0].velocity == 1.0
    assert motion_group_setup.global_limits.joints[1].velocity == 2.0
    assert motion_group_setup.global_limits.joints[2].velocity == 3.0

    assert motion_group_setup.global_limits.joints[0].acceleration == 4.0
    assert motion_group_setup.global_limits.joints[1].acceleration == 5.0
    assert motion_group_setup.global_limits.joints[2].acceleration == 6.0


def test_no_joint_limits_in_settings():
    """Test that existing joint limits are preserved when settings has no joint limits."""
    # Arrange
    existing_joint_limits = [
        models.JointLimits(velocity=10.0, acceleration=20.0),
        models.JointLimits(velocity=30.0, acceleration=40.0),
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    settings = MotionSettings()

    # Act
    _update_collision_free_motion_group_setup_with_action_settings(motion_group_setup, settings)

    # Assert - Existing joint limits should be preserved
    assert motion_group_setup.global_limits.joints == existing_joint_limits
