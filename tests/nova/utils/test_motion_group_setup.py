from unittest.mock import AsyncMock, MagicMock

import pytest

from nova.actions import joint_ptp
from nova.api import models
from nova.cell.motion_group import MotionGroup
from nova.types.motion_settings import DEFAULT_TCP_VELOCITY_LIMIT, MotionSettings
from nova.utils.motion_group_setup import (
    controller_global_limits,
    get_joint_position_limits_from_motion_group_setup,
    motion_group_setup_from_motion_group_description,
    update_motion_group_setup_with_motion_settings,
)


def test_tcp_limits_patching_with_none_setup():
    """Test TCP limits patching when setup has no existing TCP limits."""
    # Arrange
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"), cycle_time=8
    )
    settings = MotionSettings(
        tcp_velocity_limit=100.0,
        tcp_acceleration_limit=200.0,
        tcp_orientation_velocity_limit=1.5,
        tcp_orientation_acceleration_limit=3.0,
    )

    # Act
    motion_group_setup = update_motion_group_setup_with_motion_settings(
        motion_group_setup, settings
    )

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
        velocity=50.0, acceleration=100.0, orientation_velocity=1.0, orientation_acceleration=2.0
    )
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"),
        cycle_time=8,
        global_limits=models.LimitSet(tcp=existing_tcp_limits),
        collision_setups=None,
    )
    settings = MotionSettings(tcp_velocity_limit=200.0, tcp_orientation_acceleration_limit=5.0)

    # Act
    motion_group_setup = update_motion_group_setup_with_motion_settings(
        motion_group_setup, settings
    )

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
        velocity=50.0, acceleration=100.0, orientation_velocity=1.0, orientation_acceleration=2.0
    )
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(tcp=existing_tcp_limits),
    )
    settings = MotionSettings()

    # Act
    motion_group_setup = update_motion_group_setup_with_motion_settings(
        motion_group_setup, settings
    )

    # Updated to default velocity limit
    assert motion_group_setup.global_limits.tcp.velocity == DEFAULT_TCP_VELOCITY_LIMIT

    # unchanged
    assert motion_group_setup.global_limits.tcp.acceleration == 100.0
    assert motion_group_setup.global_limits.tcp.orientation_velocity == 1.0
    assert motion_group_setup.global_limits.tcp.orientation_acceleration == 2.0


def test_joint_limits_replacement_with_none_setup():
    """Test joint limits replacement when setup has no existing joint limits."""
    # Arrange
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test", cycle_time=8, collision_setups=None
    )
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0), joint_acceleration_limits=(4.0, 5.0, 6.0)
    )

    # Act
    motion_group_setup = update_motion_group_setup_with_motion_settings(
        motion_group_setup, settings
    )

    # Assert
    assert motion_group_setup.global_limits.joints is not None
    assert motion_group_setup.global_limits.joints[0].velocity == 1.0
    assert motion_group_setup.global_limits.joints[1].velocity == 2.0
    assert motion_group_setup.global_limits.joints[2].velocity == 3.0

    assert motion_group_setup.global_limits.joints[0].acceleration == 4.0
    assert motion_group_setup.global_limits.joints[1].acceleration == 5.0
    assert motion_group_setup.global_limits.joints[2].acceleration == 6.0


def test_joint_limits_merging_with_existing_setup():
    """Test joint limits merging preserves position limits while updating velocity/acceleration."""
    # Arrange
    existing_joint_limits = [
        models.JointLimits(
            position=models.LimitRange(lower_limit=-3.14, upper_limit=3.14),
            velocity=10.0,
            acceleration=20.0,
        ),
        models.JointLimits(
            position=models.LimitRange(lower_limit=-2.0, upper_limit=2.0),
            velocity=30.0,
            acceleration=40.0,
        ),
        models.JointLimits(
            position=models.LimitRange(lower_limit=-1.5, upper_limit=1.5),
            velocity=50.0,
            acceleration=60.0,
        ),
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0), joint_acceleration_limits=(4.0, 5.0, 6.0)
    )

    # Act
    motion_group_setup = update_motion_group_setup_with_motion_settings(
        motion_group_setup, settings
    )

    # Assert - velocity and acceleration updated, position preserved
    assert len(motion_group_setup.global_limits.joints) == 3

    # Joint 0: verify merge
    assert motion_group_setup.global_limits.joints[0].velocity == 1.0
    assert motion_group_setup.global_limits.joints[0].acceleration == 4.0
    assert motion_group_setup.global_limits.joints[0].position is not None
    assert motion_group_setup.global_limits.joints[0].position.lower_limit == -3.14
    assert motion_group_setup.global_limits.joints[0].position.upper_limit == 3.14

    # Joint 1: verify merge
    assert motion_group_setup.global_limits.joints[1].velocity == 2.0
    assert motion_group_setup.global_limits.joints[1].acceleration == 5.0
    assert motion_group_setup.global_limits.joints[1].position is not None
    assert motion_group_setup.global_limits.joints[1].position.lower_limit == -2.0
    assert motion_group_setup.global_limits.joints[1].position.upper_limit == 2.0

    # Joint 2: verify merge
    assert motion_group_setup.global_limits.joints[2].velocity == 3.0
    assert motion_group_setup.global_limits.joints[2].acceleration == 6.0
    assert motion_group_setup.global_limits.joints[2].position is not None
    assert motion_group_setup.global_limits.joints[2].position.lower_limit == -1.5
    assert motion_group_setup.global_limits.joints[2].position.upper_limit == 1.5


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
    motion_group_setup = update_motion_group_setup_with_motion_settings(
        motion_group_setup, settings
    )

    # Assert - Existing joint limits should be preserved
    assert motion_group_setup.global_limits.joints == existing_joint_limits


def test_joint_limits_raises_error_when_joint_count_differs():
    """Test that ValueError is raised when joint count differs (configuration error)."""
    # Arrange - 2 existing joints
    existing_joint_limits = [
        models.JointLimits(
            position=models.LimitRange(lower_limit=-3.14, upper_limit=3.14),
            velocity=10.0,
            acceleration=20.0,
        ),
        models.JointLimits(
            position=models.LimitRange(lower_limit=-2.0, upper_limit=2.0),
            velocity=30.0,
            acceleration=40.0,
        ),
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    # Settings has 3 joints (different count - configuration error)
    settings = MotionSettings(
        joint_velocity_limits=(1.0, 2.0, 3.0), joint_acceleration_limits=(4.0, 5.0, 6.0)
    )

    # Act & Assert - Should raise ValueError
    with pytest.raises(ValueError, match="Joint count mismatch"):
        update_motion_group_setup_with_motion_settings(motion_group_setup, settings)


def test_joint_limits_merge_with_partial_settings():
    """Test merging when settings only specify velocity (no acceleration)."""
    # Arrange
    existing_joint_limits = [
        models.JointLimits(
            position=models.LimitRange(lower_limit=-3.14, upper_limit=3.14),
            velocity=10.0,
            acceleration=20.0,
        )
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    # Only velocity specified, no acceleration
    settings = MotionSettings(joint_velocity_limits=(5.0,))

    # Act
    motion_group_setup = update_motion_group_setup_with_motion_settings(
        motion_group_setup, settings
    )

    # Assert - velocity updated, acceleration and position preserved
    joint = motion_group_setup.global_limits.joints[0]
    assert joint.velocity == 5.0  # Updated from settings
    assert joint.acceleration == 20.0  # Preserved (settings had None)
    assert joint.position is not None  # Preserved
    assert joint.position.lower_limit == -3.14
    assert joint.position.upper_limit == 3.14


def test_joint_limits_merge_preserves_position_and_torque():
    """Test that position and torque limits are preserved when settings updates velocity/accel.

    This is the primary bug fix test - settings.as_joint_limits() returns JointLimits
    with only velocity and acceleration set, and position/torque should be preserved.
    """
    # Arrange
    existing_joint_limits = [
        models.JointLimits(
            position=models.LimitRange(lower_limit=-1.57, upper_limit=1.57),
            velocity=100.0,
            acceleration=200.0,
            torque=50.0,
        )
    ]
    motion_group_setup = models.MotionGroupSetup(
        motion_group_model="test",
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(joints=existing_joint_limits),
    )
    settings = MotionSettings(joint_velocity_limits=(25.0,), joint_acceleration_limits=(75.0,))

    # Act
    motion_group_setup = update_motion_group_setup_with_motion_settings(
        motion_group_setup, settings
    )

    # Assert - velocity and acceleration updated, position and torque preserved
    joint = motion_group_setup.global_limits.joints[0]
    assert joint.velocity == 25.0  # Updated from settings
    assert joint.acceleration == 75.0  # Updated from settings
    assert joint.position is not None  # Preserved
    assert joint.position.lower_limit == -1.57  # Preserved
    assert joint.position.upper_limit == 1.57  # Preserved
    assert joint.torque == 50.0  # Preserved


_JOINT_TORQUES = [150.0, 140.0, 100.0, 30.0, 30.0, 20.0]


def _description_with_torque() -> models.MotionGroupDescription:
    return models.MotionGroupDescription(
        motion_group_model=models.MotionGroupModel("UniversalRobots_UR5e"),
        operation_limits=models.OperationLimits(
            auto_limits=models.LimitSet(
                joints=[
                    models.JointLimits(velocity=3.0, acceleration=10.0, torque=torque)
                    for torque in _JOINT_TORQUES
                ],
                tcp=models.CartesianLimits(velocity=250.0),
            )
        ),
    )


def test_motion_group_setup_forwards_joint_torque_limits():
    """Torque limits reported by the controller flow into ``global_limits`` for the planner.

    ``torque`` is a per-joint global limit (``LimitSet.joints[i].torque``); it has no per-segment
    override in the v2 API. Controllers that report it (e.g. KUKA) must have it forwarded to
    ``plan_trajectory`` unchanged via ``MotionGroupSetup.global_limits``.
    """
    setup = motion_group_setup_from_motion_group_description(_description_with_torque())

    assert setup.global_limits is not None
    assert setup.global_limits.joints is not None
    assert [joint.torque for joint in setup.global_limits.joints] == _JOINT_TORQUES


@pytest.mark.asyncio
async def test_plan_sends_joint_torque_limits_to_trajectory_planner():
    """End-to-end: building actions and planning forwards torque into the plan_trajectory request.

    Drives ``_plan_with_collision_check`` (the path used by ``plan`` for collision-checked
    motions) with a mocked planning API and asserts that the request actually sent to the
    controller carries the per-joint torque limits unchanged in ``global_limits``.
    """
    setup = motion_group_setup_from_motion_group_description(_description_with_torque())

    plan_trajectory = AsyncMock(return_value=MagicMock())
    api_client = MagicMock()
    api_client.trajectory_planning_api.plan_trajectory = plan_trajectory
    motion_group = MotionGroup(
        api_client=api_client, cell="cell", controller_id="ur10e", motion_group_id="0"
    )

    start = (0.0,) * 6
    await motion_group._plan_with_collision_check(
        actions=[joint_ptp(target=start)],
        tcp="Flange",
        motion_group_setup=setup,
        start_joint_position=start,
    )

    plan_trajectory.assert_awaited_once()
    request = plan_trajectory.await_args.kwargs["plan_trajectory_request"]
    assert request.motion_group_setup.global_limits is not None
    assert request.motion_group_setup.global_limits.joints is not None
    assert [
        joint.torque for joint in request.motion_group_setup.global_limits.joints
    ] == _JOINT_TORQUES


# ---------------------------------------------------------------------------
# controller_global_limits
# ---------------------------------------------------------------------------


def test_controller_global_limits_returns_auto_limits():
    """``auto_limits`` is the single source of truth for ``global_limits``."""
    auto = models.LimitSet(tcp=models.CartesianLimits(velocity=250.0))
    description = models.MotionGroupDescription(
        motion_group_model=models.MotionGroupModel("test"),
        operation_limits=models.OperationLimits(auto_limits=auto),
    )

    assert controller_global_limits(description) is auto


def test_controller_global_limits_requires_auto_limits():
    """Missing controller (auto) limits is a hard error: we cannot derive a planning ceiling."""
    description = models.MotionGroupDescription(
        motion_group_model=models.MotionGroupModel("test"),
        operation_limits=models.OperationLimits(auto_limits=None),
    )

    with pytest.raises(AssertionError):
        controller_global_limits(description)


# ---------------------------------------------------------------------------
# motion_group_setup_from_motion_group_description
# ---------------------------------------------------------------------------


def test_setup_uses_auto_limits_as_global_ceiling():
    """The controller-max (auto) limits become ``global_limits`` verbatim."""
    description = _description_with_torque()

    setup = motion_group_setup_from_motion_group_description(description)

    assert setup.global_limits is description.operation_limits.auto_limits


def test_setup_defaults_cycle_time_to_8_when_missing():
    description = models.MotionGroupDescription(
        motion_group_model=models.MotionGroupModel("test"),
        operation_limits=models.OperationLimits(auto_limits=models.LimitSet()),
        cycle_time=None,
    )

    setup = motion_group_setup_from_motion_group_description(description)

    assert setup.cycle_time == 8


def test_setup_forwards_cycle_time_and_mounting():
    mounting = models.Pose(
        position=models.Vector3d([1.0, 2.0, 3.0]),
        orientation=models.RotationVector([0.0, 0.0, 0.0]),
    )
    description = models.MotionGroupDescription(
        motion_group_model=models.MotionGroupModel("test"),
        operation_limits=models.OperationLimits(auto_limits=models.LimitSet()),
        cycle_time=4,
        mounting=mounting,
    )

    setup = motion_group_setup_from_motion_group_description(description)

    assert setup.cycle_time == 4
    assert setup.mounting == mounting


def test_setup_registers_safety_collision_setup():
    setup = motion_group_setup_from_motion_group_description(_description_with_torque())

    assert setup.collision_setups is not None
    assert "safety" in setup.collision_setups.root


# ---------------------------------------------------------------------------
# get_joint_position_limits_from_motion_group_setup
# ---------------------------------------------------------------------------


def test_joint_position_limits_none_without_global_limits():
    setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"), cycle_time=8
    )

    assert get_joint_position_limits_from_motion_group_setup(setup) is None


def test_joint_position_limits_none_without_joints():
    setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"),
        cycle_time=8,
        global_limits=models.LimitSet(tcp=models.CartesianLimits(velocity=100.0)),
    )

    assert get_joint_position_limits_from_motion_group_setup(setup) is None


def test_joint_position_limits_extracts_ranges():
    joints = [
        models.JointLimits(position=models.LimitRange(lower_limit=-3.14, upper_limit=3.14)),
        models.JointLimits(position=models.LimitRange(lower_limit=-2.0, upper_limit=2.0)),
    ]
    setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"),
        cycle_time=8,
        global_limits=models.LimitSet(joints=joints),
    )

    result = get_joint_position_limits_from_motion_group_setup(setup)

    assert result is not None
    assert result.root == [joint.position for joint in joints]


def test_joint_position_limits_skips_joints_without_position():
    joints = [
        models.JointLimits(position=models.LimitRange(lower_limit=-1.0, upper_limit=1.0)),
        models.JointLimits(velocity=5.0),  # no position limit -> skipped
    ]
    setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"),
        cycle_time=8,
        global_limits=models.LimitSet(joints=joints),
    )

    result = get_joint_position_limits_from_motion_group_setup(setup)

    assert result is not None
    assert result.root == [joints[0].position]


# ---------------------------------------------------------------------------
# update_motion_group_setup_with_motion_settings — no-mutation invariant
# ---------------------------------------------------------------------------


def test_update_does_not_mutate_input_setup():
    """The helper returns a patched copy and must not mutate the caller's setup."""
    setup = models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"),
        cycle_time=8,
        collision_setups=None,
        global_limits=models.LimitSet(tcp=models.CartesianLimits(velocity=250.0)),
    )

    patched = update_motion_group_setup_with_motion_settings(
        setup, MotionSettings(tcp_velocity_limit=10.0)
    )

    assert patched.global_limits.tcp.velocity == 10.0
    # Original is untouched (global_limits is deep-copied internally).
    assert setup.global_limits.tcp.velocity == 250.0
