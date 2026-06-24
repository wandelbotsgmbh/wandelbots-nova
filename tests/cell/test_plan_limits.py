"""Tests for how robot limits flow into plan requests (the "global limits" behaviour).

The two planning paths treat user ``MotionSettings`` differently, and that divergence is the
subtle part worth locking in:

- ``_plan_with_collision_check`` (collision-checked ``plan_trajectory``) keeps
  ``MotionGroupSetup.global_limits`` at the controller maximum and sends the user limits as a
  per-segment ``limits_override`` inside each motion command.
- ``_plan_collision_free`` has no per-segment override, so the user limits are folded directly
  into ``global_limits``.

These tests also assert the no-mutation invariants of the setup helpers, since both paths must
deep-copy the caller's setup before patching it.
"""

from unittest.mock import AsyncMock, MagicMock

import pytest

from nova.actions import cartesian_ptp, collision_free
from nova.api import models
from nova.cell.motion_group import MotionGroup, _with_collision_setup
from nova.core.gateway import ApiGateway
from nova.types.motion_settings import DEFAULT_TCP_VELOCITY_LIMIT, MotionSettings
from nova.utils.motion_group_setup import motion_group_setup_from_motion_group_description

CONTROLLER_MAX_TCP_VELOCITY = 250.0
CONTROLLER_MAX_JOINT_VELOCITY = 3.0
START = (0.0,) * 6


def _controller_max_description() -> models.MotionGroupDescription:
    """A description whose auto (controller-max) limits are deliberately high."""
    return models.MotionGroupDescription(
        motion_group_model=models.MotionGroupModel("UniversalRobots_UR5e"),
        operation_limits=models.OperationLimits(
            auto_limits=models.LimitSet(
                joints=[
                    models.JointLimits(velocity=CONTROLLER_MAX_JOINT_VELOCITY, acceleration=10.0)
                    for _ in range(6)
                ],
                tcp=models.CartesianLimits(velocity=CONTROLLER_MAX_TCP_VELOCITY),
            )
        ),
    )


def _motion_group() -> MotionGroup:
    api_client = MagicMock(spec=ApiGateway)
    api_client.trajectory_planning_api = MagicMock()
    api_client.trajectory_planning_api.plan_trajectory = AsyncMock(return_value=MagicMock())
    api_client.trajectory_planning_api.plan_collision_free = AsyncMock()
    return MotionGroup(
        api_client=api_client, cell="cell", controller_id="ur5e", motion_group_id="0"
    )


def _joint_trajectory() -> models.JointTrajectory:
    return models.JointTrajectory(
        joint_positions=[models.Joints([0.0] * 6), models.Joints([0.1] * 6)],
        times=[0.0, 1.0],
        locations=[models.Location(0.0), models.Location(1.0)],
    )


def _collision_free_response(traj: models.JointTrajectory) -> MagicMock:
    response = MagicMock()
    response.response = traj
    return response


def _collision_setup() -> models.CollisionSetup:
    collider = models.Collider(
        shape=models.Sphere(radius=1.0),
        pose=models.Pose(
            position=models.Vector3d([1.0, 0.0, 0.0]),
            orientation=models.RotationVector([0.0, 0.0, 0.0]),
        ),
    )
    return models.CollisionSetup(colliders=models.ColliderDictionary({"c": collider}))


# ---------------------------------------------------------------------------
# Collision-checked path: user limits -> per-segment override, global stays at max
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_collision_check_sends_user_limits_as_override_and_keeps_global_at_max():
    setup = motion_group_setup_from_motion_group_description(_controller_max_description())
    motion_group = _motion_group()

    await motion_group._plan_with_collision_check(
        actions=[
            cartesian_ptp((1, 2, 3, 0, 0, 0), settings=MotionSettings(tcp_velocity_limit=10.0))
        ],
        tcp="Flange",
        motion_group_setup=setup,
        start_joint_position=START,
    )

    plan_trajectory = motion_group._api_client.trajectory_planning_api.plan_trajectory
    plan_trajectory.assert_awaited_once()
    request = plan_trajectory.await_args.kwargs["plan_trajectory_request"]

    # global_limits is left at the controller maximum...
    assert request.motion_group_setup.global_limits.tcp.velocity == CONTROLLER_MAX_TCP_VELOCITY
    # ...while the user's limit is applied per-segment as a limits_override.
    assert request.motion_commands[0].limits_override is not None
    assert request.motion_commands[0].limits_override.tcp_velocity_limit == 10.0


@pytest.mark.asyncio
async def test_collision_check_does_not_mutate_caller_setup():
    setup = motion_group_setup_from_motion_group_description(_controller_max_description())
    motion_group = _motion_group()

    await motion_group._plan_with_collision_check(
        actions=[
            cartesian_ptp((1, 2, 3, 0, 0, 0), settings=MotionSettings(tcp_velocity_limit=10.0))
        ],
        tcp="Flange",
        motion_group_setup=setup,
        start_joint_position=START,
    )

    # The caller's setup must be untouched: limits unchanged and no "collision-check" key added.
    assert setup.global_limits.tcp.velocity == CONTROLLER_MAX_TCP_VELOCITY
    assert "collision-check" not in setup.collision_setups.root


# ---------------------------------------------------------------------------
# Collision-free path: user limits folded into global_limits (no per-segment override)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_collision_free_folds_user_limits_into_global():
    setup = motion_group_setup_from_motion_group_description(_controller_max_description())
    motion_group = _motion_group()
    traj = _joint_trajectory()
    plan_collision_free = AsyncMock(return_value=_collision_free_response(traj))
    motion_group._api_client.trajectory_planning_api.plan_collision_free = plan_collision_free

    # Joint-space target avoids the inverse-kinematics round-trip, keeping this a focused unit test.
    result = await motion_group._plan_collision_free(
        action=collision_free((0.1,) * 6, settings=MotionSettings(tcp_velocity_limit=10.0)),
        tcp=None,
        motion_group_setup=setup,
        start_joint_position=START,
    )

    assert result is traj
    plan_collision_free.assert_awaited_once()
    request = plan_collision_free.await_args.kwargs["plan_collision_free_request"]
    # No per-segment override exists for collision-free; the user limit lives in global_limits.
    assert request.motion_group_setup.global_limits.tcp.velocity == 10.0


@pytest.mark.asyncio
async def test_collision_free_default_settings_clamp_global_to_default_velocity():
    """Even without explicit user limits, collision_free folds the *default* MotionSettings
    (``tcp_velocity_limit`` defaults to 50 mm/s) into global_limits, clamping the controller max.
    """
    setup = motion_group_setup_from_motion_group_description(_controller_max_description())
    motion_group = _motion_group()
    plan_collision_free = AsyncMock(return_value=_collision_free_response(_joint_trajectory()))
    motion_group._api_client.trajectory_planning_api.plan_collision_free = plan_collision_free

    await motion_group._plan_collision_free(
        action=collision_free((0.1,) * 6),
        tcp=None,
        motion_group_setup=setup,
        start_joint_position=START,
    )

    request = plan_collision_free.await_args.kwargs["plan_collision_free_request"]
    assert request.motion_group_setup.global_limits.tcp.velocity == DEFAULT_TCP_VELOCITY_LIMIT


@pytest.mark.asyncio
async def test_collision_free_does_not_mutate_caller_setup():
    setup = motion_group_setup_from_motion_group_description(_controller_max_description())
    motion_group = _motion_group()
    plan_collision_free = AsyncMock(return_value=_collision_free_response(_joint_trajectory()))
    motion_group._api_client.trajectory_planning_api.plan_collision_free = plan_collision_free

    await motion_group._plan_collision_free(
        action=collision_free((0.1,) * 6, settings=MotionSettings(tcp_velocity_limit=10.0)),
        tcp=None,
        motion_group_setup=setup,
        start_joint_position=START,
    )

    assert setup.global_limits.tcp.velocity == CONTROLLER_MAX_TCP_VELOCITY


# ---------------------------------------------------------------------------
# _with_collision_setup helper
# ---------------------------------------------------------------------------


def _bare_setup() -> models.MotionGroupSetup:
    return models.MotionGroupSetup(
        motion_group_model=models.MotionGroupModel("test"), cycle_time=8, collision_setups=None
    )


def test_with_collision_setup_registers_under_key():
    collision_setup = _collision_setup()

    result = _with_collision_setup(_bare_setup(), "collision-check", collision_setup)

    assert result.collision_setups is not None
    assert result.collision_setups.root["collision-check"] == collision_setup


def test_with_collision_setup_does_not_mutate_caller():
    setup = _bare_setup()

    result = _with_collision_setup(setup, "collision-check", _collision_setup())

    # Input setup is deep-copied: its collision_setups stays None.
    assert setup.collision_setups is None
    assert "collision-check" in result.collision_setups.root


def test_with_collision_setup_none_collision_is_noop():
    result = _with_collision_setup(_bare_setup(), "collision-check", None)

    assert result.collision_setups is not None
    assert "collision-check" not in result.collision_setups.root
