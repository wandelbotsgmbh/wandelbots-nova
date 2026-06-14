from unittest.mock import AsyncMock, MagicMock

import pytest

from nova.actions import joint_ptp
from nova.api import models
from nova.cell.motion_group import MotionGroup
from nova.utils.collision_setup import \
    motion_group_setup_from_motion_group_description

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
