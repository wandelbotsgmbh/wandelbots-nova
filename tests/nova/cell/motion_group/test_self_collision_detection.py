"""Tests for the self-collision flag of the motion group setup used for planning."""

import pytest

from nova.actions import jnt
from tests.nova.cell.motion_group.test_payload import _build_mock_motion_group

TARGET_JOINTS = (0.1, -1.47, -1.47, 0.1, 0.1, 0.1)


def _planned_setup(mock_api_client):
    request = mock_api_client.trajectory_planning_api.plan_trajectory.call_args.kwargs[
        "plan_trajectory_request"
    ]
    return request.motion_group_setup


@pytest.mark.asyncio
async def test_plan_without_setup_checks_self_collision():
    mg, mock_api_client = _build_mock_motion_group(payloads=None, active_payload=None)

    await mg.plan([jnt(TARGET_JOINTS)], tcp=None)

    assert (
        _planned_setup(mock_api_client).collision_setups["safety"].self_collision_detection is True
    )


@pytest.mark.asyncio
async def test_plan_with_setup_and_disabled_self_collision_keeps_it_disabled():
    mg, mock_api_client = _build_mock_motion_group(payloads=None, active_payload=None)
    setup = await mg.get_setup(self_collision_detection=False)

    await mg.plan([jnt(TARGET_JOINTS)], tcp=None, motion_group_setup=setup)

    assert (
        _planned_setup(mock_api_client).collision_setups["safety"].self_collision_detection is False
    )
