"""Tests for optional TCP support in plan/execute.

Motion groups without a TCP should be able to plan and execute joint-space-only
workflows (jnt, wait, io_write) without providing a TCP. Cartesian actions
(lin, ptp, cir, collision_free with Pose target) must raise a clear error
when TCP is None.
"""

from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.actions import cir, collision_free, jnt, lin, ptp, wait
from nova.cell.motion_group import MotionGroup
from nova.core.gateway import ApiGateway
from nova.types import Pose


@pytest.fixture
def mock_motion_group():
    """Create a MotionGroup instance with mocked internals for TCP-optional tests."""
    mock_api_client = MagicMock(spec=ApiGateway)

    # Mock get_current_motion_group_state (used by joints())
    mock_state = MagicMock()
    mock_state.joint_position = [0.0, -1.57, -1.57, 0.0, 0.0, 0.0]
    mock_state.tcp_pose = api.models.Pose(
        position=api.models.Vector3d([0.0, 0.0, 0.0]),
        orientation=api.models.RotationVector([0.0, 0.0, 0.0]),
    )
    mock_state.tcp = None
    mock_api_client.motion_group_api = MagicMock()
    mock_api_client.motion_group_api.get_current_motion_group_state = AsyncMock(
        return_value=mock_state
    )

    # Mock get_motion_group_description (used by get_setup)
    mock_description = MagicMock()
    mock_description.motion_group_model = api.models.MotionGroupModel("test-model")
    mock_description.cycle_time = 8
    mock_description.mounting = None
    mock_description.tcps = None  # No TCPs configured
    mock_description.operation_limits = MagicMock()
    mock_description.operation_limits.auto_limits = api.models.LimitSet(joints=[])
    mock_description.safety_tool_colliders = None
    mock_description.safety_link_colliders = None
    mock_description.safety_zones = None
    mock_api_client.motion_group_api.get_motion_group_description = AsyncMock(
        return_value=mock_description
    )

    # Mock trajectory planning API
    mock_plan_response = MagicMock()
    mock_plan_response.response = api.models.JointTrajectory(
        joint_positions=[
            api.models.Joints([0.0, -1.57, -1.57, 0.0, 0.0, 0.0]),
            api.models.Joints([0.1, -1.47, -1.47, 0.1, 0.1, 0.1]),
        ],
        times=[0.0, 1.0],
        locations=[api.models.Location(0.0), api.models.Location(1.0)],
    )
    mock_api_client.trajectory_planning_api = MagicMock()
    mock_api_client.trajectory_planning_api.plan_trajectory = AsyncMock(
        return_value=mock_plan_response
    )

    # Mock trajectory caching API (used by _load_planned_motion in execute)
    mock_add_trajectory_response = MagicMock()
    mock_add_trajectory_response.error = None
    mock_add_trajectory_response.trajectory = "test-trajectory-id"
    mock_api_client.trajectory_caching_api = MagicMock()
    mock_api_client.trajectory_caching_api.add_trajectory = AsyncMock(
        return_value=mock_add_trajectory_response
    )

    return MotionGroup(
        api_client=mock_api_client,
        cell="test_cell",
        controller_id="test-controller",
        motion_group_id="0@test-controller",
    )


# ---------------------------------------------------------------------------
# Validation: tcp=None rejects cartesian actions
# ---------------------------------------------------------------------------


class TestPlanWithoutTcpRejectsCartesianActions:
    """tcp=None must raise ValueError for any action that requires a TCP."""

    @pytest.mark.asyncio
    async def test_rejects_lin(self, mock_motion_group):
        with pytest.raises(ValueError, match="TCP is required for cartesian motion actions"):
            await mock_motion_group.plan([lin((0, 0, 0, 0, 0, 0))])

    @pytest.mark.asyncio
    async def test_rejects_ptp(self, mock_motion_group):
        with pytest.raises(ValueError, match="TCP is required for cartesian motion actions"):
            await mock_motion_group.plan([ptp((0, 0, 0, 0, 0, 0))])

    @pytest.mark.asyncio
    async def test_rejects_cir(self, mock_motion_group):
        with pytest.raises(ValueError, match="TCP is required for cartesian motion actions"):
            await mock_motion_group.plan([cir((0, 0, 0, 0, 0, 0), intermediate=(1, 1, 1, 0, 0, 0))])

    @pytest.mark.asyncio
    async def test_rejects_collision_free_with_pose_target(self, mock_motion_group):
        with pytest.raises(ValueError, match="TCP is required for collision_free"):
            await mock_motion_group.plan([collision_free(Pose((0, 0, 0, 0, 0, 0)))])

    @pytest.mark.asyncio
    async def test_rejects_mixed_jnt_and_lin(self, mock_motion_group):
        """Even if some actions are joint-space, a single cartesian action should fail."""
        with pytest.raises(ValueError, match="TCP is required for cartesian motion actions"):
            await mock_motion_group.plan(
                [jnt((0.0, -1.57, -1.57, 0.0, 0.0, 0.0)), lin((0, 0, 0, 0, 0, 0))]
            )


# ---------------------------------------------------------------------------
# Happy path: tcp=None works for joint-space actions
# ---------------------------------------------------------------------------


class TestPlanWithoutTcpAcceptsJointActions:
    """tcp=None must succeed for joint-space-only workflows."""

    @pytest.mark.asyncio
    async def test_plan_jnt_without_tcp(self, mock_motion_group):
        trajectory = await mock_motion_group.plan([jnt((0.1, -1.47, -1.47, 0.1, 0.1, 0.1))])
        assert trajectory is not None
        assert len(trajectory.joint_positions) > 0

    @pytest.mark.asyncio
    async def test_plan_wait_without_tcp(self, mock_motion_group):
        trajectory = await mock_motion_group.plan([wait(1.0)])
        assert trajectory is not None
        # Wait generates a trajectory with same joint positions at each timestep
        assert len(trajectory.joint_positions) >= 2

    @pytest.mark.asyncio
    async def test_plan_jnt_and_wait_without_tcp(self, mock_motion_group):
        trajectory = await mock_motion_group.plan(
            [jnt((0.1, -1.47, -1.47, 0.1, 0.1, 0.1)), wait(0.5)]
        )
        assert trajectory is not None
        assert len(trajectory.joint_positions) > 0

    @pytest.mark.asyncio
    async def test_plan_collision_free_with_joint_target_without_tcp(self, mock_motion_group):
        """collision_free with a tuple (joint) target should not require TCP."""
        joint_target = (0.1, -1.47, -1.47, 0.1, 0.1, 0.1)

        mock_plan_cf_response = MagicMock()
        mock_plan_cf_response.response = api.models.JointTrajectory(
            joint_positions=[
                api.models.Joints([0.0, -1.57, -1.57, 0.0, 0.0, 0.0]),
                api.models.Joints(list(joint_target)),
            ],
            times=[0.0, 1.0],
            locations=[api.models.Location(0.0), api.models.Location(1.0)],
        )
        mock_motion_group._api_client.trajectory_planning_api.plan_collision_free = AsyncMock(
            return_value=mock_plan_cf_response
        )

        trajectory = await mock_motion_group.plan([collision_free(joint_target)])
        assert trajectory is not None


# ---------------------------------------------------------------------------
# Execution: tcp=None flows through correctly
# ---------------------------------------------------------------------------


class TestLoadPlannedMotionWithNoneTcp:
    """_load_planned_motion should pass tcp=None to AddTrajectoryRequest."""

    @pytest.mark.asyncio
    async def test_load_planned_motion_passes_none_tcp(self, mock_motion_group):
        joint_trajectory = api.models.JointTrajectory(
            joint_positions=[api.models.Joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
            times=[0.0],
            locations=[api.models.Location(0.0)],
        )

        await mock_motion_group._load_planned_motion(joint_trajectory, tcp=None)

        call_kwargs = mock_motion_group._api_client.trajectory_caching_api.add_trajectory.call_args
        request = call_kwargs.kwargs["add_trajectory_request"]
        assert request.tcp is None


# ---------------------------------------------------------------------------
# Edge case: empty actions
# ---------------------------------------------------------------------------


class TestPlanWithoutTcpEdgeCases:
    @pytest.mark.asyncio
    async def test_empty_actions_raises_before_tcp_check(self, mock_motion_group):
        with pytest.raises(ValueError, match="No actions provided"):
            await mock_motion_group.plan([])

    @pytest.mark.asyncio
    async def test_plan_jnt_with_explicit_tcp_still_works(self, mock_motion_group):
        """Backward compatibility: passing tcp explicitly should still work."""
        # Need to mock tcps for get_setup when tcp is provided
        mock_desc = (
            mock_motion_group._api_client.motion_group_api.get_motion_group_description.return_value
        )
        mock_desc.tcps = {
            "Flange": MagicMock(
                pose=api.models.Pose(
                    position=api.models.Vector3d([0, 0, 0]),
                    orientation=api.models.RotationVector([0, 0, 0]),
                ),
                name="Flange",
            )
        }

        trajectory = await mock_motion_group.plan(
            [jnt((0.1, -1.47, -1.47, 0.1, 0.1, 0.1))], tcp="Flange"
        )
        assert trajectory is not None
