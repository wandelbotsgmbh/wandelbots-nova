"""Unit tests for trajectory module.

Tests focus on business logic for data transformations,
and motion processing. Does not test rerun library functionality directly.
"""

from unittest.mock import AsyncMock, Mock, patch

import pytest

from nova import api
from nova_rerun_bridge.trajectory import log_motion


class TestLogMotionParameterValidation:
    """Test log_motion parameter validation."""

    @pytest.mark.asyncio
    async def test_accepts_valid_parameters(self):
        """Should accept valid parameters without errors."""
        # Create mock MotionGroupSetup
        mock_setup = Mock()
        mock_setup.dh_parameters = [Mock() for _ in range(6)]
        mock_setup.mounting = Mock()
        mock_setup.motion_group_type = "test_type"
        mock_setup.safety_setup = Mock()
        mock_setup.safety_setup.robot_model_geometries = []
        mock_setup.safety_setup.tcp_geometries = []

        # Create mock MotionGroup
        mock_motion_group = Mock()
        mock_motion_group.id = "test_group"
        mock_motion_group.get_setup = AsyncMock(return_value=mock_setup)
        mock_motion_group.get_model = AsyncMock(return_value="test_model")
        mock_motion_group.forward_kinematics = AsyncMock(return_value=[])

        # Description used by log_motion for DH parameters and safety geometry
        mock_description = Mock()
        mock_description.dh_parameters = [Mock(a=0, d=0, alpha=0, theta=0) for _ in range(2)]
        mock_description.safety_tool_colliders = {}
        mock_description.safety_link_colliders = None
        mock_motion_group.get_description = AsyncMock(return_value=mock_description)

        # Create JointTrajectory
        joint_trajectory = api.models.JointTrajectory(
            joint_positions=[api.models.Joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) for _ in range(5)],
            times=[0.0, 0.1, 0.2, 0.3, 0.4],
            locations=[api.models.Location(0.0) for _ in range(5)],
            tcp="test_tcp",
        )

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch("nova_rerun_bridge.trajectory.extract_link_chain_and_tcp") as mock_extract,
            patch("nova_rerun_bridge.trajectory.RobotVisualizer"),
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
            patch("nova_rerun_bridge.trajectory.log_trajectory") as mock_log_trajectory,
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            # Should not raise exceptions with valid parameters
            await log_motion(
                trajectory=joint_trajectory,
                tcp="test_tcp",
                motion_group=mock_motion_group,
                collision_setups={},
                time_offset=0.0,
            )

            # Verify that the sub-functions were called
            mock_log_trajectory.assert_called_once()

    @pytest.mark.asyncio
    async def test_handles_empty_trajectory(self):
        """Should handle empty trajectory list."""
        # Create mock MotionGroupSetup
        mock_setup = Mock()
        mock_setup.dh_parameters = [Mock() for _ in range(6)]
        mock_setup.mounting = Mock()
        mock_setup.motion_group_type = "test_type"
        mock_setup.safety_setup = Mock()
        mock_setup.safety_setup.robot_model_geometries = []
        mock_setup.safety_setup.tcp_geometries = []

        # Create mock MotionGroup
        mock_motion_group = Mock()
        mock_motion_group.id = "test_group"
        mock_motion_group.get_setup = AsyncMock(return_value=mock_setup)
        mock_motion_group.get_model = AsyncMock(return_value="test_model")
        mock_motion_group.forward_kinematics = AsyncMock(return_value=[])

        # Description used by log_motion for DH parameters and safety geometry
        mock_description = Mock()
        mock_description.dh_parameters = [Mock(a=0, d=0, alpha=0, theta=0) for _ in range(2)]
        mock_description.safety_tool_colliders = {}
        mock_description.safety_link_colliders = None
        mock_motion_group.get_description = AsyncMock(return_value=mock_description)

        # Create empty JointTrajectory
        empty_trajectory = api.models.JointTrajectory(
            joint_positions=[], times=[], locations=[], tcp="test_tcp"
        )

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch("nova_rerun_bridge.trajectory.extract_link_chain_and_tcp") as mock_extract,
            patch("nova_rerun_bridge.trajectory.RobotVisualizer"),
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
            patch("nova_rerun_bridge.trajectory.log_trajectory") as mock_log_trajectory,
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            # Should handle empty trajectory without errors
            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=mock_motion_group,
                collision_setups={},
            )

            # Should still be called but with empty trajectory
            mock_log_trajectory.assert_called_once()

    @pytest.mark.asyncio
    async def test_respects_show_safety_link_chain_parameter(self):
        """Should respect the show_safety_link_chain parameter."""
        # Create mock MotionGroupSetup
        mock_setup = Mock()
        mock_setup.dh_parameters = [Mock() for _ in range(6)]
        mock_setup.mounting = Mock()
        mock_setup.motion_group_type = "test_type"
        mock_setup.safety_setup = Mock()
        mock_setup.safety_setup.robot_model_geometries = []
        mock_setup.safety_setup.tcp_geometries = []

        # Create mock MotionGroup
        mock_motion_group = Mock()
        mock_motion_group.id = "test_group"
        mock_motion_group.get_setup = AsyncMock(return_value=mock_setup)
        mock_motion_group.get_model = AsyncMock(return_value="test_model")
        mock_motion_group.forward_kinematics = AsyncMock(return_value=[])

        # Description used by log_motion for DH parameters and safety geometry
        mock_description = Mock()
        mock_description.dh_parameters = [Mock(a=0, d=0, alpha=0, theta=0) for _ in range(2)]
        mock_description.safety_tool_colliders = {}
        mock_description.safety_link_colliders = None
        mock_motion_group.get_description = AsyncMock(return_value=mock_description)

        # Create empty JointTrajectory
        empty_trajectory = api.models.JointTrajectory(
            joint_positions=[], times=[], locations=[], tcp="test_tcp"
        )

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch("nova_rerun_bridge.trajectory.extract_link_chain_and_tcp") as mock_extract,
            patch("nova_rerun_bridge.trajectory.RobotVisualizer"),
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
            patch("nova_rerun_bridge.trajectory.log_trajectory") as mock_log_trajectory,
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            # Test with show_safety_link_chain=True (default)
            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=mock_motion_group,
                collision_setups={},
                show_safety_link_chain=True,
            )

            # Should be called
            assert mock_log_trajectory.call_count == 1

            # Test with show_safety_link_chain=False
            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=mock_motion_group,
                collision_setups={},
                show_safety_link_chain=False,
            )

            # Should be called again
            assert mock_log_trajectory.call_count == 2
