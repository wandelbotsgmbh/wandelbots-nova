"""Unit tests for trajectory module.

Tests focus on business logic for data transformations,
and motion processing. Does not test rerun library functionality directly.
"""

from unittest.mock import AsyncMock, Mock, patch

import pytest

from nova import api
from nova_rerun_bridge.trajectory import log_motion


class TestVisualizerCaching:
    """Test visualizer caching behavior."""

    @pytest.fixture
    def mock_motion_group(self):
        """Create a mock motion group for testing."""
        mock_setup = Mock()
        mock_setup.dh_parameters = [Mock() for _ in range(6)]
        mock_setup.mounting = Mock()

        mock_description = Mock()
        mock_description.dh_parameters = [Mock(a=0, d=0, alpha=0, theta=0) for _ in range(2)]
        mock_description.safety_tool_colliders = {}
        mock_description.safety_link_colliders = None

        mock_motion_group = Mock()
        mock_motion_group.id = "test_group"
        mock_motion_group.get_setup = AsyncMock(return_value=mock_setup)
        mock_motion_group.get_model = AsyncMock(return_value="test_model")
        mock_motion_group.get_description = AsyncMock(return_value=mock_description)
        mock_motion_group.forward_kinematics = AsyncMock(return_value=[])

        return mock_motion_group

    @pytest.fixture
    def empty_trajectory(self):
        """Create an empty trajectory for testing."""
        return api.models.JointTrajectory(
            joint_positions=[], times=[], locations=[], tcp="test_tcp"
        )

    @pytest.mark.asyncio
    async def test_model_loaded_only_once_for_same_motion_group(
        self, mock_motion_group, empty_trajectory
    ):
        """Should only call load_model_data once for the same motion group."""
        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch(
                "nova_rerun_bridge.trajectory.extract_link_chain_and_tcp", return_value=([], Mock())
            ),
            patch("nova_rerun_bridge.trajectory.RobotVisualizer") as mock_visualizer_class,
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
            patch("nova_rerun_bridge.trajectory.load_model_data") as mock_load_model,
        ):
            mock_load_model.return_value = b"fake glb data"
            mock_visualizer_class.return_value = Mock()

            # First call - should load model
            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=mock_motion_group,
                collision_setups={},
            )

            # Second call with same motion group - should use cached visualizer
            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=mock_motion_group,
                collision_setups={},
            )

            # Third call - still cached
            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=mock_motion_group,
                collision_setups={},
            )

            # Model should only be loaded once
            assert mock_load_model.call_count == 1
            # Visualizer should only be created once
            assert mock_visualizer_class.call_count == 1

    @pytest.mark.asyncio
    async def test_different_motion_groups_load_separate_models(self, empty_trajectory):
        """Should load model separately for different motion groups."""

        # Create two different motion groups
        def create_mock_motion_group(group_id):
            mock_setup = Mock()
            mock_setup.dh_parameters = [Mock() for _ in range(6)]
            mock_setup.mounting = Mock()

            mock_description = Mock()
            mock_description.dh_parameters = [Mock(a=0, d=0, alpha=0, theta=0) for _ in range(2)]
            mock_description.safety_tool_colliders = {}
            mock_description.safety_link_colliders = None

            mg = Mock()
            mg.id = group_id
            mg.get_setup = AsyncMock(return_value=mock_setup)
            mg.get_model = AsyncMock(return_value=f"model_{group_id}")
            mg.get_description = AsyncMock(return_value=mock_description)
            mg.forward_kinematics = AsyncMock(return_value=[])
            return mg

        motion_group_1 = create_mock_motion_group("group_1")
        motion_group_2 = create_mock_motion_group("group_2")

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch(
                "nova_rerun_bridge.trajectory.extract_link_chain_and_tcp", return_value=([], Mock())
            ),
            patch("nova_rerun_bridge.trajectory.RobotVisualizer") as mock_visualizer_class,
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
            patch("nova_rerun_bridge.trajectory.load_model_data") as mock_load_model,
        ):
            mock_load_model.return_value = b"fake glb data"
            mock_visualizer_class.return_value = Mock()

            # First motion group
            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=motion_group_1,
                collision_setups={},
            )

            # Second motion group - should load its own model
            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=motion_group_2,
                collision_setups={},
            )

            # Each motion group should have its own model loaded
            assert mock_load_model.call_count == 2
            assert mock_visualizer_class.call_count == 2

    @pytest.mark.asyncio
    async def test_visualizer_cache_stores_by_motion_group_id(
        self, mock_motion_group, empty_trajectory
    ):
        """Should store visualizers in cache by motion group ID."""
        test_cache = {}

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch(
                "nova_rerun_bridge.trajectory.extract_link_chain_and_tcp", return_value=([], Mock())
            ),
            patch("nova_rerun_bridge.trajectory.RobotVisualizer") as mock_visualizer_class,
            patch("nova_rerun_bridge.trajectory._visualizer_cache", test_cache),
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
            patch("nova_rerun_bridge.trajectory.load_model_data", return_value=b"fake data"),
        ):
            mock_visualizer = Mock()
            mock_visualizer_class.return_value = mock_visualizer

            await log_motion(
                trajectory=empty_trajectory,
                tcp="test_tcp",
                motion_group=mock_motion_group,
                collision_setups={},
            )

            # Cache should contain the motion group ID as key
            assert "test_group" in test_cache
            assert test_cache["test_group"] == mock_visualizer


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
        mock_setup.safety_setup.tcp_geometries = {}

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
        mock_setup.safety_setup.tcp_geometries = {}

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
        mock_setup.safety_setup.tcp_geometries = {}

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
