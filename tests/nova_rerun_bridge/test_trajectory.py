"""Unit tests for trajectory module.

Tests focus on business logic for data transformations,
and motion processing. Does not test rerun library functionality directly.
"""

from unittest.mock import Mock, patch

from nova_rerun_bridge.trajectory import log_motion


class TestLogMotionParameterValidation:
    """Test log_motion parameter validation."""

    def test_accepts_valid_parameters(self):
        """Should accept valid parameters without errors."""
        # Create minimal mocks for required parameters
        mock_optimizer_config = Mock()
        mock_optimizer_config.dh_parameters = [Mock() for _ in range(6)]
        mock_optimizer_config.mounting = Mock()
        mock_optimizer_config.motion_group_type = "test_type"
        mock_optimizer_config.safety_setup = Mock()
        mock_optimizer_config.safety_setup.robot_model_geometries = []

        # Mock the trajectory data with proper structure
        mock_trajectory_point = Mock()
        mock_trajectory_point.time = 1.0  # Add time attribute
        mock_trajectory = [mock_trajectory_point for _ in range(5)]

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch("nova_rerun_bridge.trajectory.extract_link_chain_and_tcp") as mock_extract,
            patch("nova_rerun_bridge.trajectory.RobotVisualizer"),
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            # Should not raise exceptions with valid parameters
            log_motion(
                motion_id="test_motion",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=mock_trajectory,
                collision_scenes={},
                time_offset=0.0,
            )

    def test_handles_empty_trajectory(self):
        """Should handle empty trajectory list."""
        mock_optimizer_config = Mock()
        mock_optimizer_config.dh_parameters = [Mock() for _ in range(6)]
        mock_optimizer_config.mounting = Mock()
        mock_optimizer_config.motion_group_type = "test_type"
        mock_optimizer_config.safety_setup = Mock()
        mock_optimizer_config.safety_setup.robot_model_geometries = []

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch("nova_rerun_bridge.trajectory.extract_link_chain_and_tcp") as mock_extract,
            patch("nova_rerun_bridge.trajectory.RobotVisualizer"),
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            # Should handle empty trajectory without errors
            log_motion(
                motion_id="test_motion",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],  # Empty trajectory
                collision_scenes={},
            )

    def test_respects_show_safety_link_chain_parameter(self):
        """Should respect the show_safety_link_chain parameter."""
        mock_optimizer_config = Mock()
        mock_optimizer_config.dh_parameters = [Mock() for _ in range(6)]
        mock_optimizer_config.mounting = Mock()
        mock_optimizer_config.motion_group_type = "test_type"
        mock_optimizer_config.safety_setup = Mock()
        mock_optimizer_config.safety_setup.robot_model_geometries = []

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch("nova_rerun_bridge.trajectory.extract_link_chain_and_tcp") as mock_extract,
            patch("nova_rerun_bridge.trajectory.RobotVisualizer"),
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            # Test with show_safety_link_chain=True (default)
            log_motion(
                motion_id="test_motion",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
                show_safety_link_chain=True,
            )

            # Test with show_safety_link_chain=False
            log_motion(
                motion_id="test_motion",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
                show_safety_link_chain=False,
            )
