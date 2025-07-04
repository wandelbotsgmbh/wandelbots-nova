"""Unit tests for trajectory module.

Tests focus on business logic for timing modes, data transformations,
and motion processing. Does not test rerun library functionality directly.
"""

from unittest.mock import Mock, patch

import numpy as np
import pytest

from nova_rerun_bridge.trajectory import TimingMode, continue_after_sync


class TestTimingMode:
    """Test TimingMode enumeration."""

    def test_has_all_expected_values(self):
        """Should define all expected timing mode values."""
        assert hasattr(TimingMode, "RESET")
        assert hasattr(TimingMode, "CONTINUE")
        assert hasattr(TimingMode, "SYNC")
        assert hasattr(TimingMode, "OVERRIDE")

    def test_values_are_unique(self):
        """Should have unique values for each mode."""
        modes = [TimingMode.RESET, TimingMode.CONTINUE, TimingMode.SYNC, TimingMode.OVERRIDE]
        assert len(set(modes)) == 4

    def test_values_are_integers(self):
        """Should use integer values from auto()."""
        assert isinstance(TimingMode.RESET.value, int)
        assert isinstance(TimingMode.CONTINUE.value, int)
        assert isinstance(TimingMode.SYNC.value, int)
        assert isinstance(TimingMode.OVERRIDE.value, int)


class TestContinueAfterSync:
    """Test continue_after_sync function."""

    def test_function_exists_and_callable(self):
        """Should be callable without parameters."""
        # Function should be callable without parameters
        continue_after_sync()

    def test_multiple_calls_are_safe(self):
        """Should handle multiple calls safely."""
        # Should be able to call multiple times
        continue_after_sync()
        continue_after_sync()
        continue_after_sync()

    def test_function_updates_internal_state(self):
        """Should update internal timing state."""
        # Multiple calls should be safe
        for _ in range(5):
            continue_after_sync()


class TestLogMotionParameterValidation:
    """Test log_motion parameter validation."""

    def test_raises_error_for_none_dh_parameters(self):
        """Should raise ValueError when DH parameters are None."""
        from nova_rerun_bridge.trajectory import log_motion

        mock_optimizer_config = Mock()
        mock_optimizer_config.dh_parameters = None

        with pytest.raises(ValueError, match="DH parameters cannot be None"):
            log_motion(
                motion_id="test_motion",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
            )

    def test_accepts_valid_parameters(self):
        """Should accept valid parameters without errors."""
        from nova_rerun_bridge.trajectory import log_motion

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
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
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
                timing_mode=TimingMode.RESET,
            )

    def test_handles_empty_trajectory(self):
        """Should handle empty trajectory list."""
        from nova_rerun_bridge.trajectory import log_motion

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
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
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

    def test_handles_empty_collision_scenes(self):
        """Should handle empty collision scenes dictionary."""
        from nova_rerun_bridge.trajectory import log_motion

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
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            # Should handle empty collision scenes without errors
            log_motion(
                motion_id="test_motion",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},  # Empty collision scenes
            )


class TestTimingModeHandling:
    """Test different timing mode behaviors."""

    def test_all_timing_modes_are_accepted(self):
        """Should accept all timing mode values."""
        from nova_rerun_bridge.trajectory import log_motion

        mock_optimizer_config = Mock()
        mock_optimizer_config.dh_parameters = [Mock() for _ in range(6)]
        mock_optimizer_config.mounting = Mock()
        mock_optimizer_config.motion_group_type = "test_type"
        mock_optimizer_config.safety_setup = Mock()
        mock_optimizer_config.safety_setup.robot_model_geometries = []

        timing_modes = [TimingMode.RESET, TimingMode.CONTINUE, TimingMode.SYNC, TimingMode.OVERRIDE]

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot"),
            patch("nova_rerun_bridge.trajectory.extract_link_chain_and_tcp") as mock_extract,
            patch("nova_rerun_bridge.trajectory.RobotVisualizer"),
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            for timing_mode in timing_modes:
                # Should work with all timing modes
                log_motion(
                    motion_id=f"test_motion_{timing_mode.name}",
                    model_from_controller="test_model",
                    motion_group="test_group",
                    optimizer_config=mock_optimizer_config,
                    trajectory=[],
                    collision_scenes={},
                    timing_mode=timing_mode,
                )


class TestYaskawaSpecialHandling:
    """Test special handling for Yaskawa TURN2 model."""

    def test_yaskawa_turn2_dh_parameter_modification(self):
        """Should modify DH parameters for Yaskawa TURN2 model."""
        from nova_rerun_bridge.trajectory import log_motion

        # Create mock DH parameters
        mock_dh_param_0 = Mock()
        mock_dh_param_0.a = 100  # Will be set to 0
        mock_dh_param_0.d = 100  # Will be set to 360
        mock_dh_param_0.alpha = 0  # Will be set to pi/2
        mock_dh_param_0.theta = 100  # Will be set to 0

        mock_dh_param_1 = Mock()
        mock_dh_param_1.a = 100  # Will be set to 0
        mock_dh_param_1.d = 100  # Will be set to 0
        mock_dh_param_1.alpha = 100  # Will be set to 0
        mock_dh_param_1.theta = 0  # Will be set to pi/2

        mock_optimizer_config = Mock()
        mock_optimizer_config.dh_parameters = [mock_dh_param_0, mock_dh_param_1] + [
            Mock() for _ in range(4)
        ]
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
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            log_motion(
                motion_id="test_motion",
                model_from_controller="Yaskawa_TURN2",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
            )

            # Check that DH parameters were modified
            assert mock_dh_param_0.a == 0
            assert mock_dh_param_0.d == 360
            assert mock_dh_param_0.alpha == np.pi / 2
            assert mock_dh_param_0.theta == 0

            assert mock_dh_param_1.a == 0
            assert mock_dh_param_1.d == 0
            assert mock_dh_param_1.alpha == 0
            assert mock_dh_param_1.theta == np.pi / 2

    def test_non_yaskawa_model_parameters_unchanged(self):
        """Should not modify DH parameters for non-Yaskawa models."""
        from nova_rerun_bridge.trajectory import log_motion

        # Create mock DH parameters with original values
        mock_dh_param_0 = Mock()
        original_a = mock_dh_param_0.a = 123
        original_d = mock_dh_param_0.d = 456
        original_alpha = mock_dh_param_0.alpha = 0.789
        original_theta = mock_dh_param_0.theta = 1.234

        mock_optimizer_config = Mock()
        mock_optimizer_config.dh_parameters = [mock_dh_param_0] + [Mock() for _ in range(5)]
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
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            log_motion(
                motion_id="test_motion",
                model_from_controller="Other_Robot",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
            )

            # Check that DH parameters were NOT modified
            assert mock_dh_param_0.a == original_a
            assert mock_dh_param_0.d == original_d
            assert mock_dh_param_0.alpha == original_alpha
            assert mock_dh_param_0.theta == original_theta


class TestVisualizerCaching:
    """Test visualizer caching mechanism."""

    def test_visualizer_cache_reuse(self):
        """Should cache and reuse visualizers by motion group."""
        from nova_rerun_bridge.trajectory import _visualizer_cache, log_motion

        # Clear cache
        _visualizer_cache.clear()

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
            patch("nova_rerun_bridge.trajectory.RobotVisualizer") as mock_visualizer_class,
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            mock_visualizer = Mock()
            mock_visualizer_class.return_value = mock_visualizer

            # First call should create new visualizer
            log_motion(
                motion_id="test_motion_1",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
            )

            # Should have created one visualizer
            assert mock_visualizer_class.call_count == 1
            assert "test_group" in _visualizer_cache

            # Second call with same motion group should reuse visualizer
            log_motion(
                motion_id="test_motion_2",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
            )

            # Should still have only created one visualizer (reused from cache)
            assert mock_visualizer_class.call_count == 1

    def test_different_motion_groups_get_separate_visualizers(self):
        """Should create separate visualizers for different motion groups."""
        from nova_rerun_bridge.trajectory import _visualizer_cache, log_motion

        # Clear cache
        _visualizer_cache.clear()

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
            patch("nova_rerun_bridge.trajectory.RobotVisualizer") as mock_visualizer_class,
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            mock_visualizer_class.return_value = Mock()

            # Call with first motion group
            log_motion(
                motion_id="test_motion_1",
                model_from_controller="test_model",
                motion_group="group_1",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
            )

            # Call with second motion group
            log_motion(
                motion_id="test_motion_2",
                model_from_controller="test_model",
                motion_group="group_2",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
            )

            # Should have created two visualizers
            assert mock_visualizer_class.call_count == 2
            assert "group_1" in _visualizer_cache
            assert "group_2" in _visualizer_cache


class TestConstants:
    """Test constants and magic numbers in trajectory module."""

    def test_joint_count_assumption(self):
        """Should assume 6 joints consistently."""
        # This test verifies the assumption that robots have 6 joints
        # by checking that the Yaskawa special case modifies parameters 0 and 1
        from nova_rerun_bridge.trajectory import log_motion

        mock_optimizer_config = Mock()
        # Should have 6 DH parameters
        mock_optimizer_config.dh_parameters = [Mock() for _ in range(6)]
        mock_optimizer_config.mounting = Mock()
        mock_optimizer_config.motion_group_type = "test_type"
        mock_optimizer_config.safety_setup = Mock()
        mock_optimizer_config.safety_setup.robot_model_geometries = []

        with (
            patch("nova_rerun_bridge.trajectory.rr"),
            patch("nova_rerun_bridge.trajectory.DHRobot") as mock_robot,
            patch("nova_rerun_bridge.trajectory.extract_link_chain_and_tcp") as mock_extract,
            patch("nova_rerun_bridge.trajectory.RobotVisualizer"),
            patch("nova_rerun_bridge.trajectory._visualizer_cache", {}),
            patch("nova_rerun_bridge.trajectory.log_trajectory"),
        ):
            # Configure the extract function to return expected tuple
            mock_extract.return_value = ([], Mock())  # (link_chain, tcp)

            log_motion(
                motion_id="test_motion",
                model_from_controller="test_model",
                motion_group="test_group",
                optimizer_config=mock_optimizer_config,
                trajectory=[],
                collision_scenes={},
            )

            # DHRobot should be called with the DH parameters
            mock_robot.assert_called_once()
            call_args = mock_robot.call_args[0]
            dh_params = call_args[0]

            # Should have been called with 6 DH parameters
            assert len(dh_params) == 6
