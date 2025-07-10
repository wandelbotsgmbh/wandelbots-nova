"""Tests for Nova External Integration API"""

from unittest.mock import Mock, patch

from nova.external_control.external_integration import (
    nova_get_available_robots,
    nova_pause_robot,
    nova_resume_robot,
    nova_set_playback_speed,
    register_external_control_functions,
)


class TestExternalIntegration:
    """Test suite for external control function registration and calling"""

    def test_register_functions_adds_to_globals(self):
        """Test that functions are registered in global namespace"""
        # Note: In a real test environment, we'd want to check this more carefully
        # For now, just ensure the function exists and can be called
        register_external_control_functions()

        # Functions should be accessible
        assert callable(nova_set_playback_speed)
        assert callable(nova_pause_robot)
        assert callable(nova_resume_robot)
        assert callable(nova_get_available_robots)

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_nova_set_playback_speed_success(self, mock_get_manager):
        """Test successful speed setting via external function"""
        mock_manager = Mock()
        mock_get_manager.return_value = mock_manager

        result = nova_set_playback_speed("robot1", 50)

        assert result["success"] is True
        assert result["speed"] == 50
        assert result["robot_id"] == "robot1"
        assert "50%" in result["message"]
        mock_manager.set_external_override.assert_called_once()

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_nova_set_playback_speed_clamps_values(self, mock_get_manager):
        """Test that speed values are clamped to valid range"""
        mock_manager = Mock()
        mock_get_manager.return_value = mock_manager

        # Test clamping high value
        result = nova_set_playback_speed("robot1", 150)
        assert result["success"] is True
        assert result["speed"] == 100

        # Test clamping low value
        result = nova_set_playback_speed("robot1", -50)
        assert result["success"] is True
        assert result["speed"] == 0

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_nova_pause_robot_success(self, mock_get_manager):
        """Test successful robot pausing via external function"""
        mock_manager = Mock()
        mock_get_manager.return_value = mock_manager

        result = nova_pause_robot("robot1")

        assert result["success"] is True
        assert result["motion_group_id"] == "robot1"
        assert result["state"] == "paused"
        mock_manager.pause.assert_called_once()

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_nova_resume_robot_success(self, mock_get_manager):
        """Test successful robot resuming via external function"""
        mock_manager = Mock()
        mock_get_manager.return_value = mock_manager

        result = nova_resume_robot("robot1")

        assert result["success"] is True
        assert result["robot_id"] == "robot1"
        assert result["state"] == "playing"
        mock_manager.resume.assert_called_once()

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_error_handling_in_external_functions(self, mock_get_manager):
        """Test that exceptions are caught and returned as error responses"""
        mock_manager = Mock()
        mock_manager.set_external_override.side_effect = Exception("Test error")
        mock_get_manager.return_value = mock_manager

        result = nova_set_playback_speed("robot1", 50)

        assert result["success"] is False
        assert "Test error" in result["error"]
        assert result["robot_id"] == "robot1"

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_get_available_robots_success(self, mock_get_manager):
        """Test robot discovery function"""
        mock_manager = Mock()
        mock_manager.get_all_robots.return_value = ["robot1", "robot2"]
        mock_manager.get_effective_speed.side_effect = [50, 80]
        mock_manager.get_effective_state.side_effect = [Mock(value="playing"), Mock(value="paused")]
        mock_get_manager.return_value = mock_manager

        result = nova_get_available_robots()

        assert result["success"] is True
        assert "robots" in result
        assert len(result["robots"]) == 2
        assert result["count"] == 2

        # Check robot details
        robot1 = result["robots"][0]
        assert robot1["id"] == "robot1"
        assert robot1["speed"] == 50
        assert robot1["speed_percent"] == "50%"

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_get_available_robots_empty(self, mock_get_manager):
        """Test robot discovery when no robots are available"""
        mock_manager = Mock()
        mock_manager.get_all_robots.return_value = []
        mock_get_manager.return_value = mock_manager

        result = nova_get_available_robots()

        assert result["success"] is True
        assert result["robots"] == []
        assert result["count"] == 0

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_get_available_robots_error_handling(self, mock_get_manager):
        """Test error handling in robot discovery"""
        mock_get_manager.side_effect = Exception("Manager error")

        result = nova_get_available_robots()

        assert result["success"] is False
        assert "Manager error" in result["error"]
        assert result["robots"] == []

    def test_speed_clamping_edge_cases(self):
        """Test edge cases for speed clamping"""
        # Test with exactly 0.0 and 1.0
        with patch(
            "nova.external_control.external_integration.get_playback_manager"
        ) as mock_get_manager:
            mock_manager = Mock()
            mock_get_manager.return_value = mock_manager

            result = nova_set_playback_speed("robot1", 0)
            assert result["success"] is True
            assert result["speed"] == 0

            result = nova_set_playback_speed("robot1", 100)
            assert result["success"] is True
            assert result["speed"] == 100


class TestFunctionReturnFormats:
    """Test that all external functions return consistent formats"""

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_consistent_success_format(self, mock_get_manager):
        """Test that success responses have consistent format"""
        mock_manager = Mock()
        mock_manager.get_all_robots.return_value = []
        mock_get_manager.return_value = mock_manager

        functions_and_args = [
            (nova_set_playback_speed, ("robot1", 50)),
            (nova_pause_robot, ("robot1",)),
            (nova_resume_robot, ("robot1",)),
            (nova_get_available_robots, ()),
        ]

        for func, args in functions_and_args:
            result = func(*args)

            # All should have success field
            assert "success" in result
            assert isinstance(result["success"], bool)

            # All should have message field
            assert "message" in result
            assert isinstance(result["message"], str)

    @patch("nova.external_control.external_integration.get_playback_manager")
    def test_consistent_error_format(self, mock_get_manager):
        """Test that error responses have consistent format"""
        mock_get_manager.side_effect = Exception("Test error")

        functions_and_args = [
            (nova_set_playback_speed, ("robot1", 50)),
            (nova_pause_robot, ("robot1",)),
            (nova_resume_robot, ("robot1",)),
            (nova_get_available_robots, ()),
        ]

        for func, args in functions_and_args:
            result = func(*args)

            # All should have success=False
            assert result["success"] is False

            # All should have error field
            assert "error" in result
            assert isinstance(result["error"], str)

            # All should have message field
            assert "message" in result
            assert isinstance(result["message"], str)
