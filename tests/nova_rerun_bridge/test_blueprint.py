"""Unit tests for nova_rerun_bridge.blueprint module.

This test suite focuses on business logic and data transformations within the
blueprint module. It tests the core functionality without directly testing
rerun library internals, using mocks to isolate the code under test.

The tests cover:
- Blueprint creation and configuration
- Motion group handling and integration
"""

from unittest.mock import Mock, patch

from nova_rerun_bridge.blueprint import get_blueprint, send_blueprint


class TestGetBlueprint:
    """Test blueprint generation and configuration."""

    def test_handles_empty_motion_group_list(self):
        """Should handle empty motion group list without errors."""
        with patch("nova_rerun_bridge.blueprint.rrb.Blueprint") as mock_blueprint:
            mock_blueprint.return_value = Mock()
            get_blueprint([])
            mock_blueprint.assert_called_once()

    def test_handles_single_motion_group(self):
        """Should handle single motion group correctly."""
        with patch("nova_rerun_bridge.blueprint.rrb.Blueprint") as mock_blueprint:
            mock_blueprint.return_value = Mock()
            get_blueprint(["single_group"])
            mock_blueprint.assert_called_once()

    def test_handles_multiple_motion_groups(self):
        """Should handle multiple motion groups correctly."""
        with patch("nova_rerun_bridge.blueprint.rrb.Blueprint") as mock_blueprint:
            mock_blueprint.return_value = Mock()
            get_blueprint(["group1", "group2", "group3"])
            mock_blueprint.assert_called_once()


class TestSendBlueprint:
    """Test blueprint sending functionality."""

    @patch("nova_rerun_bridge.blueprint.rr.send_blueprint")
    @patch("nova_rerun_bridge.blueprint.get_blueprint")
    def test_sends_generated_blueprint_to_rerun(self, mock_get_blueprint, mock_send):
        """Should call rerun's send_blueprint with the generated blueprint."""
        motion_group_list = ["group1"]
        mock_blueprint = Mock()
        mock_get_blueprint.return_value = mock_blueprint

        send_blueprint(motion_group_list)

        mock_get_blueprint.assert_called_once_with(motion_group_list)
        mock_send.assert_called_once_with(mock_blueprint)
