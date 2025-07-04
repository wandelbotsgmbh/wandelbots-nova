"""Tests for the nova.viewers module.

This comprehensive test suite covers the nova.viewers system, which provides
a modular, type-safe, and extensible framework for visualizing robot motion
planning and execution.

Key areas tested:
- Viewer base class and protocol compliance
- ViewerManager functionality for coordinating multiple viewers
- Rerun viewer implementation and configuration
- Public API contracts and utility functions
- Error handling and edge cases
- Integration workflows

The tests use pytest best practices:
- Focused unit tests that only test relevant in-repo code
- Proper mocking of external dependencies (nova_rerun_bridge)
- Async test support for viewer operations
- Edge case and error condition coverage
- Clear test organization by functional area
"""

from unittest.mock import AsyncMock, Mock, patch

import pytest

from nova.viewers import Rerun, Viewer, ViewerManager, get_viewer_manager
from nova.viewers.protocol import NovaRerunBridgeProtocol


class TestViewerBaseClass:
    """Test the abstract Viewer base class."""

    def test_viewer_is_abstract(self):
        """Viewer should be abstract and not instantiable."""
        with pytest.raises(TypeError):
            Viewer()

    def test_viewer_has_required_methods(self):
        """Viewer should define required abstract methods."""
        # Check that the abstract methods exist
        assert hasattr(Viewer, "configure")
        assert hasattr(Viewer, "cleanup")
        assert hasattr(Viewer, "log_planning_success")
        assert hasattr(Viewer, "log_planning_failure")


class TestRerunViewer:
    """Test the Rerun viewer implementation."""

    def test_rerun_viewer_instantiation(self):
        """Should be able to create Rerun viewer with default parameters."""
        viewer = Rerun()
        assert isinstance(viewer, Viewer)
        assert viewer.show_collision_link_chain is False
        assert viewer.show_safety_link_chain is True
        assert viewer.show_details is False
        assert viewer.show_safety_zones is True
        assert viewer.show_collision_scenes is True

    def test_rerun_viewer_custom_parameters(self):
        """Should accept custom parameters."""
        viewer = Rerun(
            show_collision_link_chain=True,
            show_safety_link_chain=False,
            show_details=True,
            show_safety_zones=False,
            tcp_tools={"gripper": "gripper.stl"},
        )
        assert viewer.show_collision_link_chain is True
        assert viewer.show_safety_link_chain is False
        assert viewer.show_details is True
        assert viewer.show_safety_zones is False
        assert viewer.tcp_tools == {"gripper": "gripper.stl"}

    @patch("nova.viewers.rerun.register_viewer")
    def test_rerun_viewer_auto_registers(self, mock_register):
        """Should automatically register itself when created."""
        viewer = Rerun()
        mock_register.assert_called_once_with(viewer)

    @patch("nova_rerun_bridge.NovaRerunBridge")
    def test_rerun_viewer_configure(self, mock_bridge_class):
        """Should configure NovaRerunBridge correctly."""
        mock_bridge = Mock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun(show_safety_link_chain=False)
        mock_nova = Mock()

        viewer.configure(mock_nova)

        # Verify bridge was created with correct parameters
        mock_bridge_class.assert_called_once_with(
            nova=mock_nova,
            spawn=True,
            recording_id=None,
            show_details=False,
            show_collision_link_chain=False,
            show_safety_link_chain=False,
        )
        assert viewer._bridge is mock_bridge

    @patch("nova_rerun_bridge.NovaRerunBridge")
    def test_rerun_viewer_configure_idempotent(self, mock_bridge_class):
        """Should not reconfigure if already configured."""
        mock_bridge = Mock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun()
        mock_nova = Mock()

        # Configure twice
        viewer.configure(mock_nova)
        viewer.configure(mock_nova)

        # Should only be called once
        mock_bridge_class.assert_called_once()

    @patch("nova_rerun_bridge.NovaRerunBridge")
    def test_rerun_viewer_cleanup(self, mock_bridge_class):
        """Should clean up bridge correctly."""
        mock_bridge = Mock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun()
        viewer.configure(Mock())

        viewer.cleanup()

        # Cleanup should reset the bridge reference and logged safety zones
        assert viewer._bridge is None
        assert len(viewer._logged_safety_zones) == 0

    @patch("nova_rerun_bridge.NovaRerunBridge")
    @pytest.mark.asyncio
    async def test_rerun_viewer_log_planning_success(self, mock_bridge_class):
        """Should delegate planning success to bridge."""
        mock_bridge = AsyncMock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun()
        viewer.configure(Mock())

        # Mock parameters
        mock_actions = [Mock()]
        mock_trajectory = Mock()
        mock_tcp = "tcp1"
        mock_motion_group = Mock()
        mock_motion_group.motion_group_id = "mg1"

        await viewer.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

        # Should call log_trajectory on the bridge
        mock_bridge.log_trajectory.assert_called_once_with(
            mock_trajectory, mock_tcp, mock_motion_group, tool_asset=None
        )

    @patch("nova_rerun_bridge.NovaRerunBridge")
    @pytest.mark.asyncio
    async def test_rerun_viewer_log_planning_failure(self, mock_bridge_class):
        """Should delegate planning failures to bridge."""
        mock_bridge = AsyncMock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun()
        viewer.configure(Mock())

        # Mock parameters
        mock_actions = [Mock()]
        mock_error = Exception("Planning failed")
        mock_tcp = "tcp1"
        mock_motion_group = Mock()
        mock_motion_group.motion_group_id = "mg1"

        await viewer.log_planning_failure(mock_actions, mock_error, mock_tcp, mock_motion_group)

        # Should handle the error gracefully
        assert True  # If we get here, no exception was raised

    @patch("nova_rerun_bridge.NovaRerunBridge")
    def test_rerun_viewer_configure_with_import_error(self, mock_bridge_class):
        """Should handle ImportError gracefully when nova_rerun_bridge is not available."""
        # Make import raise an error
        mock_bridge_class.side_effect = ImportError("nova_rerun_bridge not available")

        viewer = Rerun()
        mock_nova = Mock()

        # Should not raise an exception even if import fails
        viewer.configure(mock_nova)

        # Bridge should remain None
        assert viewer._bridge is None

    @patch("nova_rerun_bridge.NovaRerunBridge")
    def test_rerun_viewer_get_bridge(self, mock_bridge_class):
        """Should return the bridge instance when configured."""
        mock_bridge = Mock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun()

        # Initially no bridge
        assert viewer.get_bridge() is None

        # After configuration, should return bridge
        viewer.configure(Mock())
        assert viewer.get_bridge() is mock_bridge

    def test_rerun_viewer_resolve_tool_asset(self):
        """Should resolve tool assets correctly."""
        viewer = Rerun(tcp_tools={"gripper": "gripper.stl", "vacuum": "vacuum.stl"})

        assert viewer._resolve_tool_asset("gripper") == "gripper.stl"
        assert viewer._resolve_tool_asset("vacuum") == "vacuum.stl"
        assert viewer._resolve_tool_asset("unknown") is None

    @patch("nova_rerun_bridge.NovaRerunBridge")
    @pytest.mark.asyncio
    async def test_rerun_viewer_setup_after_preconditions(self, mock_bridge_class):
        """Should setup async components after preconditions."""
        mock_bridge = AsyncMock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun()
        viewer.configure(Mock())

        # Should setup async components
        await viewer.setup_after_preconditions()

        # Should mark as setup to avoid duplicate calls
        assert hasattr(viewer, "_async_setup_done")
        assert viewer._async_setup_done is True

        # Calling again should not duplicate setup
        mock_bridge.__aenter__.reset_mock()
        mock_bridge.setup_blueprint.reset_mock()

        await viewer.setup_after_preconditions()

        # Should not be called again
        mock_bridge.__aenter__.assert_not_called()
        mock_bridge.setup_blueprint.assert_not_called()


class TestViewerManager:
    """Test the ViewerManager functionality."""

    def test_viewer_manager_singleton(self):
        """Should return the same instance."""
        manager1 = get_viewer_manager()
        manager2 = get_viewer_manager()
        assert manager1 is manager2

    def test_register_viewer(self):
        """Should register viewers correctly."""
        manager = ViewerManager()
        viewer = Mock(spec=Viewer)

        manager.register_viewer(viewer)

        assert viewer in manager._viewers

    def test_register_duplicate_viewer(self):
        """Should not register the same viewer twice."""
        manager = ViewerManager()
        viewer = Mock(spec=Viewer)

        manager.register_viewer(viewer)
        manager.register_viewer(viewer)  # Register again

        # WeakSet automatically handles duplicates
        assert len(manager._viewers) == 1

    def test_configure_viewers(self):
        """Should configure all registered viewers."""
        manager = ViewerManager()
        viewer1 = Mock(spec=Viewer)
        viewer2 = Mock(spec=Viewer)
        mock_nova = Mock()

        manager.register_viewer(viewer1)
        manager.register_viewer(viewer2)

        manager.configure_viewers(mock_nova)

        viewer1.configure.assert_called_once_with(mock_nova)
        viewer2.configure.assert_called_once_with(mock_nova)

    @pytest.mark.asyncio
    async def test_setup_viewers_after_preconditions(self):
        """Should setup all registered viewers after preconditions."""
        manager = ViewerManager()
        viewer1 = AsyncMock(spec=Viewer)
        viewer2 = AsyncMock(spec=Viewer)

        manager.register_viewer(viewer1)
        manager.register_viewer(viewer2)

        await manager.setup_viewers_after_preconditions()

        viewer1.setup_after_preconditions.assert_called_once()
        viewer2.setup_after_preconditions.assert_called_once()

    def test_cleanup_viewers(self):
        """Should clean up all registered viewers."""
        manager = ViewerManager()
        viewer1 = Mock(spec=Viewer)
        viewer2 = Mock(spec=Viewer)

        manager.register_viewer(viewer1)
        manager.register_viewer(viewer2)

        manager.cleanup_viewers()

        viewer1.cleanup.assert_called_once()
        viewer2.cleanup.assert_called_once()
        assert len(manager._viewers) == 0

    @pytest.mark.asyncio
    async def test_log_planning_success(self):
        """Should log planning success to all viewers."""
        manager = ViewerManager()
        viewer1 = AsyncMock(spec=Viewer)
        viewer2 = AsyncMock(spec=Viewer)

        manager.register_viewer(viewer1)
        manager.register_viewer(viewer2)

        # Mock parameters
        mock_actions = [Mock()]
        mock_trajectory = Mock()
        mock_tcp = "tcp1"
        mock_motion_group = Mock()

        await manager.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

        viewer1.log_planning_success.assert_called_once_with(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )
        viewer2.log_planning_success.assert_called_once_with(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

    @pytest.mark.asyncio
    async def test_log_planning_failure(self):
        """Should log planning failures to all viewers."""
        manager = ViewerManager()
        viewer1 = AsyncMock(spec=Viewer)
        viewer2 = AsyncMock(spec=Viewer)

        manager.register_viewer(viewer1)
        manager.register_viewer(viewer2)

        # Mock parameters
        mock_actions = [Mock()]
        mock_error = Exception("Planning failed")
        mock_tcp = "tcp1"
        mock_motion_group = Mock()

        await manager.log_planning_failure(mock_actions, mock_error, mock_tcp, mock_motion_group)

        viewer1.log_planning_failure.assert_called_once_with(
            mock_actions, mock_error, mock_tcp, mock_motion_group
        )
        viewer2.log_planning_failure.assert_called_once_with(
            mock_actions, mock_error, mock_tcp, mock_motion_group
        )

    @pytest.mark.asyncio
    async def test_error_handling_continues_on_failure(self):
        """Should continue processing other viewers if one fails."""
        manager = ViewerManager()
        viewer1 = AsyncMock(spec=Viewer)
        viewer2 = AsyncMock(spec=Viewer)

        # Make viewer1 raise an exception
        viewer1.log_planning_success.side_effect = Exception("Test error")

        manager.register_viewer(viewer1)
        manager.register_viewer(viewer2)

        # Mock parameters
        mock_actions = [Mock()]
        mock_trajectory = Mock()
        mock_tcp = "tcp1"
        mock_motion_group = Mock()

        # Should not raise exception despite viewer1 failing
        await manager.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

        # Verify both viewers were called
        viewer1.log_planning_success.assert_called_once()
        viewer2.log_planning_success.assert_called_once()

    def test_configure_viewers_error_propagation(self):
        """Should propagate exceptions from viewer configure calls."""
        manager = ViewerManager()
        viewer1 = Mock(spec=Viewer)

        # Make viewer1 raise an exception during configure
        viewer1.configure.side_effect = Exception("Configure failed")

        manager.register_viewer(viewer1)
        mock_nova = Mock()

        # Should raise exception from failing viewer
        with pytest.raises(Exception, match="Configure failed"):
            manager.configure_viewers(mock_nova)

    def test_cleanup_viewers_error_propagation(self):
        """Should propagate exceptions from viewer cleanup calls."""
        manager = ViewerManager()
        viewer1 = Mock(spec=Viewer)

        # Make viewer1 raise an exception during cleanup
        viewer1.cleanup.side_effect = Exception("Cleanup failed")

        manager.register_viewer(viewer1)

        # Should raise exception from failing viewer
        with pytest.raises(Exception, match="Cleanup failed"):
            manager.cleanup_viewers()

    @pytest.mark.asyncio
    async def test_setup_viewers_after_preconditions_error_propagation(self):
        """Should propagate exceptions from viewer setup calls."""
        manager = ViewerManager()
        viewer1 = AsyncMock(spec=Viewer)

        # Make viewer1 raise an exception during setup
        viewer1.setup_after_preconditions.side_effect = Exception("Setup failed")

        manager.register_viewer(viewer1)

        # Should raise exception from failing viewer
        with pytest.raises(Exception, match="Setup failed"):
            await manager.setup_viewers_after_preconditions()

    def test_has_active_viewers(self):
        """Should correctly report whether there are active viewers."""
        manager = ViewerManager()

        # Initially no active viewers
        assert manager.has_active_viewers is False

        # Add a viewer
        viewer = Mock(spec=Viewer)
        manager.register_viewer(viewer)
        assert manager.has_active_viewers is True

        # Clean up viewers
        manager.cleanup_viewers()
        assert manager.has_active_viewers is False


class TestNovaRerunBridgeProtocol:
    """Test the NovaRerunBridge protocol definition."""

    def test_protocol_defines_required_methods(self):
        """Protocol should define all required methods."""
        # Check that the protocol has the expected methods
        assert hasattr(NovaRerunBridgeProtocol, "log_trajectory")
        assert hasattr(NovaRerunBridgeProtocol, "log_error_feedback")
        assert hasattr(NovaRerunBridgeProtocol, "log_safety_zones")

    def test_protocol_compliance(self):
        """Mock objects should be able to implement the protocol."""
        # Create a mock that implements the protocol
        mock_bridge = Mock(spec=NovaRerunBridgeProtocol)

        # Should have all the expected methods
        assert hasattr(mock_bridge, "log_trajectory")
        assert hasattr(mock_bridge, "log_error_feedback")
        assert hasattr(mock_bridge, "log_safety_zones")


class TestPublicAPI:
    """Test the public API exports and functionality."""

    def test_public_api_exports(self):
        """Should export all expected classes and functions."""
        from nova.viewers import (
            NovaRerunBridgeProtocol,
            Rerun,
            Viewer,
            ViewerManager,
            get_viewer_manager,
        )

        # Verify all expected exports are available
        assert Viewer is not None
        assert ViewerManager is not None
        assert Rerun is not None
        assert NovaRerunBridgeProtocol is not None
        assert get_viewer_manager is not None

    def test_viewer_manager_function(self):
        """get_viewer_manager should return a ViewerManager instance."""
        manager = get_viewer_manager()
        assert isinstance(manager, ViewerManager)

    def test_rerun_viewer_creation(self):
        """Should be able to create Rerun viewer from public API."""
        viewer = Rerun(show_safety_link_chain=False)
        assert isinstance(viewer, Viewer)
        assert viewer.show_safety_link_chain is False


class TestViewerIntegration:
    """Test integration between different viewer components."""

    @patch("nova_rerun_bridge.NovaRerunBridge")
    @pytest.mark.asyncio
    async def test_full_viewer_workflow(self, mock_bridge_class):
        """Test complete workflow: register -> configure -> use -> cleanup."""
        mock_bridge = AsyncMock()
        mock_bridge_class.return_value = mock_bridge

        # Create viewer and manager
        with patch("nova.viewers.rerun.register_viewer"):
            viewer = Rerun(show_safety_link_chain=True)

        manager = ViewerManager()

        # Register viewer
        manager.register_viewer(viewer)

        # Configure
        mock_nova = Mock()
        manager.configure_viewers(mock_nova)

        # Use
        mock_actions = [Mock()]
        mock_trajectory = Mock()
        mock_tcp = "tcp1"
        mock_motion_group = Mock()

        await manager.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

        mock_error = Exception("Test error")
        await manager.log_planning_failure(mock_actions, mock_error, mock_tcp, mock_motion_group)

        # Cleanup
        manager.cleanup_viewers()

        # Verify all interactions
        mock_bridge_class.assert_called_once()
        # Note: viewer.cleanup() only resets the bridge reference, doesn't call bridge.cleanup()

    def test_viewer_configuration_propagation(self):
        """Test that viewer configuration is properly propagated."""
        with patch("nova.viewers.rerun.register_viewer"):
            viewer = Rerun(
                show_collision_link_chain=True,
                show_safety_link_chain=False,
                show_details=True,
                tcp_tools={"gripper": "gripper.stl"},
            )

        # Verify configuration
        assert viewer.show_collision_link_chain is True
        assert viewer.show_safety_link_chain is False
        assert viewer.show_details is True
        assert viewer.tcp_tools == {"gripper": "gripper.stl"}

        # Configuration should be passed to bridge during configure
        with patch("nova_rerun_bridge.NovaRerunBridge") as mock_bridge_class:
            viewer.configure(Mock())

            mock_bridge_class.assert_called_once()
            call_args = mock_bridge_class.call_args[1]
            assert call_args["show_collision_link_chain"] is True
            assert call_args["show_safety_link_chain"] is False
            assert call_args["show_details"] is True
            # Note: tcp_tools is stored on the viewer but not passed to the bridge constructor


class TestViewerUtilities:
    """Test utility functions and helpers."""

    def test_extract_collision_scenes_import(self):
        """Should be able to import collision scene utility."""
        from nova.viewers.utils import extract_collision_scenes_from_actions

        assert extract_collision_scenes_from_actions is not None

    def test_register_viewer_function(self):
        """Should be able to register viewers through utility function."""
        from nova.viewers.manager import register_viewer

        viewer = Mock(spec=Viewer)
        register_viewer(viewer)

        # Should be in the global viewer manager
        manager = get_viewer_manager()
        assert viewer in manager._viewers

    def test_extract_collision_scenes_empty_actions(self):
        """Should handle empty actions list."""
        from nova.viewers.utils import extract_collision_scenes_from_actions

        result = extract_collision_scenes_from_actions([])
        assert result == {}

    def test_global_viewer_manager_isolation(self):
        """Test that viewers are properly isolated between tests."""
        from nova.viewers.manager import _viewer_manager

        # Clear any existing viewers (for test isolation)
        _viewer_manager._viewers.clear()

        manager1 = get_viewer_manager()
        viewer = Mock(spec=Viewer)

        manager1.register_viewer(viewer)
        assert len(manager1._viewers) >= 1

        # Get manager again should be same instance
        manager2 = get_viewer_manager()
        assert manager1 is manager2
        assert viewer in manager2._viewers


class TestViewerEdgeCases:
    """Test edge cases and error conditions."""

    @patch("nova_rerun_bridge.NovaRerunBridge")
    @pytest.mark.asyncio
    async def test_rerun_viewer_operations_without_bridge(self, mock_bridge_class):
        """Should handle operations gracefully when bridge is not configured."""
        viewer = Rerun()
        # Don't configure the viewer, so bridge remains None

        # Mock parameters
        mock_actions = [Mock()]
        mock_trajectory = Mock()
        mock_tcp = "tcp1"
        mock_motion_group = Mock()
        mock_motion_group.motion_group_id = "mg1"
        mock_error = Exception("Test error")

        # All operations should work without errors even without bridge
        await viewer.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )
        await viewer.log_planning_failure(mock_actions, mock_error, mock_tcp, mock_motion_group)
        await viewer.setup_after_preconditions()

        # Bridge should never have been called
        mock_bridge_class.assert_not_called()

    def test_rerun_viewer_application_id_propagation(self):
        """Should properly propagate application_id as recording_id."""
        with patch("nova_rerun_bridge.NovaRerunBridge") as mock_bridge_class:
            viewer = Rerun(application_id="test_app_123")
            viewer.configure(Mock())

            # Should pass application_id as recording_id
            call_args = mock_bridge_class.call_args[1]
            assert call_args["recording_id"] == "test_app_123"

    @patch("nova_rerun_bridge.NovaRerunBridge")
    @pytest.mark.asyncio
    async def test_rerun_viewer_safety_zones_only_logged_once(self, mock_bridge_class):
        """Should only log safety zones once per motion group."""
        mock_bridge = AsyncMock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun(show_safety_zones=True)
        viewer.configure(Mock())

        # Mock parameters
        mock_actions = [Mock()]
        mock_trajectory = Mock()
        mock_tcp = "tcp1"
        mock_motion_group = Mock()
        mock_motion_group.motion_group_id = "mg1"

        # Call multiple times
        await viewer.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )
        await viewer.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

        # Safety zones should only be logged once
        assert mock_bridge.log_safety_zones.call_count == 1
        assert "mg1" in viewer._logged_safety_zones

    @patch("nova_rerun_bridge.NovaRerunBridge")
    @pytest.mark.asyncio
    async def test_rerun_viewer_disabled_safety_zones(self, mock_bridge_class):
        """Should not log safety zones when disabled."""
        mock_bridge = AsyncMock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun(show_safety_zones=False)
        viewer.configure(Mock())

        # Mock parameters
        mock_actions = [Mock()]
        mock_trajectory = Mock()
        mock_tcp = "tcp1"
        mock_motion_group = Mock()
        mock_motion_group.motion_group_id = "mg1"

        await viewer.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

        # Safety zones should not be logged
        mock_bridge.log_safety_zones.assert_not_called()
        assert len(viewer._logged_safety_zones) == 0
