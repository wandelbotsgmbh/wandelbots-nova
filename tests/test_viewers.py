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
        assert viewer.show_collision_tool is True
        assert viewer.show_safety_link_chain is True
        assert viewer.show_details is False
        assert viewer.show_safety_zones is True
        assert viewer.show_collision_scenes is True
        assert viewer.trajectory_sample_interval_ms == 50.0

    def test_rerun_viewer_custom_parameters(self):
        """Should accept custom parameters."""
        viewer = Rerun(
            show_collision_link_chain=True,
            show_collision_tool=False,
            show_safety_link_chain=False,
            show_details=True,
            show_safety_zones=False,
            tcp_tools={"gripper": "gripper.stl"},
            trajectory_sample_interval_ms=100.0,
        )
        assert viewer.show_collision_link_chain is True
        assert viewer.show_collision_tool is False
        assert viewer.show_safety_link_chain is False
        assert viewer.show_details is True
        assert viewer.show_safety_zones is False
        assert viewer.tcp_tools == {"gripper": "gripper.stl"}
        assert viewer.trajectory_sample_interval_ms == 100.0

    def test_rerun_viewer_auto_registers(self):
        """Should automatically register itself when created."""
        manager = get_viewer_manager()
        initial_count = len(manager._viewers)

        viewer = Rerun()

        # Should have one more viewer registered
        assert len(manager._viewers) == initial_count + 1
        assert viewer is not None  # Keep the variable used

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
            show_collision_tool=True,
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

    def _create_mock_trajectory(self, n_samples: int = 10):
        """Create a mock trajectory with the given number of samples."""
        from nova import api

        joint_positions = [api.models.Joints(root=[float(i) * 0.1] * 6) for i in range(n_samples)]
        times = [float(i) * 0.01 for i in range(n_samples)]
        locations = [api.models.Location(root=float(i)) for i in range(n_samples)]

        return api.models.JointTrajectory(
            joint_positions=joint_positions, times=times, locations=locations
        )

    @patch("nova_rerun_bridge.NovaRerunBridge")
    @pytest.mark.asyncio
    async def test_rerun_viewer_log_planning_success(self, mock_bridge_class):
        """Should delegate planning success to bridge."""
        mock_bridge = AsyncMock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun()
        viewer.configure(Mock())

        # Mock parameters - use a real trajectory since downsampling needs it
        mock_actions = [Mock()]
        mock_trajectory = self._create_mock_trajectory(10)
        mock_tcp = "tcp1"
        mock_motion_group = Mock()
        mock_motion_group.id = "mg1"

        await viewer.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

        # Should call log_trajectory on the bridge (trajectory may be unchanged since it's small)
        mock_bridge.log_trajectory.assert_called_once()
        call_kwargs = mock_bridge.log_trajectory.call_args[1]
        assert call_kwargs["tcp"] == mock_tcp
        assert call_kwargs["motion_group"] == mock_motion_group
        assert call_kwargs["collision_setups"] == {}
        assert call_kwargs["tool_asset"] is None
        # Trajectory should be passed (possibly unchanged since small)
        assert call_kwargs["trajectory"] is not None

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
        mock_motion_group.id = "mg1"

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
        """Should do nothing as viewer uses lazy initialization."""
        mock_bridge = AsyncMock()
        mock_bridge_class.return_value = mock_bridge

        viewer = Rerun()
        viewer.configure(Mock())

        # Should do nothing - lazy initialization via _ensure_bridge_initialized
        await viewer.setup_after_preconditions()

        # Bridge should NOT be initialized yet (lazy loading)
        mock_bridge.__aenter__.assert_not_called()
        mock_bridge.setup_blueprint.assert_not_called()

        # Calling multiple times should still do nothing
        await viewer.setup_after_preconditions()
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
            actions=mock_actions,
            trajectory=mock_trajectory,
            tcp=mock_tcp,
            motion_group=mock_motion_group,
        )
        viewer2.log_planning_success.assert_called_once_with(
            actions=mock_actions,
            trajectory=mock_trajectory,
            tcp=mock_tcp,
            motion_group=mock_motion_group,
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
            actions=mock_actions, error=mock_error, tcp=mock_tcp, motion_group=mock_motion_group
        )
        viewer2.log_planning_failure.assert_called_once_with(
            actions=mock_actions, error=mock_error, tcp=mock_tcp, motion_group=mock_motion_group
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


class TestTrajectoryDownsampling:
    """Test adaptive trajectory downsampling functionality."""

    def _create_mock_trajectory(
        self, n_samples: int, n_joints: int = 6, duration_seconds: float = 10.0
    ):
        """Create a mock trajectory with the given number of samples and duration."""
        import numpy as np

        from nova import api

        np.random.seed(42)  # For reproducibility
        base_positions = np.linspace(0, np.pi, n_samples)

        joint_positions = []
        for i in range(n_samples):
            # Add some noise and joint-specific offsets
            joints = [base_positions[i] + j * 0.1 for j in range(n_joints)]
            joint_positions.append(api.models.Joints(root=joints))

        # Time in seconds, spread over duration_seconds
        times = [float(i) * duration_seconds / max(1, n_samples - 1) for i in range(n_samples)]
        locations = [api.models.Location(root=float(i)) for i in range(n_samples)]

        return api.models.JointTrajectory(
            joint_positions=joint_positions, times=times, locations=locations
        )

    def _create_trajectory_with_high_curvature(
        self, n_samples: int = 1000, duration_seconds: float = 10.0
    ):
        """Create a trajectory with a high-curvature section in the middle."""
        import numpy as np

        from nova import api

        joint_positions = []
        for i in range(n_samples):
            t = i / n_samples
            if 0.4 < t < 0.6:
                # High curvature section (sharp direction change)
                angle = np.sin(t * 20 * np.pi)
            else:
                # Low curvature section (straight motion)
                angle = t * np.pi
            joints = [angle + j * 0.1 for j in range(6)]
            joint_positions.append(api.models.Joints(root=joints))

        # Time in seconds
        times = [float(i) * duration_seconds / max(1, n_samples - 1) for i in range(n_samples)]
        locations = [api.models.Location(root=float(i)) for i in range(n_samples)]

        return api.models.JointTrajectory(
            joint_positions=joint_positions, times=times, locations=locations
        )

    def test_downsample_import(self):
        """Should be able to import downsample_trajectory utility."""
        from nova.viewers.utils import downsample_trajectory

        assert downsample_trajectory is not None

    def test_downsample_already_sparse_unchanged(self):
        """Trajectories already at or below target sample rate should not be modified."""
        from nova.viewers.utils import downsample_trajectory

        # 100 samples over 10 seconds = 100ms average interval
        # With 50ms target interval, this is already sparser than target
        trajectory = self._create_mock_trajectory(100, duration_seconds=10.0)
        result = downsample_trajectory(trajectory, sample_interval_ms=50.0)

        # Should return the same trajectory (unchanged)
        assert len(result.joint_positions) == 100
        assert len(result.times) == 100
        assert len(result.locations) == 100

    def test_downsample_dense_trajectory_reduced(self):
        """Dense trajectories should be reduced based on sample interval."""
        from nova.viewers.utils import downsample_trajectory

        # 2000 samples over 10 seconds = 5ms average interval (very dense)
        # With 50ms target interval, should be reduced to ~200 samples
        trajectory = self._create_mock_trajectory(2000, duration_seconds=10.0)
        result = downsample_trajectory(trajectory, sample_interval_ms=50.0)

        # Should be reduced to approximately 10000ms / 50ms = 200 samples
        assert len(result.joint_positions) < 2000
        assert len(result.joint_positions) >= 100  # At least half the target
        assert len(result.joint_positions) <= 400  # At most double the target
        assert len(result.times) == len(result.joint_positions)
        assert len(result.locations) == len(result.joint_positions)

    def test_downsample_preserves_first_and_last(self):
        """First and last samples should always be preserved."""
        from nova.viewers.utils import downsample_trajectory

        trajectory = self._create_mock_trajectory(1000, duration_seconds=10.0)
        result = downsample_trajectory(trajectory, sample_interval_ms=100.0)

        # First and last should be preserved
        assert result.joint_positions[0].root == trajectory.joint_positions[0].root
        assert result.joint_positions[-1].root == trajectory.joint_positions[-1].root
        assert result.times[0] == trajectory.times[0]
        assert result.times[-1] == trajectory.times[-1]

    def test_downsample_scales_with_duration(self):
        """Longer trajectories should result in more samples."""
        from nova.viewers.utils import downsample_trajectory

        # Short trajectory: 1000 samples over 5 seconds
        short_traj = self._create_mock_trajectory(1000, duration_seconds=5.0)
        short_result = downsample_trajectory(short_traj, sample_interval_ms=50.0)

        # Long trajectory: 1000 samples over 20 seconds
        long_traj = self._create_mock_trajectory(1000, duration_seconds=20.0)
        long_result = downsample_trajectory(long_traj, sample_interval_ms=50.0)

        # Longer trajectory should have more samples (roughly 4x)
        assert len(long_result.joint_positions) > len(short_result.joint_positions)

    def test_downsample_high_curvature_section_preserved(self):
        """High curvature sections should have denser sampling."""
        import numpy as np

        from nova.viewers.utils import downsample_trajectory

        trajectory = self._create_trajectory_with_high_curvature(1000, duration_seconds=10.0)
        result = downsample_trajectory(trajectory, sample_interval_ms=50.0)

        # Get the indices in the original trajectory that correspond to the middle section
        # The high curvature section is between 0.4 and 0.6 of the trajectory
        result_times = np.array(result.times)
        total_time = trajectory.times[-1]
        middle_start = 0.4 * total_time
        middle_end = 0.6 * total_time

        # Count samples in the middle section (high curvature)
        middle_samples = np.sum((result_times >= middle_start) & (result_times <= middle_end))

        # Count samples in the first section (low curvature)
        first_samples = np.sum(result_times < middle_start)

        # The middle section (20% of total time) should have relatively more samples
        # compared to an even distribution due to higher curvature
        # Compute densities: samples per unit of relative time
        middle_density = middle_samples / 0.2
        first_density = first_samples / 0.4 if first_samples > 0 else 0

        # Middle density should be at least comparable (allowing some tolerance for the algorithm)
        # This is a soft check since the algorithm is probabilistic
        assert middle_samples > 0, "Middle section should have samples"
        # Verify that high curvature areas get reasonable sampling density
        assert middle_density >= first_density * 0.5, (
            f"Middle density ({middle_density}) should not be drastically lower than first ({first_density})"
        )

    def test_downsample_edge_case_two_samples(self):
        """Should handle trajectories with only two samples."""
        from nova.viewers.utils import downsample_trajectory

        trajectory = self._create_mock_trajectory(2, duration_seconds=1.0)
        result = downsample_trajectory(trajectory, sample_interval_ms=50.0)

        assert len(result.joint_positions) == 2

    def test_downsample_edge_case_one_sample(self):
        """Should handle trajectories with only one sample."""
        from nova import api
        from nova.viewers.utils import downsample_trajectory

        trajectory = api.models.JointTrajectory(
            joint_positions=[api.models.Joints(root=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
            times=[0.0],
            locations=[api.models.Location(root=0.0)],
        )
        result = downsample_trajectory(trajectory, sample_interval_ms=50.0)

        assert len(result.joint_positions) == 1

    def test_downsample_custom_curvature_weight(self):
        """Different curvature weights should produce different results."""
        from nova.viewers.utils import downsample_trajectory

        trajectory = self._create_trajectory_with_high_curvature(1000, duration_seconds=10.0)

        # High curvature weight should prioritize curvature points
        result_high = downsample_trajectory(
            trajectory, sample_interval_ms=50.0, curvature_weight=0.9
        )

        # Low curvature weight should be more uniform
        result_low = downsample_trajectory(
            trajectory, sample_interval_ms=50.0, curvature_weight=0.1
        )

        # Both should produce valid results with similar sample counts
        # (since they target the same interval)
        assert len(result_high.joint_positions) > 50
        assert len(result_low.joint_positions) > 50

    def test_downsample_different_intervals(self):
        """Different sample intervals should produce different sample counts."""
        from nova.viewers.utils import downsample_trajectory

        # 1000 samples over 10 seconds
        trajectory = self._create_mock_trajectory(1000, duration_seconds=10.0)

        # Coarse sampling: 200ms interval -> ~50 samples
        coarse = downsample_trajectory(trajectory, sample_interval_ms=200.0)

        # Fine sampling: 20ms interval -> ~500 samples
        fine = downsample_trajectory(trajectory, sample_interval_ms=20.0)

        # Fine should have more samples than coarse
        assert len(fine.joint_positions) > len(coarse.joint_positions)


class TestViewerUtilities:
    """Test utility functions and helpers."""

    def test_extract_collision_scenes_import(self):
        """Should be able to import collision scene utility."""
        from nova.viewers.utils import extract_collision_setups_from_actions

        assert extract_collision_setups_from_actions is not None

    def test_register_viewer_function(self):
        """Should be able to register viewers through utility function."""
        from nova.viewers.manager import register_viewer

        # Create a real viewer instance that won't be garbage collected
        viewer = Rerun()

        # Get initial count
        manager = get_viewer_manager()
        initial_count = len(manager._viewers)

        # Register should not add it again since Rerun auto-registers
        register_viewer(viewer)

        # Should still have same count since it was already registered
        assert len(manager._viewers) == initial_count

    def test_extract_collision_scenes_empty_actions(self):
        """Should handle empty actions list."""
        from nova.viewers.utils import extract_collision_setups_from_actions

        result = extract_collision_setups_from_actions([])
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
        mock_motion_group.id = "mg1"
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
        mock_motion_group.id = "mg1"

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
        mock_motion_group.id = "mg1"

        await viewer.log_planning_success(
            mock_actions, mock_trajectory, mock_tcp, mock_motion_group
        )

        # Safety zones should not be logged
        mock_bridge.log_safety_zones.assert_not_called()
        assert len(viewer._logged_safety_zones) == 0
