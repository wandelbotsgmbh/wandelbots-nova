"""Tests for the direct rerun integration."""


def test_is_rerun_enabled_default():
    """Test is_rerun_enabled returns correct state."""
    from nova.rerun_integration import is_rerun_enabled

    # Should be enabled by default (environment variable)
    result = is_rerun_enabled()
    assert isinstance(result, bool)


def test_configure_rerun_default():
    """Test configure_rerun with default parameters."""
    from nova.rerun_integration import configure_rerun, is_rerun_enabled

    configure_rerun()

    assert is_rerun_enabled() is True


def test_configure_rerun_with_params():
    """Test configure_rerun with custom parameters."""
    from nova.rerun_integration import configure_rerun, is_rerun_enabled

    configure_rerun(enabled=False)

    assert is_rerun_enabled() is False

    # Reset to enabled for other tests
    configure_rerun(enabled=True)


def test_enable_rerun():
    """Test enable_rerun function."""
    from nova.rerun_integration import enable_rerun, is_rerun_enabled

    enable_rerun(True)

    assert is_rerun_enabled() is True


def test_disable_rerun():
    """Test disable_rerun function."""
    from nova.rerun_integration import disable_rerun, is_rerun_enabled

    disable_rerun()

    assert is_rerun_enabled() is False

    # Reset for other tests
    from nova.rerun_integration import enable_rerun

    enable_rerun(True)


def test_log_trajectory_when_disabled():
    """Test log_trajectory does nothing when disabled."""
    from nova.rerun_integration import configure_rerun, log_trajectory

    configure_rerun(enabled=False)

    # Should not raise any exception
    log_trajectory(trajectory={"test": "data"}, motion_group=None, tcp="Flange")


def test_log_trajectory_when_enabled():
    """Test log_trajectory works when enabled."""
    from nova.rerun_integration import configure_rerun, log_trajectory

    configure_rerun(enabled=True)
    test_trajectory = {"joint_positions": [], "times": []}

    # Should not raise any exception
    log_trajectory(trajectory=test_trajectory, motion_group=None, tcp="Flange")


def test_log_trajectory_handles_exceptions():
    """Test log_trajectory handles exceptions gracefully."""
    from nova.rerun_integration import log_trajectory

    # Should not raise exception due to internal error handling
    log_trajectory(trajectory={"test": "data"}, motion_group=None, tcp="Flange")


def test_log_trajectory_with_params():
    """Test log_trajectory with various parameters."""
    from nova.rerun_integration import configure_rerun, log_trajectory

    configure_rerun(enabled=True)
    test_trajectory = {"joint_positions": [[1, 2, 3]], "times": [0.1]}

    # Should not raise any exception
    log_trajectory(trajectory=test_trajectory, motion_group=None, tcp="CustomTCP")


def test_log_trajectory_with_no_bridge():
    """Test log_trajectory handles missing bridge gracefully."""
    from nova.rerun_integration import configure_rerun, log_trajectory

    configure_rerun(enabled=True)

    # No bridge available, should not raise exceptions due to internal error handling
    log_trajectory(trajectory={}, motion_group=None, tcp="Flange")


def test_import_integration_from_nova():
    """Test that rerun functions can be imported from nova package."""
    from nova import configure_rerun, disable_rerun, enable_rerun

    # Should not raise any exceptions
    assert callable(configure_rerun)
    assert callable(enable_rerun)
    assert callable(disable_rerun)


def test_full_workflow():
    """Test a complete workflow from config to logging."""
    from nova.rerun_integration import configure_rerun, log_trajectory

    # Configure
    configure_rerun(enabled=True)

    # Log sequence - should not raise any exceptions
    trajectory = {"joint_positions": [[1, 2, 3]], "times": [0.1]}

    log_trajectory(trajectory=trajectory, motion_group=None, tcp="Flange")

    # No exceptions should be raised
    assert True


def test_enable_disable_workflow():
    """Test enabling and disabling rerun in a workflow."""
    from nova.rerun_integration import (
        configure_rerun,
        disable_rerun,
        enable_rerun,
        is_rerun_enabled,
        log_trajectory,
    )

    # Start enabled
    enable_rerun(True)
    assert is_rerun_enabled() is True

    # Log should work (no exceptions)
    log_trajectory(trajectory={"test": "data"}, motion_group=None, tcp="Flange")

    # Disable
    disable_rerun()
    assert is_rerun_enabled() is False

    # Log should still work (no exceptions) but do nothing
    log_trajectory(trajectory={"test": "data"}, motion_group=None, tcp="Flange")

    # Re-enable via configure
    configure_rerun(enabled=True)
    assert is_rerun_enabled() is True
