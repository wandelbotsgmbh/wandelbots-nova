"""Test the new viewer parameter in program decorator."""

import nova
from nova import viewers


def test_program_with_rerun_viewer():
    """Test that program decorator accepts viewer parameter."""

    # Test that we can create a Rerun viewer
    rerun_viewer = viewers.Rerun(application_id="test-app")
    assert rerun_viewer.application_id == "test-app"
    assert rerun_viewer.spawn is True

    # Test that program decorator accepts viewer
    @nova.program(
        name="Test Program", viewer=rerun_viewer, preconditions=nova.ProgramPreconditions()
    )
    async def test_program(ctx: nova.ProgramContext):
        return "success"

    # Check that the function was decorated properly
    assert test_program.name == "Test Program"
    assert hasattr(test_program, "_wrapped")


def test_rerun_viewer_instantiation():
    """Test that Rerun viewer can be instantiated with different parameters."""
    # Test default instantiation
    viewer = viewers.Rerun()
    assert viewer.application_id is None
    assert viewer.spawn is True

    # Test with custom parameters
    viewer_custom = viewers.Rerun(application_id="test_app", spawn=False)
    assert viewer_custom.application_id == "test_app"
    assert viewer_custom.spawn is False


def test_rerun_viewer_configure_cleanup():
    """Test Rerun viewer configure and cleanup methods don't raise errors."""
    from unittest.mock import Mock

    viewer = viewers.Rerun()
    mock_nova = Mock()

    # These methods should not raise any exceptions
    viewer.configure(mock_nova)
    viewer.cleanup()


def test_rerun_viewer_type():
    """Test that Rerun viewer is a proper Viewer instance."""
    viewer = viewers.Rerun()
    assert isinstance(viewer, viewers.Viewer)
