"""Test viewer configuration in the program decorator."""

from unittest.mock import Mock

import nova
from nova import viewers


def test_program_with_rerun_viewer():
    """Test that program decorator accepts viewer parameter."""

    # Test that we can create a Rerun viewer
    rerun_viewer = viewers.Rerun(application_id="test-app")
    manager = viewers.get_viewer_manager()
    assert rerun_viewer.application_id == "test-app"
    assert rerun_viewer.spawn is True
    assert rerun_viewer in manager.active_viewers

    # The decorator claims the legacy registration for execution-scoped activation.
    @nova.program(
        name="Test Program", viewer=rerun_viewer, preconditions=nova.ProgramPreconditions()
    )
    async def test_program(ctx: nova.ProgramContext):
        return "success"

    # Check that the function was decorated properly without leaving a global viewer.
    assert test_program.name == "Test Program"
    assert hasattr(test_program, "_wrapped")
    assert rerun_viewer not in manager.active_viewers


def test_program_viewer_factory_has_no_decoration_side_effect():
    manager = viewers.get_viewer_manager()
    manager.cleanup_viewers()

    @nova.program(viewer=viewers.Rerun)
    async def test_program(ctx: nova.ProgramContext):
        return "success"

    assert test_program._viewer is viewers.Rerun
    assert not manager.active_viewers


def test_program_viewer_creates_a_fresh_viewer_per_execution():
    viewer1 = Mock(spec=viewers.Viewer)
    viewer2 = Mock(spec=viewers.Viewer)
    factory = Mock(side_effect=[viewer1, viewer2])

    @nova.program(viewer=factory)
    async def test_program(ctx: nova.ProgramContext):
        return "success"

    assert test_program._create_viewer() is viewer1
    assert test_program._create_viewer() is viewer2
    assert factory.call_count == 2


def test_program_runner_scopes_and_cleans_factory_viewer():
    from nova import ProgramContext, program
    from nova.cell.simulation import SimulatedRobotCell
    from nova.program.runner import PythonProgramRunner
    from nova.viewers import Viewer, get_viewer_manager

    get_viewer_manager().cleanup_viewers()
    viewer = Mock(spec=Viewer)
    active_during_run = []

    @program(viewer=lambda: viewer)
    async def test_program(ctx: ProgramContext):
        # Resolve inside the runner thread so module-reload tests cannot leave us
        # holding an obsolete singleton reference.
        from nova.viewers import get_viewer_manager

        active_during_run.extend(get_viewer_manager().active_viewers)

    runner = PythonProgramRunner(test_program, robot_cell_override=SimulatedRobotCell())
    runner.start(sync=True)

    assert active_during_run == [viewer]
    viewer.cleanup.assert_called_once()
    from nova.viewers import get_viewer_manager

    assert viewer not in get_viewer_manager().active_viewers


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
