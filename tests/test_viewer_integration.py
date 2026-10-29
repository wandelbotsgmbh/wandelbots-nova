"""Test the new viewer parameter in program decorator."""

from unittest.mock import AsyncMock, Mock, patch

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


def test_a_program_takes_its_viewer_out_of_the_global_set():
    """Rerun() still registers itself; the program activates it only while it runs."""
    manager = viewers.get_viewer_manager()
    rerun_viewer = viewers.Rerun(spawn=False)
    assert rerun_viewer in manager.active_viewers

    @nova.program(viewer=rerun_viewer)
    async def test_program(ctx: nova.ProgramContext):
        return "success"

    assert rerun_viewer not in manager.active_viewers


def test_every_run_activates_the_viewer_and_cleans_it_up():
    """A second run in the same process is visualized too."""
    from nova.cell.simulation import SimulatedRobotCell
    from nova.program.runner import PythonProgramRunner

    viewer = Mock(spec=viewers.Viewer)
    active_during_runs = []

    @nova.program(viewer=viewer)
    async def test_program(ctx: nova.ProgramContext):
        active_during_runs.append(viewers.get_viewer_manager().active_viewers)

    for _ in range(2):
        PythonProgramRunner(test_program, robot_cell_override=SimulatedRobotCell()).start(sync=True)

    assert active_during_runs == [(viewer,), (viewer,)]
    assert viewer.cleanup.call_count == 2
    assert viewer not in viewers.get_viewer_manager().active_viewers


async def test_a_directly_awaited_program_activates_and_cleans_up_its_viewer():
    """Awaiting a program with ``nova=...`` has no runner, so the program owns its viewer."""
    viewer = Mock(spec=viewers.Viewer)
    nova_instance = Mock()
    active_during_run = []

    @nova.program(viewer=viewer)
    async def test_program(ctx: nova.ProgramContext):
        active_during_run.extend(viewers.get_viewer_manager().active_viewers)

    await test_program(nova=nova_instance)
    await test_program(nova=nova_instance)

    assert active_during_run == [viewer, viewer]
    assert viewer.configure.call_count == 2
    viewer.configure.assert_called_with(nova_instance)
    assert viewer.cleanup.call_count == 2
    assert viewer not in viewers.get_viewer_manager().active_viewers


def test_an_operator_run_configures_its_viewer_before_flagging_operator_execution():
    """Nova.open() configures the viewers; Rerun skips its setup once the flag is set."""
    from nova.program.runner import PythonProgramRunner, is_operator_execution_var

    flag_seen = {}

    class _FakeNova:
        def __init__(self, config=None):
            pass

        async def open(self):
            flag_seen["at_open"] = is_operator_execution_var.get()

        async def close(self):
            pass

        def is_connected(self):
            return False

        def cell(self):
            return Mock()

    @nova.program()
    async def test_program(ctx: nova.ProgramContext):
        flag_seen["in_program"] = is_operator_execution_var.get()

    with (
        patch("nova.program.runner.Nova", _FakeNova),
        patch("nova.program.runner._ensure_preconditions", AsyncMock(return_value=[])),
        patch("nova.program.runner._cleanup_preconditions", AsyncMock()),
    ):
        PythonProgramRunner(test_program, app_name="novax").start(sync=True)

    assert flag_seen == {"at_open": False, "in_program": True}


def test_rerun_cleanup_releases_its_bridge_for_the_next_run():
    viewer = viewers.Rerun(spawn=False)
    bridge = Mock()
    viewer._bridge = bridge
    viewer._bridge_initialized = True

    viewer.cleanup()

    bridge.close.assert_called_once()
    assert viewer._bridge is None
    assert viewer._bridge_initialized is False
    viewers.get_viewer_manager().cleanup_viewer(viewer)


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
