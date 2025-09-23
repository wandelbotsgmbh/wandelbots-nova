import pytest

from nova.cell.robot_cell import RobotCell
from nova.cell.simulation import SimulatedRobotCell
from nova.program.exceptions import NotPlannableError
from nova.program.runner import ProgramRunner, ProgramRunState


class TestProgramRunner(ProgramRunner):
    """Concrete implementation of ProgramRunner for testing"""

    def __init__(
        self,
        program_id: str,
        args: dict,
        should_fail: bool = False,
        should_not_plannable: bool = False,
        robot_cell_override: RobotCell = SimulatedRobotCell(),
    ):
        super().__init__(program_id=program_id, args=args, robot_cell_override=robot_cell_override)
        self._should_fail = should_fail
        self._should_not_plannable = should_not_plannable

    async def _run(self, execution_context):
        if self._should_not_plannable:
            raise NotPlannableError(location=None, value="Test not plannable error")
        if self._should_fail:
            raise RuntimeError("Test failure")
        self._program_run.state = ProgramRunState.RUNNING


def test_program_runner_initialization():
    # Test basic initialization
    runner = TestProgramRunner(program_id="test", args={})

    assert runner.run_id is not None
    assert runner.state == ProgramRunState.PREPARING
    assert not runner.is_running()


def test_program_runner_state_transitions():
    runner = TestProgramRunner(program_id="test", args={})

    # Test state transitions
    assert runner.state == ProgramRunState.PREPARING
    runner.start(sync=True)
    assert runner.state == ProgramRunState.COMPLETED


def test_program_runner_stop():
    runner = TestProgramRunner(program_id="test", args={})

    # Test stopping before start
    with pytest.raises(RuntimeError):
        runner.stop()

    # Test stopping after start
    runner.start(sync=True)
    assert runner.state == ProgramRunState.COMPLETED
    assert not runner.is_running()


def test_program_runner_double_start():
    runner = TestProgramRunner(program_id="test", args={})

    # Test starting twice
    runner.start(sync=True)
    with pytest.raises(RuntimeError):
        runner.start()


def test_program_runner_error_handling():
    # Test general exception handling
    with pytest.raises(RuntimeError):
        runner = TestProgramRunner(program_id="test", args={}, should_fail=True)
        runner.start(sync=True)
        assert runner.state == ProgramRunState.FAILED
        assert runner.program_run.error is not None
        assert runner.program_run.traceback is not None
        assert runner.is_running() is False

    # Test NotPlannableError handling
    with pytest.raises(NotPlannableError):
        runner = TestProgramRunner(program_id="test", args={}, should_not_plannable=True)
        runner.start(sync=True)
        assert runner.state == ProgramRunState.FAILED
        assert "NotPlannableError" in runner.program_run.error
        assert runner.is_running() is False


def test_program_runner_logs_and_stdout():
    runner = TestProgramRunner(program_id="test", args={})

    # Test initial state
    assert runner.program_run.logs is None
    assert runner.program_run.stdout is None

    # Run the program
    runner.start(sync=True)

    # Verify logs and stdout are captured
    assert runner.program_run.logs is not None
    assert runner.program_run.stdout is not None
    assert runner.is_running() is False


# TODO: this test is failing because the runner only captures logs with the loguru
#       can we capture everything from the thread?
#       since we have this feature to capture program output and report it to other systems
#       what kind of output do we want to capture? logger? print? both?
def test_program_runner_contains_logs():
    class DummyRunner(TestProgramRunner):
        def __init__(self, program_id, args):
            super().__init__(program_id, args)

        async def _run(self, execution_context):
            print("Hello from DummyRunner")
            from loguru import logger

            logger.info("Hello from DummyRunner")

    runner = DummyRunner(program_id="test", args={})

    # Run the program
    runner.start(sync=True)

    # After running, logs should be present
    assert runner.program_run.logs.find("Hello from DummyRunner") != -1
