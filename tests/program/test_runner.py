import pytest

import nova
from nova import run_program
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
    ):
        super().__init__(program_id=program_id, args=args)
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


@pytest.mark.integration
def test_program_runner_state_transitions():
    runner = TestProgramRunner(program_id="test", args={})

    # Test state transitions
    assert runner.state == ProgramRunState.PREPARING
    runner.start(sync=True)
    assert runner.state == ProgramRunState.COMPLETED


@pytest.mark.integration
def test_program_runner_stop():
    runner = TestProgramRunner(program_id="test", args={})

    # Test stopping before start
    with pytest.raises(RuntimeError):
        runner.stop()

    # Test stopping after start
    runner.start(sync=True)
    assert runner.state == ProgramRunState.COMPLETED
    assert not runner.is_running()


@pytest.mark.integration
def test_program_runner_double_start():
    runner = TestProgramRunner(program_id="test", args={})

    # Test starting twice
    runner.start(sync=True)
    with pytest.raises(RuntimeError):
        runner.start()


@pytest.mark.integration
def test_program_runner_error_handling():
    # Test general exception handling
    with pytest.raises(RuntimeError):
        runner = TestProgramRunner(program_id="test", args={}, should_fail=True)
        runner.start(sync=True)
        assert runner.state == ProgramRunState.FAILED
        assert runner.program_run.error is not None
        assert runner.program_run.traceback is not None

    # Test NotPlannableError handling
    with pytest.raises(NotPlannableError):
        runner = TestProgramRunner(program_id="test", args={}, should_not_plannable=True)
        runner.start(sync=True)
        assert runner.state == ProgramRunState.FAILED
        assert "NotPlannableError" in runner.program_run.error


@pytest.mark.integration
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


@pytest.mark.integration
def test_program_run_with_error_state_in_cell():
    @nova.program
    def test_program():
        print("test program")

    runner = run_program(test_program)
    assert runner.state == ProgramRunState.COMPLETED
    assert runner.program_run.error is not None
    assert runner.program_run.traceback is not None
