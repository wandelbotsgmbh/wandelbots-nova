import pytest

import nova
from nova import api, run_program
from nova.program.exceptions import NotPlannableError
from nova.program.function import Program
from nova.program.runner import ProgramRunner


@nova.program()
async def hello_world_program():
    print("Hello, world")


class TestProgramRunner(ProgramRunner):
    """Concrete implementation of ProgramRunner for testing"""

    def __init__(
        self,
        program: Program,
        parameters: dict,
        should_fail: bool = False,
        should_not_plannable: bool = False,
    ):
        super().__init__(program, parameters=parameters)
        self._should_fail = should_fail
        self._should_not_plannable = should_not_plannable

    async def _run(self, execution_context):
        if self._should_not_plannable:
            raise NotPlannableError(location=None, value="Test not plannable error")
        if self._should_fail:
            raise RuntimeError("Test failure")
        self._program_run.state = api.models.ProgramRunState.RUNNING


def test_program_runner_initialization():
    # Test basic initialization
    runner = TestProgramRunner(hello_world_program, parameters={})

    assert runner.run_id is not None
    assert runner.state == api.models.ProgramRunState.PREPARING
    assert not runner.is_running()


@pytest.mark.integration
def test_program_runner_state_transitions() -> None:
    runner = TestProgramRunner(hello_world_program, parameters={})

    # Test state transitions
    assert runner.state == api.models.ProgramRunState.PREPARING
    runner.start(sync=True)
    assert runner.state == api.models.ProgramRunState.COMPLETED  # type: ignore


@pytest.mark.integration
def test_program_runner_stop():
    runner = TestProgramRunner(hello_world_program, parameters={})

    # Test stopping before start
    with pytest.raises(RuntimeError):
        runner.stop()

    # Test stopping after start
    runner.start(sync=True)
    assert runner.state == api.models.ProgramRunState.COMPLETED
    assert not runner.is_running()


@pytest.mark.integration
def test_program_runner_double_start():
    runner = TestProgramRunner(hello_world_program, parameters={})

    # Test starting twice
    runner.start(sync=True)
    with pytest.raises(RuntimeError):
        runner.start()


@pytest.mark.integration
def test_program_runner_error_handling():
    # Test general exception handling
    with pytest.raises(RuntimeError):
        runner = TestProgramRunner(hello_world_program, parameters={}, should_fail=True)
        runner.start(sync=True)
        assert runner.state == api.models.ProgramRunState.FAILED
        assert runner.program_run.error is not None
        assert runner.program_run.traceback is not None

    # Test NotPlannableError handling
    with pytest.raises(NotPlannableError):
        runner = TestProgramRunner(hello_world_program, parameters={}, should_not_plannable=True)
        runner.start(sync=True)
        assert runner.state == api.models.ProgramRunState.FAILED
        assert "NotPlannableError" in runner.program_run.error


@pytest.mark.integration
def test_program_runner_logs_and_stdout():
    runner = TestProgramRunner(hello_world_program, parameters={})

    # Test initial state
    assert runner.program_run.logs is None
    assert runner.program_run.stdout is None

    # Run the program
    runner.start(sync=True)

    # Verify logs and stdout are captured
    assert runner.program_run.logs is not None
    assert runner.program_run.stdout is not None


@pytest.mark.integration
def test_simple_program():
    @nova.program()
    async def test_program():
        print("Hello, world")

    runner = run_program(test_program)
    assert runner.state == api.models.ProgramRunState.COMPLETED
    assert runner.program_run.error is None
