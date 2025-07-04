import asyncio
import time
from unittest.mock import AsyncMock

import pytest

from nova.runtime.exceptions import NotPlannableError
from nova.runtime.runner import (
    ExecutionContext,
    Program,
    ProgramRunner,
    ProgramRunState,
    ProgramType,
)


class MockProgramRunner(ProgramRunner):
    """Mock implementation of ProgramRunner for testing state transitions"""

    def __init__(
        self,
        program: Program,
        args: dict,
        should_fail: bool = False,
        should_not_plannable: bool = False,
        execution_delay: float = 0.1,
        robot_cell_override=None,
    ):
        super().__init__(program, args, robot_cell_override)
        self._should_fail = should_fail
        self._should_not_plannable = should_not_plannable
        self._execution_delay = execution_delay

    async def _run(self, execution_context: ExecutionContext):
        await asyncio.sleep(self._execution_delay)

        if self._should_not_plannable:
            raise NotPlannableError(location=None, value="Test not plannable error")
        if self._should_fail:
            raise RuntimeError("Test failure")


@pytest.fixture
def sample_program():
    return Program(content="test program", program_type=ProgramType.PYTHON)


@pytest.fixture
def mock_robot_cell():
    mock_cell = AsyncMock()
    mock_cell.__aenter__ = AsyncMock(return_value=mock_cell)
    mock_cell.__aexit__ = AsyncMock(return_value=None)
    mock_cell.stop = AsyncMock()

    def mock_stream_state(*args, **kwargs):
        async def empty_async_iterator():
            return
            yield  # This line is never reached, but makes it a generator

        return empty_async_iterator()

    mock_cell.stream_state = mock_stream_state
    return mock_cell


def test_initial_state(sample_program):
    """Test that runner starts in NOT_STARTED state with correct initial values"""
    runner = MockProgramRunner(sample_program, {})

    assert runner.state == ProgramRunState.NOT_STARTED
    assert runner.id is not None
    assert len(runner.id) > 0
    assert not runner.is_running()
    assert not runner.stopped
    assert runner.program_run.state == ProgramRunState.NOT_STARTED
    assert runner.program_run.start_time is None
    assert runner.program_run.end_time is None
    assert runner.program_run.logs is None
    assert runner.program_run.stdout is None
    assert runner.program_run.error is None
    assert runner.program_run.traceback is None


def test_successful_state_transition(sample_program, mock_robot_cell):
    """Test successful state transition: NOT_STARTED -> RUNNING -> COMPLETED"""
    runner = MockProgramRunner(
        sample_program, {}, execution_delay=0.05, robot_cell_override=mock_robot_cell
    )

    assert runner.state == ProgramRunState.NOT_STARTED

    runner.start(sync=True)

    assert runner.state == ProgramRunState.COMPLETED
    assert runner.program_run.start_time is not None
    assert runner.program_run.end_time is not None
    assert runner.program_run.start_time <= runner.program_run.end_time
    assert not runner.is_running()


def test_failed_state_transition(sample_program, mock_robot_cell):
    """Test failed state transition: NOT_STARTED -> RUNNING -> FAILED"""
    runner = MockProgramRunner(
        sample_program,
        {},
        should_fail=True,
        execution_delay=0.05,
        robot_cell_override=mock_robot_cell,
    )

    assert runner.state == ProgramRunState.NOT_STARTED

    with pytest.raises(RuntimeError):
        runner.start(sync=True)

    assert runner.state == ProgramRunState.FAILED
    assert runner.program_run.error is not None
    assert "RuntimeError" in runner.program_run.error
    assert "Test failure" in runner.program_run.error
    assert runner.program_run.traceback is not None
    assert not runner.is_running()


def test_not_plannable_state_transition(sample_program, mock_robot_cell):
    """Test NotPlannableError state transition: NOT_STARTED -> RUNNING -> FAILED"""
    runner = MockProgramRunner(
        sample_program,
        {},
        should_not_plannable=True,
        execution_delay=0.05,
        robot_cell_override=mock_robot_cell,
    )

    assert runner.state == ProgramRunState.NOT_STARTED

    with pytest.raises(NotPlannableError):
        runner.start(sync=True)

    assert runner.state == ProgramRunState.FAILED
    assert runner.program_run.error is not None
    assert "NotPlannableError" in runner.program_run.error
    assert not runner.is_running()


def test_double_start_prevention(sample_program, mock_robot_cell):
    """Test that starting a runner twice raises RuntimeError"""
    runner = MockProgramRunner(sample_program, {}, robot_cell_override=mock_robot_cell)

    runner.start(sync=True)

    with pytest.raises(RuntimeError, match="not in the not_started state"):
        runner.start()


def test_invalid_state_operations(sample_program):
    """Test that operations on invalid states raise appropriate errors"""
    runner = MockProgramRunner(sample_program, {})

    with pytest.raises(RuntimeError, match="Program is not running"):
        runner.stop()

    with pytest.raises(AttributeError):
        runner.join()


def test_stop_during_execution(sample_program, mock_robot_cell):
    """Test stopping execution during RUNNING state"""
    runner = MockProgramRunner(
        sample_program, {}, execution_delay=1.0, robot_cell_override=mock_robot_cell
    )

    runner.start(sync=False)

    time.sleep(0.2)

    if runner.is_running():
        runner.stop(sync=True)
        assert runner.state == ProgramRunState.STOPPED
        assert not runner.is_running()
    else:
        runner.join()
        assert runner.state in [
            ProgramRunState.COMPLETED,
            ProgramRunState.FAILED,
            ProgramRunState.STOPPED,
        ]


def test_state_consistency_during_transitions(sample_program, mock_robot_cell):
    """Test that state properties remain consistent during transitions"""
    runner = MockProgramRunner(sample_program, {}, robot_cell_override=mock_robot_cell)

    assert runner.state == runner.program_run.state
    assert runner.id == runner.program_run.id

    runner.start(sync=True)

    assert runner.state == runner.program_run.state
    assert runner.state == ProgramRunState.COMPLETED


def test_unique_runner_ids(sample_program):
    """Test that each runner gets a unique ID"""
    runners = [MockProgramRunner(sample_program, {}) for _ in range(10)]
    ids = [runner.id for runner in runners]

    assert len(set(ids)) == len(ids)

    assert all(isinstance(id_, str) and len(id_) > 0 for id_ in ids)


def test_execution_timing_recorded(sample_program, mock_robot_cell):
    """Test that execution start and end times are properly recorded"""
    runner = MockProgramRunner(
        sample_program, {}, execution_delay=0.1, robot_cell_override=mock_robot_cell
    )

    before_start = time.time()

    runner.start(sync=True)

    after_end = time.time()

    assert runner.program_run.start_time is not None
    assert runner.program_run.end_time is not None
    assert runner.program_run.start_time <= runner.program_run.end_time

    start_timestamp = runner.program_run.start_time.timestamp()
    end_timestamp = runner.program_run.end_time.timestamp()

    assert before_start <= start_timestamp <= after_end
    assert before_start <= end_timestamp <= after_end
    assert end_timestamp >= start_timestamp
