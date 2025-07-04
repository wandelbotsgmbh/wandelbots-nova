import asyncio
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from unittest.mock import AsyncMock, patch

import pytest

from nova.runtime.runner import (
    ExecutionContext,
    Program,
    ProgramRunner,
    ProgramRunState,
    ProgramType,
)


class ThreadSafeProgramRunner(ProgramRunner):
    """Mock implementation for testing thread safety"""

    def __init__(
        self, program: Program, args: dict, execution_delay: float = 0.1, robot_cell_override=None
    ):
        super().__init__(program, args, robot_cell_override)
        self._execution_delay = execution_delay
        self._execution_count = 0
        self._execution_lock = threading.Lock()

    async def _run(self, execution_context: ExecutionContext):
        with self._execution_lock:
            self._execution_count += 1
        await asyncio.sleep(self._execution_delay)


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


def test_concurrent_property_access(sample_program):
    """Test that properties can be safely accessed from multiple threads"""
    runner = ThreadSafeProgramRunner(sample_program, {})
    results = {}
    errors = []

    def access_properties(thread_id):
        try:
            for _ in range(100):
                _ = runner.id
                _ = runner.state
                _ = runner.program_run
                _ = runner.stopped
                _ = runner.is_running()
            results[thread_id] = "success"
        except Exception as e:
            errors.append(f"Thread {thread_id}: {e}")

    threads = []
    for i in range(10):
        thread = threading.Thread(target=access_properties, args=(i,))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

    assert len(errors) == 0, f"Errors occurred: {errors}"
    assert len(results) == 10
    assert all(result == "success" for result in results.values())


def test_thread_creation_and_cleanup(sample_program):
    """Test that threads are properly created and cleaned up"""
    runner = ThreadSafeProgramRunner(sample_program, {})

    assert runner._thread is None

    with patch.object(runner, "_run_program", new_callable=AsyncMock):
        runner.start(sync=True)

        assert runner._thread is not None
        assert not runner._thread.is_alive()


def test_concurrent_start_attempts(sample_program, mock_robot_cell):
    """Test that concurrent start attempts are properly handled"""
    runner = ThreadSafeProgramRunner(
        sample_program, {}, execution_delay=0.1, robot_cell_override=mock_robot_cell
    )

    results = []
    errors = []

    def try_start(thread_id):
        try:
            runner.start(sync=True)
            results.append(f"Thread {thread_id}: success")
        except RuntimeError as e:
            if "not in the not_started state" in str(e):
                results.append(f"Thread {thread_id}: correctly rejected")
            else:
                errors.append(f"Thread {thread_id}: unexpected error - {e}")
        except Exception as e:
            errors.append(f"Thread {thread_id}: unexpected error - {e}")

    first_thread = threading.Thread(target=try_start, args=(0,))
    first_thread.start()

    time.sleep(0.05)

    threads = []
    for i in range(1, 4):
        thread = threading.Thread(target=try_start, args=(i,))
        threads.append(thread)
        thread.start()

    first_thread.join()
    for thread in threads:
        thread.join()

    assert len(errors) == 0, f"Unexpected errors: {errors}"
    assert len(results) == 4

    success_count = sum(1 for result in results if "success" in result)
    rejected_count = sum(1 for result in results if "correctly rejected" in result)

    assert success_count == 1, f"Expected 1 success, got {success_count}"
    assert rejected_count == 3, f"Expected 3 rejections, got {rejected_count}"


def test_stop_thread_safety(sample_program, mock_robot_cell):
    """Test that stop() can be safely called from different threads"""
    runner = ThreadSafeProgramRunner(
        sample_program, {}, execution_delay=1.0, robot_cell_override=mock_robot_cell
    )

    runner.start(sync=False)

    time.sleep(0.2)

    stop_results = []
    stop_errors = []

    def try_stop(thread_id):
        try:
            if runner.is_running():
                runner.stop(sync=False)
                stop_results.append(f"Thread {thread_id}: stop called")
            else:
                stop_results.append(f"Thread {thread_id}: not running")
        except Exception as e:
            stop_errors.append(f"Thread {thread_id}: {e}")

    stop_threads = []
    for i in range(3):
        thread = threading.Thread(target=try_stop, args=(i,))
        stop_threads.append(thread)
        thread.start()

    for thread in stop_threads:
        thread.join()

    runner.join()

    assert len(stop_errors) == 0, f"Stop errors: {stop_errors}"
    assert len(stop_results) == 3
    assert runner.state in [ProgramRunState.STOPPED, ProgramRunState.COMPLETED]


def test_state_access_during_transitions(sample_program):
    """Test that state can be safely accessed during state transitions"""
    runner = ThreadSafeProgramRunner(sample_program, {})

    state_readings = []
    reading_errors = []

    def read_state_continuously():
        try:
            for _ in range(200):  # Read state many times
                state = runner.state
                is_running = runner.is_running()
                stopped = runner.stopped
                state_readings.append((state, is_running, stopped))
                time.sleep(0.001)  # Small delay
        except Exception as e:
            reading_errors.append(str(e))

    reader_thread = threading.Thread(target=read_state_continuously)
    reader_thread.start()

    with patch.object(runner, "_run_program", new_callable=AsyncMock):
        runner.start(sync=True)

    reader_thread.join()

    assert len(reading_errors) == 0, f"State reading errors: {reading_errors}"
    assert len(state_readings) > 0

    for state, is_running, stopped in state_readings:
        if state == ProgramRunState.RUNNING:
            pass  # This is acceptable due to timing
        elif state in [ProgramRunState.COMPLETED, ProgramRunState.FAILED, ProgramRunState.STOPPED]:
            assert not is_running, f"is_running should be False when state is {state}"


def test_execution_count_thread_safety(sample_program, mock_robot_cell):
    """Test that internal counters are thread-safe"""
    runner = ThreadSafeProgramRunner(sample_program, {}, robot_cell_override=mock_robot_cell)

    runner.start(sync=True)

    assert runner._execution_count == 1


def test_multiple_runners_independence(sample_program, mock_robot_cell):
    """Test that multiple runners don't interfere with each other"""
    runners = [
        ThreadSafeProgramRunner(sample_program, {}, robot_cell_override=mock_robot_cell)
        for _ in range(5)
    ]

    results = {}
    errors = []

    def run_program(runner_id, runner):
        try:
            runner.start(sync=True)
            results[runner_id] = runner.state
        except Exception as e:
            errors.append(f"Runner {runner_id}: {e}")

    threads = []
    for i, runner in enumerate(runners):
        thread = threading.Thread(target=run_program, args=(i, runner))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

    assert len(errors) == 0, f"Errors: {errors}"
    assert len(results) == 5

    for runner_id, state in results.items():
        assert state == ProgramRunState.COMPLETED, f"Runner {runner_id} state: {state}"

    ids = [runner.id for runner in runners]
    assert len(set(ids)) == len(ids), "Runner IDs should be unique"


def test_join_thread_safety(sample_program, mock_robot_cell):
    """Test that join() can be safely called from multiple threads"""
    runner = ThreadSafeProgramRunner(
        sample_program, {}, execution_delay=0.2, robot_cell_override=mock_robot_cell
    )

    runner.start(sync=False)

    join_results = []
    join_errors = []

    def try_join(thread_id):
        try:
            runner.join()
            join_results.append(f"Thread {thread_id}: joined successfully")
        except Exception as e:
            join_errors.append(f"Thread {thread_id}: {e}")

    join_threads = []
    for i in range(3):
        thread = threading.Thread(target=try_join, args=(i,))
        join_threads.append(thread)
        thread.start()

    for thread in join_threads:
        thread.join()

    assert len(join_errors) == 0, f"Join errors: {join_errors}"
    assert len(join_results) == 3
    assert runner.state == ProgramRunState.COMPLETED


def test_stop_event_thread_safety(sample_program, mock_robot_cell):
    """Test that stop event handling is thread-safe"""
    runner = ThreadSafeProgramRunner(sample_program, {}, robot_cell_override=mock_robot_cell)

    assert runner._stop_event is None
    assert not runner.stopped

    runner.start(sync=False)

    time.sleep(0.1)

    stopped_results = []

    def check_stopped(thread_id):
        for _ in range(50):
            stopped_results.append(runner.stopped)
            time.sleep(0.001)

    check_threads = []
    for i in range(3):
        thread = threading.Thread(target=check_stopped, args=(i,))
        check_threads.append(thread)
        thread.start()

    for thread in check_threads:
        thread.join()

    runner.join()

    assert len(stopped_results) > 0
    assert all(isinstance(result, bool) for result in stopped_results)
