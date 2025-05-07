import contextlib
import sys
import threading
import uuid
from abc import ABC, abstractmethod
from collections.abc import Awaitable, Callable
from datetime import datetime
from enum import Enum
from typing import Any

import anyio
from pydantic import BaseModel, Field

from nova.runtime.utils import Tee, stoppable_run
from nova.types import RobotState


class ProgramType(Enum):
    WANDELSCRIPT = "WANDELSCRIPT"
    PYTHON = "PYTHON"
    JOINT_TRAJECTORY = "JOINT_TRAJECTORY"


class Program(BaseModel):
    content: str = Field(..., title="Program content")
    program_type: ProgramType = Field(..., description="Type of the program.", title="Program type")


class ProgramRunState(Enum):
    not_started = "not started"
    running = "running"
    completed = "completed"
    failed = "failed"
    stopped = "stopped"


class ProgramRunResult(BaseModel):
    """The ProgramRunResult object contains the execution results of a robot.

    Arguments:
        motion_group_id: The unique id of the motion group
        motion_duration: The total execution duration of the motion group
        paths: The paths of the motion group as list of Path objects

    """

    motion_group_id: str = Field(..., description="Unique id of the motion group that was executed")
    motion_duration: float = Field(..., description="Total execution duration of the motion group")
    paths: list[list[RobotState]] = Field(
        ..., description="Paths of the motion group as list of Path objects"
    )


class ProgramRun(BaseModel):
    id: str = Field(..., description="Unique id of the program run")
    state: ProgramRunState = Field(..., description="State of the program run")
    logs: str | None = Field(None, description="Logs of the program run")
    stdout: str | None = Field(None, description="Stdout of the program run")
    store: dict = Field(default_factory=dict, description="Stores runtime variables of the run")
    error: str | None = Field(None, description="Error message of the program run, if any")
    traceback: str | None = Field(None, description="Traceback of the program run, if any")
    start_time: float | None = Field(None, description="Start time of the program run")
    end_time: float | None = Field(None, description="End time of the program run")
    execution_results: list[ProgramRunResult] = Field(
        default_factory=list, description="Execution results of the program run"
    )


class ProgramRunner(ABC):
    """Abstract base class for program runners.

    This class defines the interface that all program runners must implement.
    It provides the core functionality for running and managing program execution.
    """

    def __init__(
        self,
        program: Program,
        args: dict[str, Any],
        # robot_cell: RobotCell,
        # default_robot: str | None = None,
        # default_tcp: str | None = None,
        # run_args: dict[str, ElementType] | None = None,
        # foreign_functions: dict[str, ForeignFunction] | None = None,
        # use_plannable_context: bool = False,
    ):
        self._program = program
        self._args = args
        self._program_run: ProgramRun = ProgramRun(
            id=str(uuid.uuid4()),
            state=ProgramRunState.not_started,
            logs=None,
            stdout=None,
            error=None,
            traceback=None,
            start_time=None,
            end_time=None,
        )
        # TODO: should this be here?
        self._thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        self._exc: Exception | None = None

    @property
    def id(self) -> str:
        """Get the unique identifier of the program run.

        Returns:
            str: The unique identifier
        """
        return self._program_run.id

    @property
    def program_run(self) -> ProgramRun:
        """Get the current program run state and results.

        Returns:
            Any: The program run object containing execution state and results
        """
        return self._program_run

    @property
    def state(self) -> ProgramRunState:
        """Get the current state of the program run.

        Returns:
            ProgramRunState: The current state
        """
        return self._program_run.state

    @property
    def stopped(self) -> bool:
        """Check if the program has been stopped.

        Returns:
            bool: True if the program has been stopped, False otherwise
        """
        return self._stop_event.is_set()

    @property
    def start_time(self) -> datetime | None:
        """Get the start time of the program run.

        Returns:
            Optional[datetime]: The start time if the program has started, None otherwise
        """
        if self._program_run.start_time is None:
            return None
        return datetime.fromtimestamp(self._program_run.start_time)

    @property
    def execution_time(self) -> float | None:
        """Get the execution time of the program run.

        Returns:
            Optional[float]: The execution time in seconds if the program has finished, None otherwise
        """
        return self._program_run.end_time

    def is_running(self) -> bool:
        """Check if a program is currently running.

        Returns:
            bool: True if a program is running, False otherwise
        """
        return self._thread is not None and self.state is ProgramRunState.running

    def join(self):
        """Wait for the program execution to finish.

        Raises:
            Exception: If the program execution failed
        """
        self._thread.join()
        if self._exc:
            raise self._exc

    def stop(self, sync: bool = False):
        """Stop the program execution.

        Args:
            sync: If True, the call blocks until the program is stopped
        """
        if not self.is_running():
            raise RuntimeError("Program is not running")
        self._stop_event.set()
        if sync:
            self.join()

    def start(
        self, sync: bool = False, on_state_change: Callable[[Any], Awaitable[None]] | None = None
    ):
        """Create another thread and starts the program execution. If the program was executed already, is currently
        running, failed or was stopped a new program runner needs to be created.

        Args:
            sync: if True the execution is synchronous and the method blocks until the execution is finished
            on_state_change: callback function that is called when the state of the program runner changes

        Raises:
            RuntimeError: when the runner is not in IDLE state
        """
        # Check if another program execution is already in progress
        if self.state is not ProgramRunState.not_started:
            raise RuntimeError(
                "The runner is not in the not_started state. Create a new runner to execute again."
            )

        async def _on_state_change():
            if on_state_change is not None:
                await on_state_change(self._program_run)

        def stopper(sync_stop_event, async_stop_event):
            while not sync_stop_event.wait(0.2):
                anyio.from_thread.check_cancelled()
            anyio.from_thread.run_sync(async_stop_event.set)

        async def runner():
            self._stop_event = threading.Event()
            async_stop_event = anyio.Event()

            # TODO potential memory leak if the the program is running for a long time
            with contextlib.redirect_stdout(Tee(sys.stdout)) as stdout:
                try:
                    await stoppable_run(
                        self._run(self._robot_cell_config, async_stop_event, _on_state_change),
                        anyio.to_thread.run_sync(
                            stopper, self._stop_event, async_stop_event, abandon_on_cancel=True
                        ),
                    )
                except ExceptionGroup as eg:
                    raise eg.exceptions[0]
                self._program_run.stdout = stdout.getvalue()

        # Create new thread and runs _run
        # start a new thread
        self._thread = threading.Thread(target=anyio.run, name="ProgramRunner", args=[runner])
        self._thread.start()

        if sync:
            self.join()

    @abstractmethod
    def _run(self):
        """
        The main function that runs the program. This method should be overridden by subclasses to implement the
        runner logic.
        """
