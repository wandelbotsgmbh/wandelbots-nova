import contextlib
import contextvars
import datetime as dt
import io
import sys
import threading
import traceback as tb
import uuid
from abc import ABC, abstractmethod
from collections.abc import Awaitable, Callable
from enum import Enum
from typing import Any

import anyio
from anyio import from_thread, to_thread
from anyio.abc import TaskStatus
from exceptiongroup import ExceptionGroup
from loguru import logger
from pydantic import BaseModel, Field

from nova import Nova, api
from nova.cell.robot_cell import RobotCell
from nova.core.exceptions import PlanTrajectoryFailed
from nova.program.exceptions import NotPlannableError
from nova.program.utils import Tee, stoppable_run
from nova.types import MotionState

current_execution_context_var: contextvars.ContextVar = contextvars.ContextVar(
    "current_execution_context_var"
)


# TODO: should provide a number of tools to the program to control the execution of the program
class ExecutionContext:
    # Maps the motion group id to the list of recorded motion lists
    # Each motion list is a path the was planned separately
    motion_group_recordings: list[list[MotionState]]
    output_data: dict[str, Any]

    def __init__(self, robot_cell: RobotCell, stop_event: anyio.Event):
        self._robot_cell = robot_cell
        self._stop_event = stop_event
        self.motion_group_recordings = []
        self.output_data = {}

    @property
    def robot_cell(self) -> RobotCell:
        return self._robot_cell

    @property
    def stop_event(self) -> anyio.Event:
        return self._stop_event


# TODO: import from api.v2.models.ProgramRunState
class ProgramRunState(Enum):
    PREPARING = "PREPARING"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    STOPPED = "STOPPED"


# TODO: import from api.v2.models.ProgramRun
class ProgramRun(BaseModel):
    run: str = Field(..., description="Id of the program run")
    program: str = Field(..., description="Id of the program")
    state: ProgramRunState = Field(..., description="State of the program run")
    logs: str | None = Field(None, description="Logs of the program run")
    stdout: str | None = Field(None, description="Stdout of the program run")
    error: str | None = Field(None, description="Error message of the program run, if any")
    traceback: str | None = Field(None, description="Traceback of the program run, if any")
    start_time: dt.datetime | None = Field(None, description="Start time of the program run")
    end_time: dt.datetime | None = Field(None, description="End time of the program run")
    input_data: dict[str, Any] | None = Field(None, description="Input of the program run")
    output_data: dict[str, Any] | None = Field(None, description="Output of the program run")


class ProgramRunner(ABC):
    """Abstract base class for program runners.

    This class defines the interface that all program runners must implement.
    It provides the core functionality for running and managing program execution.
    """

    def __init__(
        self, program_id: str, args: dict[str, Any], robot_cell_override: RobotCell | None = None
    ):
        """
        Args:
            program_id (str): The unique identifier of the program.
            args (dict[str, Any]): The arguments to pass to the program.
            robot_cell_override (RobotCell | None, optional): The robot cell to use for the program. Defaults to None.
        """
        self._run_id = str(uuid.uuid4())
        self._program_id = program_id
        self._args = args
        self._robot_cell_override = robot_cell_override
        self._program_run: ProgramRun = ProgramRun(
            run=self._run_id,
            program=program_id,
            state=ProgramRunState.PREPARING,
            logs=None,
            stdout=None,
            error=None,
            traceback=None,
            start_time=None,
            end_time=None,
            input_data=args,
            output_data={},
        )
        self._thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        self._exc: Exception | None = None

    @property
    def run_id(self) -> str:
        """Get the unique identifier of the program run.

        Returns:
            str: The unique identifier
        """
        return self._run_id

    @property
    def program_id(self) -> str:
        """Get the unique identifier of the program.

        Returns:
            str: The unique identifier
        """
        return self._program_id

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
        if self._stop_event is None:
            return False
        return self._stop_event.is_set()

    def is_running(self) -> bool:
        """Check if a program is currently running.

        Returns:
            bool: True if a program is running, False otherwise
        """
        return self._thread is not None and self.state in (
            ProgramRunState.PREPARING,
            ProgramRunState.RUNNING,
        )

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
        if self._stop_event is not None:
            self._stop_event.set()
        if sync:
            self.join()

    def start(
        self, sync: bool = False, on_state_change: Callable[[Any], Awaitable[None]] | None = None
    ):
        """Creates another thread and starts the program execution. If the program was executed already, is currently
        running, failed or was stopped a new program runner needs to be created.

        Args:
            sync: if True the execution is synchronous and the method blocks until the execution is finished
            on_state_change: callback function that is called when the state of the program runner changes

        Raises:
            RuntimeError: when the runner is not in IDLE state
        """
        # Check if another program execution is already in progress
        if self.state is not ProgramRunState.PREPARING:
            raise RuntimeError(
                "The runner is not in the not_started state. Create a new runner to execute again."
            )

        async def _on_state_change():
            if on_state_change is not None:
                await on_state_change(self._program_run)

        def stopper(sync_stop_event, async_stop_event):
            while not sync_stop_event.wait(0.2):
                from_thread.check_cancelled()
            from_thread.run_sync(async_stop_event.set)

        async def runner():
            self._stop_event = threading.Event()
            async_stop_event = anyio.Event()

            # TODO potential memory leak if the the program is running for a long time
            with contextlib.redirect_stdout(Tee(sys.stdout)) as stdout:
                try:
                    await stoppable_run(
                        self._run_program(
                            stop_event=async_stop_event, on_state_change=_on_state_change
                        ),
                        to_thread.run_sync(
                            stopper, self._stop_event, async_stop_event, abandon_on_cancel=True
                        ),
                    )
                except ExceptionGroup as eg:  # noqa: F821
                    raise eg.exceptions[0]
                self._program_run.stdout = stdout.getvalue()

        # Create new thread and runs _run
        # start a new thread
        self._thread = threading.Thread(target=anyio.run, name="ProgramRunner", args=[runner])
        self._thread.start()

        if sync:
            self.join()

    async def _estop_handler(
        self,
        monitoring_scope: anyio.CancelScope,
        *,
        task_status: TaskStatus[None] = anyio.TASK_STATUS_IGNORED,
    ):
        assert self.execution_context is not None

        def state_is_estop(state_: api.models.RobotControllerState):
            # See: models.RobotControllerState.safety_state
            acceptable_safety_states = ["SAFETY_STATE_NORMAL", "SAFETY_STATE_REDUCED"]
            return (
                isinstance(state_, api.models.RobotControllerState)
                and state_.safety_state not in acceptable_safety_states
            )

        with monitoring_scope:
            cell_state_stream = self.execution_context.robot_cell.stream_state(1000)
            task_status.started()

            async for state in cell_state_stream:
                if state_is_estop(state):
                    logger.info(f"ESTOP detected: {state}")
                    self.stop()  # TODO is this clean

    def _handle_general_exception(self, exc: Exception):
        # Handle any exceptions raised during task execution
        traceback = tb.format_exc()
        logger.error(f"Program {self.program_id} run {self.run_id} failed")

        if isinstance(exc, PlanTrajectoryFailed):
            message = f"{type(exc)}: {exc.to_pretty_string()}"
        else:
            message = f"{type(exc)}: {str(exc)}"
            logger.error(traceback)
            self._exc = exc
        logger.error(message)
        self._program_run.error = message
        self._program_run.traceback = traceback
        self._program_run.state = ProgramRunState.FAILED

    async def _run_program(
        self, stop_event: anyio.Event, on_state_change: Callable[[], Awaitable[None]]
    ) -> None:
        """Runs the program and handles the execution context, program state and exception handling.

        Args:
            stop_event: event that is set when the program execution should be stopped
            on_state_change: callback function that is called when the state of the program runner changes

        Raises:
            CancelledError: when the program execution is cancelled  # noqa: DAR402

        # noqa: DAR401
        """

        # Create a new logger sink to capture the output of the program execution
        # TODO potential memory leak if the the program is running for a long time
        log_capture = io.StringIO()
        sink_id = logger.add(log_capture)

        try:
            robot_cell = None
            # TODO: this should be removed to make it possible running programs without a robot cell
            if self._robot_cell_override:
                robot_cell = self._robot_cell_override
            else:
                async with Nova() as nova:
                    cell = nova.cell()
                    robot_cell = await cell.get_robot_cell()

            if robot_cell is None:
                raise RuntimeError("No robot cell available")

            self.execution_context = execution_context = ExecutionContext(
                robot_cell=robot_cell, stop_event=stop_event
            )
            current_execution_context_var.set(execution_context)
            await on_state_change()

            monitoring_scope = anyio.CancelScope()
            async with robot_cell, anyio.create_task_group() as tg:
                await tg.start(self._estop_handler, monitoring_scope)

                try:
                    logger.info(f"Program {self.program_id} run {self.run_id} started")
                    self._program_run.state = ProgramRunState.RUNNING
                    self._program_run.start_time = dt.datetime.now(dt.timezone.utc)
                    await self._run(execution_context)
                except anyio.get_cancelled_exc_class() as exc:  # noqa: F841
                    # Program was stopped
                    logger.info(f"Program {self.program_id} run {self.run_id} cancelled")
                    try:
                        with anyio.CancelScope(shield=True):
                            await robot_cell.stop()
                    except Exception as e:
                        logger.error(
                            f"Program {self.program_id} run {self.run_id}: Error while stopping robot cell: {e!r}"
                        )
                        raise

                    self._program_run.state = ProgramRunState.STOPPED
                    raise

                except NotPlannableError as exc:
                    # Program was not plannable (aka. /plan/ endpoint)
                    self._handle_general_exception(exc)
                except Exception as exc:  # pylint: disable=broad-except
                    self._handle_general_exception(exc)
                else:
                    if self.stopped:
                        # Program was stopped
                        logger.info(
                            f"Program {self.program_id} run {self.run_id} stopped successfully"
                        )
                        self._program_run.state = ProgramRunState.STOPPED
                    elif self._program_run.state is ProgramRunState.RUNNING:
                        # Program was completed
                        self._program_run.state = ProgramRunState.COMPLETED
                        logger.info(
                            f"Program {self.program_id} run {self.run_id} completed successfully"
                        )
                finally:
                    # program output data
                    self._program_run.output_data = execution_context.output_data

                    logger.info(
                        f"Program {self.program_id} run {self.run_id} finished. Run teardown routine..."
                    )
                    self._program_run.end_time = dt.datetime.now(dt.timezone.utc)

                    logger.remove(sink_id)
                    self._program_run.logs = log_capture.getvalue()
                    monitoring_scope.cancel()
                    await on_state_change()
        except anyio.get_cancelled_exc_class():
            raise
        except Exception as exc:  # pylint: disable=broad-except
            # Handle any exceptions raised during entering the robot cell context
            self._handle_general_exception(exc)

    @abstractmethod
    async def _run(self, execution_context: ExecutionContext):
        """
        The main function that runs the program. This method should be overridden by subclasses to implement the
        runner logic.
        """
