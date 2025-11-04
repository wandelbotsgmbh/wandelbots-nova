import asyncio
import contextlib
import contextvars
import inspect
import io
import sys
import threading
import traceback as tb
import uuid
from abc import ABC, abstractmethod
from collections.abc import Awaitable, Callable
from concurrent.futures import Future
from datetime import datetime
from typing import Any, Coroutine, Optional

import anyio
from anyio import from_thread, to_thread
from anyio.abc import TaskStatus
from exceptiongroup import ExceptionGroup
from loguru import logger
from pydantic import BaseModel, Field, StrictStr
from wandelbots_api_client.v2.models import ProgramRun as ApiProgramRun
from wandelbots_api_client.v2.models import ProgramRunState

from nova import Nova, NovaConfig, api
from nova.cell.robot_cell import RobotCell
from nova.config import CELL_NAME
from nova.core.exceptions import PlanTrajectoryFailed
from nova.nats import Message
from nova.program.exceptions import NotPlannableError
from nova.program.function import Program
from nova.program.utils import Tee, stoppable_run
from nova.types import MotionState
from nova.utils import timestamp

current_execution_context_var: contextvars.ContextVar = contextvars.ContextVar(
    "current_execution_context_var"
)

# Context variable to track if running via operator/novax (for viewer optimization)
# Set to True when app_name is provided (operator execution), False for local development
is_operator_execution_var: contextvars.ContextVar[bool] = contextvars.ContextVar(
    "is_operator_execution_var", default=False
)


# needs to change somehow
class ProgramRun(ApiProgramRun):
    output_data: dict[str, Any] = {}


# Define maximum length for error messages in ProgramStatus
# This is a safeguard to prevent malicious users in a cloud environment from sending
# very large error messages that could lead to performance issues or denial of service.
PROGRAM_STATUS_ERROR_MAX_LENGTH = 1024 * 2


class ProgramStatus(BaseModel):
    run: StrictStr = Field(description="Unique identifier of the program run")
    program: StrictStr = Field(description="Unique identifier of the program")
    app: Optional[StrictStr] = Field(description="The app name where the program is hosted")
    state: ProgramRunState = Field(description="State of the program run")
    error: Optional[StrictStr] = Field(
        default=None,
        description="Error message if the program run failed",
        max_length=PROGRAM_STATUS_ERROR_MAX_LENGTH,
    )
    start_time: Optional[datetime] = Field(
        default=None, description="The RFC3339 timestamp of the start time of the program run"
    )
    end_time: Optional[datetime] = Field(
        default=None, description="The RFC3339 timestamp of the end time of the program run"
    )
    timestamp: datetime = Field(description="The RFC3339 timestamp when the message was created")


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


class ProgramRunner(ABC):
    """Abstract base class for program runners.

    This class defines the interface that all program runners must implement.
    It provides the core functionality for running and managing program execution.
    """

    def __init__(
        self,
        program: Program,
        *,
        parameters: dict[str, Any],
        robot_cell_override: RobotCell | None = None,
        cell_id: str | None = None,
        app_name: str | None = None,
        nova_config: NovaConfig | None = None,
    ):
        """
        Args:
            program_id (str): The unique identifier of the program.
            parameters (dict[str, Any]): The parameters that are passed to the program.
            robot_cell_override (RobotCell | None, optional): The robot cell to use for the program. Should only be used for testing purposes. When a robot cell is provided, no Nova instance is created. Defaults to None.
            cell_id (str | None, optional): The cell ID to use for the program. Defaults to None.
            app_name (str | None, optional): The app name to discover the program. Will be automatically set when executed via NOVAx or API. Does not need to be set by the user. Defaults to None.
            nova_config (NovaConfig | None, optional): The Nova config to use for the program. Defaults to None.
        """
        program_id = program.program_id

        self._run_id = str(uuid.uuid4())
        self._program_id = program_id
        self._preconditions = program.preconditions
        self._parameters = parameters
        self._robot_cell_override = robot_cell_override
        self._cell_id = cell_id or CELL_NAME
        self._app_name = app_name
        self._nova_config = nova_config
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
            input_data=parameters,
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
    def program_status(self) -> ProgramStatus:
        truncated_error = (
            error[:PROGRAM_STATUS_ERROR_MAX_LENGTH] if (error := self._program_run.error) else None
        )
        return ProgramStatus(
            run=self._program_run.run,
            program=self._program_run.program,
            app=self._app_name,
            state=self._program_run.state,
            error=truncated_error,
            start_time=self._program_run.start_time,
            end_time=self._program_run.end_time,
            timestamp=timestamp.now_utc(),
        )

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
        self,
        sync: bool = False,
        on_state_change: Callable[[ProgramRun], Awaitable[None]] | None = None,
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
        # TODO: introduce real state machine to manage the program run states
        if self.state is not ProgramRunState.PREPARING:
            raise RuntimeError(
                "The runner is not in the not_started state. Create a new runner to execute again."
            )

        async def _on_state_change():
            if on_state_change is not None:
                await on_state_change(self._program_run.model_copy())

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

        # Create new thread and run _run
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
            # TODO: only stream devices that are running, not everything in the robot cell.
            #   -> How to figure that out? Maybe only controllers in precondition?
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

    async def _set_program_state(
        self,
        state: ProgramRunState,
        on_state_change: Callable[[], Awaitable[None]],
        nova: Nova | None = None,
    ):
        logger.info(f"Set state of program {self.program_id} run {self.run_id} to {state}")
        self._program_run.state = state
        await on_state_change()

        if nova is not None:
            data = self.program_status.model_dump_json().encode("utf-8")

            # publish program run to NATS
            subject = f"nova.v2.cells.{self._cell_id}.programs"
            message = Message(subject=subject, data=data)
            await nova.nats.publish_message(message)

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
            nova: Nova | None = None
            robot_cell = None

            if self._robot_cell_override:
                robot_cell = self._robot_cell_override
            else:
                # When there is no robot_cell_override we create a new robot cell
                #   based on the program preconditions. That means also only devices that are
                #   part of the preconditions are opened and streamed for e.g. estop handling
                nova = Nova(config=self._nova_config)
                await nova.connect()
                cell = nova.cell()
                controllers = await cell.controllers()
                controller_specs = (
                    list(self._preconditions.controllers or []) if self._preconditions else []
                )
                controllers = []
                for controller_spec in controller_specs:
                    # Ensure the controller exists and get the actual controller object
                    # TODO: right now they are also ensured in the decorator. Maybe it makes sense to
                    #   only ensure them here
                    ctrl = await cell.ensure_controller(controller_spec)
                    controllers.append(ctrl)

                robot_cell = RobotCell(
                    timer=None,
                    cycle=None,
                    **{controller.id: controller for controller in controllers},
                )

            if robot_cell is None:
                raise RuntimeError("No robot cell available")

            # if not self._robot_cell_override and not nova.is_connected():
            #    await nova.connect()

            # Set context variable to indicate if running via operator (for viewer optimization)
            is_operator_execution_var.set(self._app_name is not None)

            self.execution_context = execution_context = ExecutionContext(
                robot_cell=robot_cell, stop_event=stop_event
            )
            current_execution_context_var.set(execution_context)
            await self._set_program_state(ProgramRunState.PREPARING, on_state_change, nova)

            monitoring_scope = anyio.CancelScope()
            async with robot_cell, anyio.create_task_group() as tg:
                await tg.start(self._estop_handler, monitoring_scope)

                try:
                    self._program_run.start_time = timestamp.now_utc()
                    await self._set_program_state(ProgramRunState.RUNNING, on_state_change, nova)
                    await self._run(execution_context)
                except anyio.get_cancelled_exc_class() as exc:  # noqa: F841
                    # Program was stopped
                    logger.info(f"Program {self.program_id} run {self.run_id} cancelled")
                    try:
                        with anyio.CancelScope(shield=True):
                            await robot_cell.stop()
                    except Exception as e:
                        logger.error(
                            f"Program {self.program_id} run {self.run_id}: Error while stopping robot cell: {e}"
                        )
                        raise

                    await self._set_program_state(ProgramRunState.STOPPED, on_state_change, nova)
                    raise
                except NotPlannableError as exc:
                    # Program was not plannable (aka. /plan/ endpoint)
                    self._handle_general_exception(exc)
                    await self._set_program_state(ProgramRunState.FAILED, on_state_change, nova)
                except Exception as exc:  # pylint: disable=broad-except
                    self._handle_general_exception(exc)
                    await self._set_program_state(ProgramRunState.FAILED, on_state_change, nova)
                else:
                    if self.stopped:
                        # Program was stopped
                        await self._set_program_state(
                            ProgramRunState.STOPPED, on_state_change, nova
                        )
                    elif self._program_run.state is ProgramRunState.RUNNING:
                        # Program was completed
                        await self._set_program_state(
                            ProgramRunState.COMPLETED, on_state_change, nova
                        )
                finally:
                    # get program output data
                    self._program_run.output_data = execution_context.output_data
                    self._program_run.end_time = timestamp.now_utc()

                    logger.info(
                        f"Program {self.program_id} run {self.run_id} finished. Run teardown routine..."
                    )

                    logger.remove(sink_id)
                    self._program_run.logs = log_capture.getvalue()
                    monitoring_scope.cancel()
        except anyio.get_cancelled_exc_class():
            raise
        except Exception as exc:  # pylint: disable=broad-except
            # Handle any exceptions raised during entering the robot cell context
            self._handle_general_exception(exc)
            await self._set_program_state(ProgramRunState.FAILED, on_state_change, nova)

    @abstractmethod
    async def _run(self, execution_context: ExecutionContext):
        """
        The main function that runs the program. This method should be overridden by subclasses to implement the
        runner logic.
        """


class PythonProgramRunner(ProgramRunner):
    """Runner for Python programs"""

    def __init__(
        self,
        program: Program,
        parameters: Optional[dict[str, Any]] = None,
        robot_cell_override: RobotCell | None = None,
        nova_config: NovaConfig | None = None,
        app_name: str | None = None,
    ):
        super().__init__(
            program,
            parameters=parameters or {},
            robot_cell_override=robot_cell_override,
            nova_config=nova_config,
            app_name=app_name,
        )
        # TODO: is this still required?
        self.program = program
        self.parameters = parameters

    async def _run(self, execution_context: ExecutionContext) -> Any:
        # Execute the function with parameters
        result = self.program(**(self.parameters or {}))

        # Check if the function is async and await it if necessary
        if inspect.iscoroutine(result):
            result = await result

        return result


def _log_future_result(future: Future):
    try:
        result = future.result()
        logger.debug(f"Program state change callback completed with result: {result}")
    except Exception as e:
        logger.error(f"Program state change callback failed with exception: {e}")


def _report_state_change_to_event_loop(
    loop: asyncio.AbstractEventLoop | None,
    state_listener: Callable[[ProgramRun], Coroutine[Any, Any, None]] | None,
) -> Callable[[ProgramRun], Awaitable[None]] | None:
    """Return an awaitable listener that schedules the user's async callback.

    If `loop` is a running, open asyncio loop, callbacks are posted to it
    (thread-safe). Otherwise, callbacks are scheduled on the *current* loop,
    i.e., the runner thread's event loop created by `anyio.run`.

    """
    if not state_listener:
        return None

    async def _state_listener(program_run: ProgramRun):
        logger.debug(f"Reporting state change to event loop for program run: {program_run.program}")
        coroutine = state_listener(program_run)

        try:
            if loop and loop.is_running() and not loop.is_closed():
                # Post the callback onto the caller's loop, thread-safe.
                future: Future = asyncio.run_coroutine_threadsafe(coroutine, loop)
                future.add_done_callback(_log_future_result)
            else:
                # No caller loop: schedule on the runner thread's loop.
                asyncio.create_task(coroutine)
        except Exception as e:
            logger.error(
                f"Unexpected error in state change callback for program {program_run.program}: {e}"
            )

    return _state_listener


def run_program(
    program: Program,
    *,
    parameters: dict[str, Any] | None = None,
    sync: bool = True,
    robot_cell_override: RobotCell | None = None,
    on_state_change: Callable[[ProgramRun], Coroutine[Any, Any, None]] | None = None,
    nova_config: NovaConfig | None = None,
    app_name: str | None = None,
) -> PythonProgramRunner:
    """Run a program with given parameters.

    Args:
        program: The program to run
        parameters: The parameters to pass to the program
        sync: If True, the program runs synchronously
        robot_cell_override: The robot cell to use for the program
        on_state_change: The callback to call when the state of the program changes

    Returns:
        PythonProgramRunner: The runner for the program

    """
    runner = PythonProgramRunner(
        program,
        parameters=parameters,
        robot_cell_override=robot_cell_override,
        nova_config=nova_config,
        app_name=app_name,
    )

    # Try to grab a caller loop if there is one; otherwise, fall back to None.
    try:
        loop: asyncio.AbstractEventLoop | None = asyncio.get_running_loop()
    except RuntimeError:
        loop = None

    on_state_change_listener = (
        _report_state_change_to_event_loop(loop, on_state_change) if on_state_change else None
    )

    runner.start(sync=sync, on_state_change=on_state_change_listener)

    return runner
