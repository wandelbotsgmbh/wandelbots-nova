from typing import Any, Callable, Coroutine, Optional

from wandelbots_api_client.v2.models.program import Program as ProgramDetails

from nova.cell.robot_cell import RobotCell
from nova.config import NovaConfig
from nova.program import Program, PythonProgramRunner, run_program
from nova.program.runner import ProgramRun
from novax.config import APP_NAME, CELL_NAME


class ProgramManager:
    """Manages program registration, storage, and execution"""

    def __init__(
        self,
        *,
        nova_config: NovaConfig | None = None,
        cell_id: str | None = None,
        app_name: str | None = None,
        robot_cell_override: RobotCell | None = None,
    ):
        """
        Initialize the ProgramManager.
        Args:
            robot_cell_override: Optional override for the robot cell the program runs against
            state_listener: Optional listener for program state changes
        """

        self._cell_id = cell_id or CELL_NAME
        self._app_name = app_name or APP_NAME
        self._programs: dict[str, ProgramDetails] = {}
        self._program_functions: dict[str, Program] = {}
        self._runner: PythonProgramRunner | None = None
        self._nova_config: NovaConfig | None = nova_config
        self._robot_cell_override: RobotCell | None = robot_cell_override

    def has_program(self, program_id: str) -> bool:
        return program_id in self._programs

    @property
    def is_any_program_running(self) -> bool:
        return self._runner is not None and self._runner.is_running()

    @property
    def running_program(self) -> Optional[str]:
        return self._runner.program_id if self.is_any_program_running and self._runner else None

    def register_program(self, program: Program) -> str:
        """
        Register a function as a program.

        Args:
            program: A Program object (decorated with @nova.program)

        Returns:
            str: The program ID
        """

        func = program
        program_id = func.program_id

        # Create ProgramDetails instance
        program_details = ProgramDetails(
            app=self._app_name,
            program=program_id,
            name=program.name,
            description=program.description,
            preconditions=program.preconditions.model_dump(mode="json")
            if program.preconditions
            else None,
            input_schema=program.input_schema,
        )

        # Store program details and function separately
        self._programs[program_id] = program_details
        self._program_functions[program_id] = func

        return program_id

    def deregister_program(self, program_id: str):
        """
        Deregister a program from the program manager

        Args:
            program_id: The ID of the program to deregister
        """
        if program_id not in self._programs:
            return
        del self._programs[program_id]
        del self._program_functions[program_id]

    async def get_programs(self) -> dict[str, ProgramDetails]:
        """Get all registered programs"""
        return self._programs.copy()

    async def get_program(self, program_id: str) -> Optional[ProgramDetails]:
        """Get a specific program by ID"""
        return self._programs.get(program_id)

    async def start_program(
        self,
        program_id: str,
        parameters: dict[str, Any] | None = None,
        sync: bool = False,
        on_state_change: Callable[[ProgramRun], Coroutine[Any, Any, None]] | None = None,
    ) -> ProgramRun:
        """
        Start a registered program with given parameters.

        Args:
            program_id: The ID of the program to start
            parameters: Optional parameters to pass to the program function
            sync: If True, run the program synchronously
            on_state_change: Optional callback to handle program state changes
        """
        program = self._program_functions[program_id]
        if program is None:
            raise KeyError(f"Program {program_id} not found")

        if self.is_any_program_running:
            raise RuntimeError("A program is already running")

        runner = run_program(
            program,
            parameters=parameters,
            sync=sync,
            on_state_change=on_state_change,
            nova_config=self._nova_config,
            robot_cell_override=self._robot_cell_override,
            app_name=self._app_name,
        )

        self._runner = runner
        return runner.program_run

    async def stop_program(self, program_id: str):
        """Stop a running program"""
        if not self.is_any_program_running or self._runner is None:
            raise RuntimeError("No program is running")

        if self.running_program != program_id:
            raise RuntimeError(
                f"Program {program_id} is not running. Currently running: {self.running_program}"
            )

        self._runner.stop(sync=True)
        self._runner = None
