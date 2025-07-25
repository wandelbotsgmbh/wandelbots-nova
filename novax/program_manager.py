import datetime as dt
import inspect
from typing import Any, Optional

from pydantic import BaseModel

from nova.cell.robot_cell import RobotCell
from nova.program.function import Program, ProgramPreconditions
from nova.program.runner import ExecutionContext, ProgramRun, ProgramRunner


class NovaxProgramRunner(ProgramRunner):
    def __init__(
        self,
        program_id: str,
        program_functions: dict[str, Program],
        parameters: Optional[dict[str, Any]] = None,
        robot_cell_override: RobotCell | None = None,
    ):
        super().__init__(program_id=program_id, args={}, robot_cell_override=robot_cell_override)
        self.program_functions = program_functions
        self.parameters = parameters

    async def _run(self, execution_context: ExecutionContext) -> Any:
        if self.program_id not in self.program_functions:
            raise ValueError(f"Program {self.program_id} not found")

        func = self.program_functions[self.program_id]

        # Execute the function with parameters
        if self.parameters:
            result = func(**self.parameters)
        else:
            result = func()

        # Check if the function is async and await it if necessary
        if inspect.iscoroutine(result):
            result = await result

        return result


class ProgramDetails(BaseModel):
    program: str
    name: str | None
    description: str | None
    created_at: dt.datetime
    preconditions: ProgramPreconditions | None = None


class RunProgramRequest(BaseModel):
    parameters: Optional[dict[str, Any]] = None


class ProgramManager:
    """Manages program registration, storage, and execution"""

    def __init__(self, robot_cell_override: RobotCell | None = None):
        self._programs: dict[str, ProgramDetails] = {}
        self._program_functions: dict[str, Program] = {}
        self._runner: NovaxProgramRunner | None = None
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
        now = dt.datetime.now(dt.timezone.utc)

        # Create ProgramDetails instance
        program_details = ProgramDetails(
            program=program_id,
            name=program.name,
            description=program.description,
            created_at=now,
            preconditions=program.preconditions,
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
        self, program_id: str, parameters: dict[str, Any] | None = None, sync: bool = False
    ) -> ProgramRun:
        """Start a registered program with given parameters"""
        if self.is_any_program_running:
            raise RuntimeError("A program is already running")

        runner = NovaxProgramRunner(
            program_id,
            self._program_functions,
            parameters,
            robot_cell_override=self._robot_cell_override,
        )
        self._runner = runner
        runner.start(sync=sync)
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
