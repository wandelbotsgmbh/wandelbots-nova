import datetime as dt
import inspect
from pathlib import Path
from typing import Any, AsyncIterator, Optional, Protocol

from loguru import logger
from pydantic import BaseModel

import nova
from nova import Nova
from nova.program.function import Program
from nova.program.runner import ExecutionContext, ProgramRun, ProgramRunner, ProgramType
from nova.program.runner import Program as SimpleProgram
from wandelscript.ffi_loader import load_foreign_functions

try:
    import wandelscript

    WANDELSCRIPT_AVAILABLE = True
except ImportError:
    WANDELSCRIPT_AVAILABLE = False


class ProgramSource(Protocol):
    """Protocol for program sources that can be registered with ProgramManager"""

    def get_programs(self, program_manager: "ProgramManager") -> AsyncIterator[Program]:
        """
        Discover and yield all programs from this source.

        Yields:
            Program: A Program object (decorated with @nova.program)
        """
        ...


class NovaxProgramRunner(ProgramRunner):
    def __init__(
        self,
        program_id: str,
        program_functions: dict[str, Program],
        parameters: Optional[dict[str, Any]] = None,
    ):
        super().__init__(
            program_id=program_id,
            program=SimpleProgram(content="", program_type=ProgramType.PYTHON),
            args={},
        )
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
    program_id: str
    created_at: dt.datetime
    updated_at: dt.datetime


class RunProgramRequest(BaseModel):
    parameters: Optional[dict[str, Any]] = None


class ProgramManager:
    """Manages program registration, storage, and execution"""

    def __init__(self):
        self._programs: dict[str, ProgramDetails] = {}
        self._program_functions: dict[str, Program] = {}
        self._runners: dict[str, dict[str, NovaxProgramRunner]] = {}
        self._program_sources: list[ProgramSource] = []

    def has_program(self, program_id: str) -> bool:
        return program_id in self._programs

    def register_program_source(self, program_source: ProgramSource) -> None:
        """
        Register a program source with the program manager.

        Args:
            program_source: The program source to register
        """
        self._program_sources.append(program_source)

    def deregister_program_source(self, program_source: ProgramSource) -> None:
        """
        Deregister a program source from the program manager.

        Args:
            program_source: The program source to deregister
        """
        if program_source in self._program_sources:
            self._program_sources.remove(program_source)

    def register_program(self, program: Program) -> str:
        """
        Register a function as a program.

        Args:
            program: A Program object (decorated with @nova.program)

        Returns:
            str: The program ID
        """

        func = program
        program_id = func.name

        now = dt.datetime.now(dt.timezone.utc)

        # Create ProgramDetails instance
        program_details = ProgramDetails(program_id=program_id, created_at=now, updated_at=now)

        # Store program details and function separately
        self._programs[program_id] = program_details
        self._program_functions[program_id] = func

        return program_id

    async def get_programs(self) -> dict[str, ProgramDetails]:
        """Get all registered programs"""
        for program_source in self._program_sources:
            async for program in program_source.get_programs(self):
                self.register_program(program)
        return self._programs.copy()

    async def get_program(self, program_id: str) -> Optional[ProgramDetails]:
        """Get a specific program by ID"""
        return self._programs.get(program_id)

    async def get_program_runs(self, program_id: str) -> list[ProgramRun]:
        """Get all runs for a specific program"""
        return [runner.program_run for runner in self._runners.get(program_id, {}).values()]

    async def get_program_run(self, program_id: str, run_id: str) -> ProgramRun:
        """Get a specific run for a program"""
        return self._runners[program_id][run_id].program_run

    async def run_program(
        self, program_id: str, parameters: Optional[dict[str, Any]] = None
    ) -> ProgramRun:
        """Run a registered program with given parameters"""
        runner = NovaxProgramRunner(program_id, self._program_functions, parameters)
        if program_id not in self._runners:
            self._runners[program_id] = {}
        self._runners[program_id][runner.run_id] = runner
        runner.start(sync=False)
        return runner.program_run

    async def stop_program(self, program_id: str, run_id: str):
        """Stop a running program"""
        runner = self._runners[program_id][run_id]
        if not runner:
            raise ValueError(f"Runner {run_id} not found")
        runner.stop(sync=True)
        del self._runners[program_id][run_id]


# Example implementations of ProgramSource


class WandelscriptProgramSource:
    def __init__(self, scan_paths: list[Path], foreign_functions_paths: list[Path] | None = None):
        """
        Initialize the WandelscriptProgramSource with a list of paths to scan.

        Args:
            scan_paths: List of paths to scan for .ws files. Can be individual files or directories.
            foreign_functions: Optional dictionary of foreign functions to attach to all programs.
        """
        self.scan_paths = scan_paths
        self.foreign_functions = (
            load_foreign_functions(foreign_functions_paths) if foreign_functions_paths else {}
        )

    async def get_programs(self, program_manager: "ProgramManager") -> AsyncIterator[Program]:
        """Discover and yield programs from filesystem"""
        for path in self.scan_paths:
            if not path.exists():
                logger.warning(f"Path does not exist: {path}")
                continue

            if path.is_file():
                # Single file
                if path.suffix == ".ws":
                    yield self._create_wandelscript_program(path)
            elif path.is_dir():
                # Directory - scan for .ws files
                for file_path in path.glob("*.ws"):
                    yield self._create_wandelscript_program(file_path)

    def _create_wandelscript_program(self, path: Path) -> Program:
        """Create a nova.program wrapper for a wandelscript file"""
        if not WANDELSCRIPT_AVAILABLE:
            raise ImportError(
                f"Cannot register wandelscript file {path}: wandelscript package is not installed. "
                "Please install it with 'pip install wandelscript' or 'uv add wandelscript'"
            )

        # Create program ID from filename
        program_id = path.stem
        logger.info(f"Creating wandelscript program: {program_id}")

        # Create a wrapper function
        @nova.program(name=program_id)
        async def wandelscript_wrapper():
            async with Nova() as nova:
                robot_cell = await nova.cell().get_robot_cell()
                # Read the file content
                with open(path) as f:
                    program_content = f.read()

                result = wandelscript.run(
                    program_id=program_id,
                    program=program_content,
                    # TODO: Also pass args
                    args={},
                    foreign_functions=self.foreign_functions,
                    robot_cell_override=robot_cell,
                )
                return result

        return wandelscript_wrapper
