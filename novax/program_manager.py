import datetime
import inspect
from pathlib import Path
from typing import Any, AsyncIterator, Optional, Protocol

from loguru import logger
from pydantic import BaseModel

import nova
from nova import Nova
from nova.program.function import Function
from nova.program.runner import ExecutionContext, Program, ProgramRun, ProgramRunner, ProgramType

try:
    import wandelscript

    WANDELSCRIPT_AVAILABLE = True
except ImportError:
    WANDELSCRIPT_AVAILABLE = False


class ProgramSource(Protocol):
    """Protocol for program sources that can be registered with ProgramManager"""

    def get_programs(self, program_manager: "ProgramManager") -> AsyncIterator[Function | Path]:
        """
        Discover and yield all programs from this source.

        Yields:
            Function | Path: Either a Function object or a Path to a wandelscript file
        """
        ...


class NovaxProgramRunner(ProgramRunner):
    def __init__(
        self,
        program_id: str,
        program_functions: dict[str, Function],
        parameters: Optional[dict[str, Any]] = None,
    ):
        super().__init__(
            program_id=program_id,
            program=Program(content="", program_type=ProgramType.PYTHON),
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
    created_at: str
    updated_at: str


class RunProgramRequest(BaseModel):
    parameters: Optional[dict[str, Any]] = None


class ProgramManager:
    """Manages program registration, storage, and execution"""

    def __init__(self):
        self._programs: dict[str, ProgramDetails] = {}
        self._program_functions: dict[str, Function] = {}
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

    def register_program(self, func_or_path: Function | Path) -> str:
        """
        Register a function or wandelscript file as a program.

        Args:
            func_or_path: Either a Function object (decorated with @nova.program)
                         or a Path to a wandelscript (.ws) file

        Returns:
            str: The program ID
        """

        if isinstance(func_or_path, Function):
            # Handle Function object (existing behavior)
            func = func_or_path
            program_id = func.name
        elif isinstance(func_or_path, Path):
            # Handle wandelscript file path
            path = func_or_path
            if not path.exists():
                raise FileNotFoundError(f"Wandelscript file not found: {path}")

            match path.suffix:
                case ".ws":
                    # Check if wandelscript package is available
                    if not WANDELSCRIPT_AVAILABLE:
                        raise ImportError(
                            f"Cannot register wandelscript file {path}: wandelscript package is not installed. "
                            "Please install it with 'pip install wandelscript' or 'uv add wandelscript'"
                        )
                    # Create program ID from filename
                    program_id = path.stem
                    logger.info(f"Registering wandelscript program: {program_id}")

                    # Create a wrapper function similar to run_ws
                    @nova.program(name=program_id)
                    async def wandelscript_wrapper():
                        # TODO: how to pass parameters here?
                        async with Nova() as nova:
                            robot_cell = await nova.cell().get_robot_cell()
                            result = await wandelscript.run_file(
                                path,
                                # args=kwargs,
                                robot_cell_override=robot_cell,
                            )
                            return result

                    func = wandelscript_wrapper
                case _:
                    raise NotImplementedError(f"File must have .ws extension: {path}")
        else:
            raise TypeError(f"Expected Function or Path, got {type(func_or_path)}")

        now = datetime.datetime.now(datetime.timezone.utc).isoformat() + "Z"

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


class FileSystemProgramSource:
    """Example program source that loads programs from a filesystem directory"""

    def __init__(self, directory_path: Path):
        self.directory_path = directory_path

    async def get_programs(self) -> AsyncIterator[Function | Path]:
        """Discover and yield programs from filesystem"""
        if not self.directory_path.exists():
            return

        for file_path in self.directory_path.glob("*.ws"):
            yield file_path
