from pathlib import Path

from loguru import logger

import nova
from nova import Nova
from nova.cell.robot_cell import RobotCell
from nova.program import Program
from nova.program import ProgramRunner as NovaProgramRunner
from nova.program.runner import ExecutionContext as NovaExecutionContext
from wandelscript.datatypes import ElementType
from wandelscript.ffi import ForeignFunction
from wandelscript.ffi_loader import load_foreign_functions
from wandelscript.metamodel import Program as WandelscriptProgram
from wandelscript.runtime import ExecutionContext


class ProgramRunner(NovaProgramRunner):
    """Provides functionalities to manage a single program execution"""

    def __init__(
        self,
        program_id: str,
        program: str,
        args: dict[str, ElementType] | None,
        robot_cell_override: RobotCell | None = None,
        default_robot: str | None = None,
        default_tcp: str | None = None,
        foreign_functions: dict[str, ForeignFunction] | None = None,
    ):
        super().__init__(
            program_id=program_id,
            args=args,  # type: ignore
            robot_cell_override=robot_cell_override,
        )
        self._program: str = program
        self._default_robot: str | None = default_robot
        self._default_tcp: str | None = default_tcp
        self._foreign_functions: dict[str, ForeignFunction] = foreign_functions or {}
        self._ws_execution_context: ExecutionContext | None = None

    async def _run(self, execution_context: NovaExecutionContext):
        # Try parsing the program and handle parsing error
        logger.info(f"Parse program {self.program_id}...")
        logger.debug(self._program)

        self._ws_execution_context = ws_execution_context = ExecutionContext(
            robot_cell=execution_context.robot_cell,
            stop_event=execution_context.stop_event,
            default_robot=self._default_robot,
            default_tcp=self._default_tcp,
            run_args=self._args,
            foreign_functions=self._foreign_functions,
        )

        program = WandelscriptProgram.from_code(self._program)
        # Execute Wandelscript
        await program(ws_execution_context)
        self.execution_context.motion_group_recordings = (
            ws_execution_context.motion_group_recordings
        )
        self.execution_context.output_data = ws_execution_context.store.data_dict


def run(
    program_id: str,
    program: str,
    args: dict[str, ElementType] | None = None,
    default_robot: str | None = None,
    default_tcp: str | None = None,
    foreign_functions: dict[str, ForeignFunction] | None = None,
    robot_cell_override: RobotCell | None = None,
) -> ProgramRunner:
    """Helper function to create a ProgramRunner and start it synchronously

    Args:
        program (str): Wandelscript code
        args (dict[str, Any], optional): Store will be initialized with this dict. Defaults to ().
        default_robot (str): The default robot that is used when no robot is active
        default_tcp (str): The default TCP that is used when no TCP is explicitly selected for a motion
        foreign_functions (dict[str, ForeignFunction], optional): 3rd party functions that you can
            register into the wandelscript language. Defaults to {}.
        robot_cell_override: The robot cell to use for the program. If None, the default robot cell is used.

    Returns:
        ProgramRunner: A new ProgramRunner object

    """
    runner = ProgramRunner(
        program_id=program_id,
        program=program,
        args=args,
        default_robot=default_robot,
        default_tcp=default_tcp,
        foreign_functions=foreign_functions,
        robot_cell_override=robot_cell_override,
    )
    runner.start(sync=True)
    return runner


def create_wandelscript_program(
    program_id: str,
    code: str,
    args: dict[str, ElementType] = {},
    foreign_functions_paths: list[Path] | None = None,
    default_robot: str | None = None,
    default_tcp: str | None = None,
) -> Program:
    logger.info(f"Creating wandelscript program: {program_id}")
    foreign_functions = (
        load_foreign_functions(foreign_functions_paths) if foreign_functions_paths else {}
    )

    @nova.program(id=program_id)
    async def wandelscript_wrapper():
        async with Nova() as nova:
            robot_cell = await nova.cell().get_robot_cell()

            # TODO: Don't create another runner here, just execute the program
            result = run(
                program_id=program_id,
                program=code,
                args=args,
                foreign_functions=foreign_functions,
                robot_cell_override=robot_cell,
                default_robot=default_robot,
                default_tcp=default_tcp,
            )
            return result

    return wandelscript_wrapper
