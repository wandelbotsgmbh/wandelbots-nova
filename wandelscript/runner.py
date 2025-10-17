from pathlib import Path

from loguru import logger

import nova
from nova import Nova
from nova.cell.robot_cell import RobotCell
from nova.events import CycleDevice
from nova.program import ProgramRunner
from nova.program.runner import ExecutionContext as NovaExecutionContext
from wandelscript.datatypes import ElementType
from wandelscript.ffi import ForeignFunction
from wandelscript.ffi_loader import load_foreign_functions
from wandelscript.metamodel import Program as WandelscriptProgram
from wandelscript.runtime import ExecutionContext


class WandelscriptProgramRunner(ProgramRunner):
    """Provides functionalities to manage a single program execution"""

    def __init__(
        self,
        program_id: str,
        code: str,
        parameters: dict[str, ElementType] | None,
        robot_cell_override: RobotCell | None = None,
        default_robot: str | None = None,
        default_tcp: str | None = None,
        foreign_functions: dict[str, ForeignFunction] | None = None,
    ):
        async def wandelscript_wrapper():
            print(f"Running wandelscript program {program_id}...")

        program = nova.program(id=program_id)(wandelscript_wrapper)

        super().__init__(
            program,
            parameters=parameters,  # type: ignore
            robot_cell_override=robot_cell_override,
        )
        self._program = wandelscript_wrapper
        self._code: str = code
        self._default_robot: str | None = default_robot
        self._default_tcp: str | None = default_tcp
        self._foreign_functions: dict[str, ForeignFunction] = foreign_functions or {}
        self._ws_execution_context: ExecutionContext | None = None

    async def _run(self, execution_context: NovaExecutionContext):
        # Try parsing the program and handle parsing error
        logger.info(f"Parse program {self.program_id}...")
        logger.debug(self._code)

        self._ws_execution_context = ws_execution_context = ExecutionContext(
            robot_cell=execution_context.robot_cell,
            stop_event=execution_context.stop_event,
            default_robot=self._default_robot,
            default_tcp=self._default_tcp,
            run_args=self._parameters,
            foreign_functions=self._foreign_functions,
        )

        program = WandelscriptProgram.from_code(self._code)
        # Execute Wandelscript
        await program(ws_execution_context)
        self.execution_context.motion_group_recordings = (
            ws_execution_context.motion_group_recordings
        )
        self.execution_context.output_data = ws_execution_context.store.data_dict


def run(
    program_id: str,
    code: str,
    parameters: dict[str, ElementType] | None = None,
    default_robot: str | None = None,
    default_tcp: str | None = None,
    foreign_functions: dict[str, ForeignFunction] | None = None,
    robot_cell_override: RobotCell | None = None,
) -> WandelscriptProgramRunner:
    """Helper function to create a ProgramRunner and start it synchronously

    Args:
        program_id (str): The unique identifier of the program.
        code (str): Wandelscript code
        parameters (dict[str, Any], optional): Store will be initialized with this dict. Defaults to ().
        default_robot (str): The default robot that is used when no robot is active
        default_tcp (str): The default TCP that is used when no TCP is explicitly selected for a motion
        foreign_functions (dict[str, ForeignFunction], optional): 3rd party functions that you can
            register into the wandelscript language. Defaults to {}.
        robot_cell_override: The robot cell to use for the program. If None, the default robot cell is used.

    Returns:
        ProgramRunner: A new ProgramRunner object

    """
    runner = WandelscriptProgramRunner(
        program_id=program_id,
        code=code,
        parameters=parameters,
        default_robot=default_robot,
        default_tcp=default_tcp,
        foreign_functions=foreign_functions,
        robot_cell_override=robot_cell_override,
    )
    runner.start(sync=True)
    return runner


async def run_wandelscript_program(
    program_id: str,
    code: str,
    parameters: dict[str, ElementType] = {},
    foreign_functions_paths: list[Path] | None = None,
    default_robot: str | None = None,
    default_tcp: str | None = None,
    nova: Nova | None = None,
) -> WandelscriptProgramRunner:
    logger.info(f"Creating wandelscript program: {program_id}")
    foreign_functions = (
        load_foreign_functions(foreign_functions_paths) if foreign_functions_paths else {}
    )

    if nova is None:
        nova = Nova()

    async with nova:
        cell = nova.cell()
        controllers = await cell.controllers()
        robot_cell = RobotCell(
            timer=None, cycle=None, **{controller.id: controller for controller in controllers}
        )
        cycle_device = CycleDevice(cell=cell)
        robot_cell.devices["cycle"] = cycle_device

        runner = WandelscriptProgramRunner(
            program_id=program_id,
            code=code,
            parameters=parameters,
            default_robot=default_robot,
            default_tcp=default_tcp,
            foreign_functions=foreign_functions,
            robot_cell_override=robot_cell,
        )
        runner.start(sync=True)
        return runner
