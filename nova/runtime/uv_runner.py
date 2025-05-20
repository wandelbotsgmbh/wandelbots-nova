import asyncio
import datetime
import json
import os
import sys
import tempfile
import traceback
from pathlib import Path

from nova.cell.robot_cell import RobotCell
from nova.core.logging import logger
from nova.runtime.runner import ExecutionContext, Program, ProgramRunner, ProgramType


class UVProgramRunner(ProgramRunner):
    """
    The UV Program runner runs Python programs using the uv library. It enables to run standalone Python programs
    where the dependencies, input parameters and metadata of the program is defined within one file.
    """

    def __init__(self, program: Program, args: dict, robot_cell_override: RobotCell | None = None):
        if not program.program_type == ProgramType.PYTHON:
            raise ValueError(f"Program type must be {ProgramType.PYTHON}")

        super().__init__(program=program, args=args, robot_cell_override=robot_cell_override)

        self.project_dir = Path(tempfile.mkdtemp())
        self.program_file = self.project_dir / "program.py"

    async def _install_uv(self):
        """Install uv if not already installed."""
        try:
            process = await asyncio.create_subprocess_exec(
                "uv", "--version", stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE
            )
            await process.communicate()

            if process.returncode == 0:
                logger.info("uv is already installed")
                return

        except FileNotFoundError:
            logger.info("Installing uv...")
            # TODO: handle stop event within process
            process = await asyncio.create_subprocess_exec(
                sys.executable,
                "-m",
                "pip",
                "install",
                "uv",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            stdout, stderr = await process.communicate()

            if process.returncode != 0:
                raise RuntimeError(f"Failed to install uv: {stderr.decode()}")

            logger.info("uv installed successfully")

    async def _validate_program(self):
        """Validate that the program has a main function with correct parameters."""
        import ast

        tree = ast.parse(self._program.content)
        main_func = None
        has_correct_decorator = False

        # Find the main function and check its decorators
        for node in ast.walk(tree):
            if isinstance(node, ast.AsyncFunctionDef) and node.name == "main":
                main_func = node
                # Check decorators
                for decorator in node.decorator_list:
                    if isinstance(decorator, ast.Name) and decorator.id == "program":
                        has_correct_decorator = True
                        break
                    elif isinstance(decorator, ast.Attribute) and decorator.attr == "program":
                        has_correct_decorator = True
                        break

        if not main_func:
            raise ValueError("Program must have an async main function")

        if not has_correct_decorator:
            raise ValueError(
                "Main function must be decorated with @nova.program or @program (imported from nova)"
            )

        logger.info("Program validation successful")

    async def _setup_environment(self):
        """Create program file and make it executable."""
        logger.info(f"Setting up program at {self.program_file}")

        # Validate program structure
        await self._validate_program()

        # Ensure uv is installed
        await self._install_uv()

        # Write program file
        self.program_file.write_text(self._program.content)
        self.program_file.chmod(0o755)  # equivalent to chmod u+x

        logger.info("Environment setup complete")

    async def _cleanup(self):
        """Clean up the temporary environment."""
        try:
            import shutil

            shutil.rmtree(self.project_dir)
            logger.info("Cleaned up temporary environment")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

    async def _run(self, execution_context: ExecutionContext):
        """Create a runner, execute the program, and clean up."""
        try:
            await self._setup_environment()

            try:
                # Convert parameters to environment variables
                env = os.environ.copy()
                env["PYTHONPATH"] = str(self.project_dir)
                if (api := os.getenv("NOVA_API")) is not None:
                    env["NOVA_API"] = api
                if (token := os.getenv("NOVA_ACCESS_TOKEN")) is not None:
                    env["NOVA_ACCESS_TOKEN"] = token

                self.program_file.write_text(self._program.content)

                # Convert parameters to command line arguments
                args = []
                for key, value in self._args.items():
                    if isinstance(value, (dict, list)):
                        # For complex types, convert to JSON string
                        args.extend([f"--{key}", json.dumps(value)])
                    else:
                        # For simple types, convert to string
                        args.extend([f"--{key}={str(value)}"])

                # Build the command for logging
                cmd = ["uv", "run", "--script", str(self.program_file)] + args
                logger.info(f"Running command: {' '.join(cmd)}")

                # TODO: pass execution context to the program
                # Run the program using uv
                process = await asyncio.create_subprocess_exec(
                    *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE, env=env
                )

                # Start timing
                start_time = datetime.datetime.now()

                # Create tasks to read stdout and stderr streams
                async def read_stream(stream, prefix):
                    while True:
                        line = await stream.readline()
                        if not line:
                            break
                        logger.info(f"{prefix}: {line.decode().strip()}")

                # Run both stream readers concurrently
                await asyncio.gather(
                    read_stream(process.stdout, "stdout"), read_stream(process.stderr, "stderr")
                )

                # Wait for process to complete
                return_code = await process.wait()

                end_time = datetime.datetime.now()
                execution_time = end_time - start_time
                logger.info(f"Program execution time: {execution_time}")

                if return_code != 0:
                    error_msg = f"Program failed with exit code {return_code}"
                    raise RuntimeError(error_msg)

                logger.info("Program execution completed successfully")

            except Exception as e:
                error_msg = f"Error running program: {str(e)}\nTraceback:\n{traceback.format_exc()}"
                logger.error(error_msg)
                raise RuntimeError(error_msg) from e
        finally:
            await self._cleanup()


# Dummy server endpoint
async def run_program_endpoint(program_content: str, args: dict):
    """REST endpoint handler for running programs."""
    try:
        logger.info(f"Running program with args: {args}")
        # TODO: provide context (nova, ...) to the execution
        runner = UVProgramRunner(
            program=Program(content=program_content, program_type=ProgramType.PYTHON), args=args
        )
        runner.start()
        return {"status": "success"}
    except Exception as e:
        error_msg = f"Failed to run program: {str(e)}\nTraceback:\n{traceback.format_exc()}"
        logger.error(error_msg)
        return {"status": "error", "message": str(e), "traceback": traceback.format_exc()}
