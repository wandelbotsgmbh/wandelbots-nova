import abc
import asyncio
import datetime
import os
import subprocess
import sys
import tempfile
import venv
from functools import wraps
from pathlib import Path
from typing import Callable, Optional

import dotenv
import pydantic
from loguru import logger

from nova import Nova

dotenv.load_dotenv()


class ProgramParameter(pydantic.BaseModel, abc.ABC):
    pass


class SandboxedProgramRunner:
    def __init__(self, program_text: str):
        self.program_text = program_text
        self.project_dir = Path(tempfile.mkdtemp())
        self.program_file = self.project_dir / "program.py"

    async def install_uv(self):
        """Install uv if not already installed."""
        try:
            # Check if uv is already installed
            process = await asyncio.create_subprocess_exec(
                "uv", "--version",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            await process.communicate()

            if process.returncode == 0:
                logger.info("uv is already installed")
                return

        except FileNotFoundError:
            logger.info("Installing uv...")

            # Install uv using pip
            process = await asyncio.create_subprocess_exec(
                sys.executable, "-m", "pip", "install", "uv",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, stderr = await process.communicate()

            if process.returncode != 0:
                raise RuntimeError(f"Failed to install uv: {stderr.decode()}")

            logger.info("uv installed successfully")

    async def setup_environment(self):
        """Create program file and make it executable."""
        logger.info(f"Setting up program at {self.program_file}")

        # Ensure uv is installed
        await self.install_uv()

        # Create program file with shebang
        if not self.program_text.startswith("#!/usr/bin/env"):
            self.program_text = "#!/usr/bin/env -S uv run --script\n" + self.program_text

        # Write program file
        self.program_file.write_text(self.program_text)

        # Make program file executable
        self.program_file.chmod(0o755)  # equivalent to chmod u+x

        logger.info("Environment setup complete")

    async def run_program(self, parameter_instance: ProgramParameter):
        """Run the program using uv with the given parameters."""
        try:
            # Convert parameters to JSON for passing to the subprocess
            params_json = parameter_instance.model_dump_json()

            # Create a wrapper script that will run the program with the parameters
            wrapper_script = f"""#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = ["wandelbots-nova"]
# ///

import asyncio
import json
from pathlib import Path
import sys

# Add the program directory to the path
program_dir = Path(__file__).parent
sys.path.append(str(program_dir))

# Import the program
from program import main

# Load parameters
params = json.loads('{params_json}')

# Run the program
asyncio.run(main(**params))
"""
            wrapper_file = self.project_dir / "wrapper.py"
            wrapper_file.write_text(wrapper_script)
            wrapper_file.chmod(0o755)

            # Run the program using uv
            logger.info("Starting program execution")
            process = await asyncio.create_subprocess_exec(
                str(wrapper_file),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env={
                    "PYTHONPATH": str(self.project_dir),
                    "PATH": os.environ["PATH"]
                }
            )

            # Capture output
            stdout, stderr = await process.communicate()

            if stdout:
                logger.info(f"Program output:\n{stdout.decode()}")
            if stderr:
                logger.error(f"Program errors:\n{stderr.decode()}")

            if process.returncode != 0:
                raise RuntimeError(f"Program failed with exit code {process.returncode}")

            logger.info("Program execution completed successfully")

        except Exception as e:
            logger.error(f"Error running program: {e}")
            raise

    async def cleanup(self):
        """Clean up the temporary environment."""
        try:
            import shutil
            shutil.rmtree(self.project_dir)
            logger.info("Cleaned up temporary environment")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

    @classmethod
    async def run(cls, program_text: str, parameter_instance: ProgramParameter):
        """Create a runner, execute the program, and clean up."""
        runner = cls(program_text)
        try:
            await runner.setup_environment()
            await runner.run_program(parameter_instance)
        finally:
            await runner.cleanup()

def define_program(parameter: type[ProgramParameter], name: str | None = None):
    def decorator(func):
        @wraps(func)
        async def wrapped_function(*, nova_context: Nova, args: ProgramParameter, **kwargs):
            if not isinstance(args, parameter):
                raise TypeError(f"Arguments must be an instance of {parameter.__name__}")

            logger.info(f"Starting program: {name or func.__name__}")
            try:
                start_time = datetime.datetime.now()
                result = await func(nova_context=nova_context, **args.model_dump(), **kwargs)
                end_time = datetime.datetime.now()
                execution_time = end_time - start_time
                logger.info(f"Program completed successfully: {name or func.__name__}")
                logger.info(f"Execution time: {execution_time}")
                return result
            except Exception as e:
                logger.error(f"Program failed: {name or func.__name__} with error: {e}")
                raise
        return wrapped_function
    return decorator


async def run_program_endpoint(program_text: str, parameters: dict):
    """REST endpoint handler for running programs."""
    try:
        param_instance = ProgramParameter(**parameters)
        await SandboxedProgramRunner.run(program_text, param_instance)
        return {"status": "success"}
    except Exception as e:
        logger.error(f"Failed to run program: {e}")
        return {"status": "error", "message": str(e)}
