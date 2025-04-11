import abc
import datetime
import subprocess
import sys
from functools import wraps
from typing import Callable

import dotenv
import pydantic
from loguru import logger

from nova import Nova

dotenv.load_dotenv()


class ProgramParameter(pydantic.BaseModel, abc.ABC):
    pass


def install_dependencies():
    """Install dependencies using uv."""
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "uv"])
        subprocess.check_call(["uv", "install"])
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to install dependencies: {e}")
        sys.exit(1)


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


async def run(program: Callable, args: type[ProgramParameter]):
    # Create a Nova context and run the program
    async with Nova() as nova:
        await program(nova_context=nova, args=args)