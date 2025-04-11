import abc
from functools import wraps

import pydantic

from nova import actions, api, types
from nova.core.controller import Controller
from nova.core.logging import logger
from nova.core.motion_group import MotionGroup
from nova.core.movement_controller import \
    speed_up as speed_up_movement_controller
from nova.core.nova import Cell, Nova
from nova.types import MotionSettings
from nova.version import version

__version__ = version


class ProgramParameter(pydantic.BaseModel, abc.ABC):
    pass


def program(parameter: type[ProgramParameter], name: str | None = None):
    @wraps
    async def wrapped_function(func):
        if not callable(func):
            raise TypeError("The function must be callable.")

        async with Nova() as nova:
            # TODO: should we pass nova completely
            # TODO: run function in Nova runtime
            start_time = nova.time()
            await func(nova_context=nova, kwargs=parameter.model_dump())
            end_time = nova.time()
            exection_time = end_time - start_time
            logger.info(f"Execution time: {exection_time}")
    return wrapped_function


__all__ = [
    "Nova",
    "Cell",
    "MotionGroup",
    "Controller",
    "speed_up_movement_controller",
    "api",
    "types",
    "actions",
    "MotionSettings",
    "logger",
    "__version__",
]
