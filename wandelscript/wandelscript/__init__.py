"""Wandelscript


Example:
>>> import asyncio
>>> from nova.types import Vector3d
>>> from wandelscript.metamodel import run_program
>>> code = 'a = (0, 1, 2) + (0, 0, 3)'
>>> context = asyncio.run(run_program(code))
>>> context.store['a']
Vector3d(x=0.0, y=1.0, z=5.0)
"""

import wandelscript.antlrvisitor  # load Program.from_code
import wandelscript.builtins
import wandelscript.motions  # load all motion connectors
from wandelscript.metamodel import Program, register_builtin_func
from wandelscript.runner import ProgramRunner, ProgramRunState, run, run_file
from wandelscript.runtime import ActionQueue, Store
from wandelscript.version import version

__version__ = version


def analyze(code: str):
    Program.from_code(code)


__all__ = [
    "run",
    "run_file",
    "Program",
    "ProgramRun",
    "ProgramRunner",
    "ProgramRunState",
    "Store",
    "__version__",
    "register_builtin_func",
]
