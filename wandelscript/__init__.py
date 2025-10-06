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

import wandelscript.antlrvisitor  # load Program.from_code # noqa: F401
from wandelscript import builtins, motions
from wandelscript.metamodel import Program, register_builtin_func
from wandelscript.runner import WandelscriptProgramRunner, run, run_wandelscript_program
from wandelscript.runtime import Store
from wandelscript.version import version

__version__ = version


def analyze(code: str):
    Program.from_code(code)


__all__ = [
    "run",
    "Program",
    "WandelscriptProgramRunner",
    "Store",
    "__version__",
    "register_builtin_func",
    "motions",
    "builtins",
    "run_wandelscript_program",
]
