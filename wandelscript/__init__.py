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

from wandelscript import _geometricalgebra_compat

_geometricalgebra_compat.apply()

from wandelscript import builtins, motions  # noqa: E402
from wandelscript.metamodel import Program, register_builtin_func  # noqa: E402
from wandelscript.runner import (  # noqa: E402
    WandelscriptProgramRunner,
    run,
    run_wandelscript_program,
)
from wandelscript.runtime import Store  # noqa: E402
from wandelscript.version import version  # noqa: E402

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
