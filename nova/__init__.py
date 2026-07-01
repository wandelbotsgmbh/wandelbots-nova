# Import api, types, and actions modules
from typing import TYPE_CHECKING

from nova import actions, api, exceptions, types, viewers
from nova.cell import Cell, Controller, MotionGroup
from nova.config import NovaConfig
from nova.core.nova import Nova
from nova.logging import logger
from nova.program import ProgramContext, ProgramPreconditions, program, run_program
from nova.version import version

if TYPE_CHECKING:
    # Declared for static analysis / __all__ export checks only. At runtime
    # `Novax` is provided lazily via __getattr__ so the optional `novax` extra
    # stays optional; defining it at module scope would shadow __getattr__.
    from novax import Novax

__version__ = version

__all__ = [
    "Nova",
    "NovaConfig",
    "Cell",
    "Controller",
    "MotionGroup",
    "api",
    "exceptions",
    "types",
    "actions",
    "viewers",
    "logger",
    "program",
    "run_program",
    "ProgramContext",
    "get_current_program_context",
    "ProgramPreconditions",
    "Novax",
    "__version__",
]


def __getattr__(name: str):
    # Lazily expose Novax from the optional `novax` extra so users can do
    # `from nova import Novax` only when it is installed. Kept lazy to avoid a
    # hard dependency on FastAPI in the core SDK.
    if name == "Novax":
        from novax import Novax

        return Novax
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def get_current_program_context() -> ProgramContext | None:
    """Get the current program context from within a running program.

    Returns the active ProgramContext when called from inside a function
    decorated with @program. The context provides access to the Nova instance,
    the default cell, and the program ID.

    Returns:
        The current ProgramContext if called within a @program-decorated function,
        or None if called outside of a program execution context.
    """
    from nova.program.context import current_program_context_var

    return current_program_context_var.get()
