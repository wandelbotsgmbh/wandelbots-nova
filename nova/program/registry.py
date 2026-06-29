"""Global registry of programs created via the ``@nova.program`` decorator.

Every program created with :func:`nova.program` is automatically added here so that
tools like Novax can discover all programs in a module without explicit registration
calls. The registry is keyed by ``program_id``; re-decorating with the same id
replaces the previous entry.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nova.program.function import Program

_REGISTRY: dict[str, "Program"] = {}


def register(program: "Program") -> None:
    """Add (or replace) a program in the global registry."""
    _REGISTRY[program.program_id] = program


def get_registered_programs() -> list["Program"]:
    """Return all programs registered via ``@nova.program``."""
    return list(_REGISTRY.values())


def clear_registry() -> None:
    """Remove all programs from the registry (useful for tests)."""
    _REGISTRY.clear()
