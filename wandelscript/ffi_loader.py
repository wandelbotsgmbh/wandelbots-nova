"""Foreign Function Interface loader module with logging-based messaging."""

import hashlib
import importlib
import importlib.util
import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType

from wandelscript import ffi

# Create a logger for this module
logger = logging.getLogger(__name__)


class FFILoaderError(Exception):
    """Exception raised when there's an error loading foreign functions."""


@dataclass
class _ForeignFunctionHandle:
    function: ffi.ForeignFunction
    path: Path


def _import_module_from_file(path: Path) -> ModuleType:
    """Import a module from a Python file."""
    # Generate a unique name for the module based on the full path.
    module_name = f"ffi_module_{path.stem}_{hashlib.sha1(str(path.resolve()).encode()).hexdigest()}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise FFILoaderError(f"Could not load module from {path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _import_module_from_path(path: Path) -> ModuleType:
    """Import a module from a path string."""
    # We assume the module_path is in the form "path/to/module.py" or "path/to/module/__init__.py"
    # Then we consider everything before the last '/' as the path and everything after as the module name
    path_str = str(path)
    path_part_str, dot_module = path_str.rsplit("/", 1)
    path_part = Path(path_part_str).resolve()
    if not path_part.exists():
        error_msg = f"Path {path_part} to module {dot_module} does not exist"
        raise FFILoaderError(error_msg)

    logger.info(f"Importing module {dot_module} from path {path_part}")
    # Temporarily add the path to sys.path so importlib can find it
    sys.path.insert(0, str(path_part))
    try:
        module = importlib.import_module(dot_module)
    finally:
        # Clean up sys.path regardless of import success
        sys.path.pop(0)
    return module


def _load_ffs_from_path(path: Path) -> list[_ForeignFunctionHandle]:
    """Load foreign functions from the provided Python file or module path."""
    logger.info(f"Importing foreign functions from {path}")

    if path.is_file() and path.suffix == ".py":
        module = _import_module_from_file(path)
    else:
        module = _import_module_from_path(path)

    foreign_functions = []
    for symbol in dir(module):
        if symbol.startswith("_"):
            # exclude private and magic symbols
            continue
        obj = getattr(module, symbol)
        if callable(obj) and (ff_obj := ffi.get_foreign_function(obj)) is not None:
            foreign_functions.append(_ForeignFunctionHandle(ff_obj, path))

    logger.info(
        f"Found {len(foreign_functions)} marked function(s): {', '.join([handle.function.name for handle in foreign_functions])}"
    )
    return foreign_functions


def load_foreign_functions(paths: list[Path]) -> dict[str, ffi.ForeignFunction]:
    """Load foreign functions from the provided paths."""
    already_seen: dict[str, _ForeignFunctionHandle] = {}
    foreign_functions = {}

    for path in paths:
        func_handles = _load_ffs_from_path(path)
        for handle in func_handles:
            func_name = handle.function.name
            if func_name in already_seen:
                # TODO do we want to allow overwriting builtins? Currently this is not prevented.
                already_seen_handle = already_seen[func_name]
                error_msg = f"Foreign function '{func_name}' from {already_seen_handle.path} redefined in {path}"
                raise FFILoaderError(error_msg)
            already_seen[func_name] = handle
            foreign_functions[func_name] = handle.function

    return foreign_functions
