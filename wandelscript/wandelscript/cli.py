#!/usr/bin/env python3
"""CLI tool to work with Wandelscript."""

import asyncio
import hashlib
import importlib
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from types import ModuleType
from urllib.parse import urlparse

from dotenv import load_dotenv
from icecream import ic
from typer import Exit, FileText, Option, Typer, echo

import wandelscript
from wandelscript import ffi

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now().time().isoformat()} | ")

load_dotenv()

app = Typer()


def _validate_url(url: str) -> bool:
    """Validate the provided URL. Return True if valid, otherwise return False."""
    parsed = urlparse(url)
    if parsed.scheme and parsed.netloc:
        return True
    return False


@dataclass
class _ForeignFunctionHandle:
    function: ffi.ForeignFunction
    path: Path


def _import_module_from_file(path: Path) -> ModuleType:
    # Generate a unique name for the module based on the full path.
    module_name = f"ffi_module_{path.stem}_{hashlib.sha1(str(path.resolve()).encode()).hexdigest()}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        echo(f"Error: Could not load module from {path}", err=True)
        raise Exit(1)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _import_module_from_path(path: Path) -> ModuleType:
    # We assume the module_path is in the form "path/to/module.py" or "path/to/module/__init__.py"
    # Then we consider everything before the last '/' as the path and everything after as the module name
    path_str = str(path)
    path_part_str, dot_module = path_str.rsplit("/", 1)
    path_part = Path(path_part_str).resolve()
    if not path_part.exists():
        echo(f"Error: Path {path_part} to module {dot_module} does not exist", err=True)
        raise Exit(1)

    echo(f"Importing module {dot_module} from path {path_part}")
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

    echo(f"Importing foreign functions from {path}")
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
    echo(
        f"Found {len(foreign_functions)} marked function(s): {', '.join([handle.function.name for handle in foreign_functions])}"
    )
    return foreign_functions


def _load_included_ffs(paths: list[Path]) -> dict[str, ffi.ForeignFunction]:
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
                echo(
                    f"Error: Foreign function '{func_name}' from {already_seen_handle.path} redefined in {path}",
                    err=True,
                )
                raise Exit(1)
            already_seen[func_name] = handle
            foreign_functions[func_name] = handle.function
    return foreign_functions


async def main(
    code: str, nova_api: str, foreign_functions: dict[str, ffi.ForeignFunction] | None = None
):
    """Main program logic."""
    runner = wandelscript.run(code, args={}, default_tcp=None, default_robot=None)
    echo(f"Execution results:\n{runner.program_run.execution_results}")


@app.command()
def run(
    script: FileText,
    nova_api: str = Option(None, "--nova-api", "-n", envvar="NOVA_API", help="URL to NOVA API"),
    import_ffs: list[Path] = Option(
        None,
        "--import-ffs",
        "-i",
        help="Python file or module path to load foreign functions from before executing the program. Can be specified multiple times.",
    ),
):
    """Run Wandelscript programs."""

    if not nova_api:
        echo("Error: NOVA_API must be set via '--nova-api' or as an environment variable", err=True)
        raise Exit(1)
    if not _validate_url(nova_api):
        echo(f"Error: NOVA_API value {nova_api} is not a valid URL", err=True)
        raise Exit(1)

    echo(f"NOVA_API: {nova_api}")

    foreign_functions = _load_included_ffs(import_ffs)

    code = script.read()
    script.close()

    asyncio.run(main(code=code, nova_api=nova_api, foreign_functions=foreign_functions))


if __name__ == "__main__":
    app()
