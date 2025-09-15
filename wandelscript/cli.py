#!/usr/bin/env python3
"""CLI tool to work with Wandelscript."""

import asyncio
from datetime import datetime
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from dotenv import load_dotenv
from icecream import ic
from typer import Exit, FileText, Option, Typer, echo

import wandelscript
from wandelscript.ffi_loader import load_foreign_functions

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now().time().isoformat()} | ")

load_dotenv()

app = Typer()


def _validate_url(url: str) -> bool:
    """Validate the provided URL. Return True if valid, otherwise return False."""
    parsed = urlparse(url)
    if parsed.scheme and parsed.netloc:
        return True
    return False


async def main(code: str, foreign_functions: dict[str, Any] | None = None):
    """Main program logic."""
    runner = wandelscript.run(
        program_id="ws_program",
        program=code,
        args={},
        default_tcp=None,
        default_robot=None,
        foreign_functions=foreign_functions,
    )
    echo(f"Execution results:\n{runner.program_run.state}")


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

    foreign_functions = load_foreign_functions(import_ffs) if import_ffs else None

    code = script.read()
    script.close()

    asyncio.run(main(code=code, foreign_functions=foreign_functions))


if __name__ == "__main__":
    app()
