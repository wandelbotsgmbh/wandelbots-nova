#!/usr/bin/env python3
"""
cli.py

Extensible command‑line interface **powered by asyncio**.

Usage examples
--------------
$ ./cli.py sync program --to local
$ ./cli.py sync program --to nova

The hierarchy is:
  • <command>  – top‑level commands (e.g. `sync`)
  • <sub‑command> – nested within a command (e.g. `program` under `sync`)

Extend the CLI by following the notes in *_add_sync_subcommands()* or by
creating new *_add_<command>_subcommands()* helpers.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
from typing import Callable, List
from pathlib import Path
import aiohttp


from nova import Nova 
import os

# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
LOGGER_NAME = "cli"
VALID_DESTINATIONS = ("local", "nova")


def configure_logging(level: int = logging.DEBUG) -> None:  # pragma: no cover
    """Configure root logger for consistent, structured output."""
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


logger = logging.getLogger(LOGGER_NAME)

# --------------------------------------------------------------------------- #
# Sync implementations (asyncio‑native)
# --------------------------------------------------------------------------- #


async def sync_program(destination: str | None) -> None:
    """Synchronise program files to *destination* (default: "local")."""
    destination = destination or "local"

    logger.info("Starting program sync → %s", destination)

    if destination not in VALID_DESTINATIONS:
        logger.error("Invalid destination %s", destination)
        sys.exit(1)

    if destination == "local":
        await _sync_program_local()
    elif destination == "nova":
        await _sync_program_nova()

    logger.info("Program sync complete ✔")


async def _sync_program_local() -> None:
    """Implementation details for syncing program to the local machine."""
    # TODO: Replace with production logic
    logger.debug("Executing local sync placeholder.")
    current_directory = os.getcwd()
    logger.debug("Current directory: %s", current_directory)

    async with Nova() as nova:
        cell = nova.cell()
        await cell.controllers()

async def _sync_program_nova() -> None:
    """Implementation details for syncing program to Nova."""
    scripts_folder = os.getcwd() + "/scripts"
    files = read_python_files(scripts_folder)
    if not files:
        logger.warning("No Python files found in the specified directory.")
        return
    logger.info("Found %d Python files to upload.", len(files))

    await upload_files_to_nova(files)

async def create_program(file_name: str, file_content: str) -> None:
    """Create a program in Nova with the given file name and content."""
    async with Nova() as nova:
        cell = nova.cell()
        await cell.controllers() # just to force token refrest
        api_client_config = nova._api_client._api_client.configuration
        url = f'{api_client_config.host}/cells/cell/store/programs'
        params = {
            'name': file_name
        }
        headers = {
            'Accept': 'application/json, text/plain',
            'Authorization': f'Bearer {api_client_config.access_token}',
            'Content-Type': 'text/plain'
        }
        data = file_content

    import requests
    response = requests.post(url, data=data, headers=headers, params=params)
    print(response.status_code)
    print(response.text)

async def upload_files_to_nova(files: dict[str, str]) -> None:
    from nova import Nova
    async with Nova() as nova:
        program_metadata_list = await nova._api_client.program_library_metadata_api.list_program_metadata(cell="cell")
        program_metadata_list = program_metadata_list.programs

        for file_name, file_content in files.items():
            # Check if the file already exists in the Nova program library
            if any(metadata.name == file_name for metadata in program_metadata_list):
                logger.info(f"File {file_name} already exists in Nova program library. Updating...")
                program_id = [metadata.id for metadata in program_metadata_list if metadata.name == file_name][0]
                await nova._api_client.program_library_api.update_program(
                    cell="cell",
                    program=program_id,
                    body=file_content
                )
                continue

            logger.info(f"Program {file_name} not found in Nova program library. Uploading...")
            logger.info(f'file content is {isinstance(file_content, str)}')
            await create_program(file_name, file_content)
            logger.info(f"Uploaded {file_name} to Nova program library.")


def read_python_files(directory_path):
    """
    Reads all .py files in the given directory and returns a dictionary 
    with filename as key and file content as value.

    :param directory_path: str or Path object of the directory to read
    :return: dict mapping filename to file content
    """
    directory = Path(directory_path)
    if not directory.is_dir():
        raise ValueError(f"The provided path '{directory_path}' is not a valid directory.")
    
    python_files = directory.glob('*.py')
    file_content_map = {}

    for file in python_files:
        try:
            with file.open('r', encoding='utf-8') as f:
                content = f.read()
            file_content_map[file.name] = content
        except Exception as e:
            print(f"Error reading file {file.name}: {e}")

    return file_content_map

# --------------------------------------------------------------------------- #
# Parser helpers
# --------------------------------------------------------------------------- #
SubcommandFunc = Callable[[argparse.Namespace], None]


def _add_sync_subcommands(sync_subparsers: argparse._SubParsersAction) -> None:
    """Register all `sync` sub‑commands.

    Steps to add *another* sync sub‑command (e.g. `robots`):
        1. Implement `sync_robots()` function.
        2. Add a new parser:
               robots_parser = sync_subparsers.add_parser("robots", help="Sync robots")
        3. Register any arguments, then:
               robots_parser.set_defaults(func=lambda ns: sync_robots(ns.<arg>))
    """

    # sync program
    program_parser = sync_subparsers.add_parser(
        "program", help="Synchronise program files."
    )
    program_parser.add_argument(
        "--to",
        choices=VALID_DESTINATIONS,
        metavar="DEST",
        help="Destination to sync to (default: 'local').",
    )
    program_parser.set_defaults(func=lambda ns: sync_program(ns.to))


def build_parser(argv: List[str] | None = None) -> argparse.ArgumentParser:
    """Construct and return the top‑level argument parser."""
    parser = argparse.ArgumentParser(
        prog="cli",
        description="Extensible command‑line utility.",
    )
    subparsers = parser.add_subparsers(
        title="commands", dest="command", required=True
    )

    # Parent command: sync
    sync_parser = subparsers.add_parser("sync", help="Synchronisation utilities.")
    sync_subparsers = sync_parser.add_subparsers(
        title="sync commands", dest="sync_command", required=True
    )

    # Register individual sync sub‑commands
    _add_sync_subcommands(sync_subparsers)

    return parser


# --------------------------------------------------------------------------- #
# Entry point helpers
# --------------------------------------------------------------------------- #


async def _dispatch_async(ns: argparse.Namespace) -> None:
    """Await the function associated with the resolved argparse namespace."""
    if not hasattr(ns, "func") or not callable(ns.func):  # pragma: no cover
        raise SystemExit("No command specified — use -h for help.")
    await ns.func(ns)  # type: ignore[arg-type]


async def _async_main(argv: List[str] | None = None) -> None:  # pragma: no cover
    """Async CLI entry point — parse *argv* and dispatch to handlers."""
    configure_logging()
    parser = build_parser(argv)
    ns = parser.parse_args(argv)
    await _dispatch_async(ns)


def main(argv: List[str] | None = None) -> None:  # pragma: no cover
    """Synchronously invoked entry that boots the asyncio event loop."""
    asyncio.run(_async_main(argv))


if __name__ == "__main__":  # pragma: no cover
    main()
