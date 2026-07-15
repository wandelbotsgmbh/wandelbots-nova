"""Command-line entry point for serving Novax programs during development.

Usage:
    novax run [module-or-file] [--host HOST] [--port PORT] [--cell CELL]

Imports the given module/file (which defines ``@nova.program`` functions) if provided,
scans the ``programs`` directory, registers all discovered programs and serves them
against a live NOVA — no Docker build required. With no target it just scans
``programs``.
"""

import argparse
import os


def main() -> None:
    parser = argparse.ArgumentParser(prog="novax", description="Serve NOVA programs locally")
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="Import programs and serve them locally")
    run.add_argument(
        "target",
        nargs="?",
        default=None,
        help="Optional dotted module path or path to a .py file with programs. "
        "When omitted, the 'programs' directory is scanned.",
    )
    run.add_argument("--host", default="0.0.0.0", help="Host to bind (default: 0.0.0.0)")
    run.add_argument("--port", type=int, default=3000, help="Port to bind (default: 3000)")
    run.add_argument("--cell", default=None, help="NOVA cell to register programs in")

    args = parser.parse_args()

    if args.command == "run":
        if args.cell:
            os.environ["CELL_NAME"] = args.cell
        # Import lazily so CELL_NAME is set before novax.config reads it.
        # Import Novax via `nova` so a missing `novax` extra surfaces the clear
        # install hint from novax/__init__.py instead of a raw ModuleNotFoundError.
        from nova import Novax
        from novax.novax import _import_module

        if args.target:
            _import_module(args.target)
        novax = Novax()
        novax.serve(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
