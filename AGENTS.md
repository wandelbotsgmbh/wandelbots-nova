# AGENT Guidelines

This repository contains the official NOVA Python SDK. The main package resides in `nova`, and additional visualization features live in `nova_rerun_bridge` which can be installed via the `nova-rerun-bridge` extra.

The project uses [uv](https://docs.astral.sh/uv/) for dependency management. Ensure you have `uv` installed locally and sync dependencies before running checks:

```bash
uv sync --extra "nova-rerun-bridge"
```

## Checks

Run the following commands before committing any changes:

```bash
# Format code
uv run ruff format

# Lint and import sorting
uv run ruff check --select I --fix
uv run ruff check .

# Type checking
uv run mypy

# Tests
PYTHONPATH=. uv run pytest -rs -v -m "not integration"
```

You may also use `pre-commit` to automatically run these checks:

```bash
pre-commit run --files <changed files>
```

## Build

Build the package locally using:

```bash
uv build
```

Pull request titles should follow the pattern `chore|feat|fix[(scope)]: Description`.
