# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Official Python SDK for the Wandelbots NOVA API. NOVA is a robot-agnostic operating system that enables developers to plan, program, control, and operate fleets of six-axis industrial robots through a unified API, across all major robot brands.

**Requirements**: Python >=3.11, <3.13

## Development Commands

```bash
# Setup (install uv first: https://docs.astral.sh/uv/)
uv sync --extra "nova-rerun-bridge"

# Formatting
uv run ruff format
uv run ruff check --select I --fix

# Linting and type checking
uv run ruff check .
uv run mypy

# Testing (excludes integration tests)
PYTHONPATH=. uv run pytest -rs -v -m "not integration"

# Pre-commit (alternative to running checks individually)
pre-commit run --files <changed_files>

# Build
uv build
```

## Architecture

### Package Structure
- **nova/** - Main SDK: `Nova` client, `Cell`, `Controller`, `MotionGroup` abstractions
- **nova/actions/** - Motion actions: `joint_ptp`, `cartesian_ptp`, `linear`, `circular`, `collision_free`
- **nova/program/** - `@nova.program` decorator framework for robot programs
- **nova/types/** - Core types: `Pose` (6D with matrix ops), `Vector3d`, `MotionSettings`
- **nova_rerun_bridge/** - Optional 3D visualization with Rerun (install with `--extra nova-rerun-bridge`)
- **novax/** - FastAPI-based app framework for NOVA applications (install with `--extra novax`)
- **wandelscript/** - DSL for declarative robot programming (install with `--extra wandelscript`)
- **vscode-ext/** - VS Code extension (TypeScript, separate Node.js project)

### Core Usage Pattern
```python
from nova import Nova

async with Nova() as nova:
    cell = nova.cell()
    controller = await cell.controller("robot-name")
    motion_group = controller[0]

    actions = [joint_ptp(joints), linear(pose)]
    trajectory = await motion_group.plan(actions, tcp_name)
    await motion_group.execute(trajectory, tcp_name, actions=actions)
```

### Program Decorator Pattern
```python
@nova.program(id="my-program", name="My Program")
async def my_program(ctx: nova.ProgramContext):
    cell = ctx.cell
    cycle = ctx.cycle()
```

## Code Style

- **Line length**: 100 characters
- **Formatting**: Black-compatible via ruff
- **Type hints**: Modern syntax (`list[T]` not `List[T]`)
- **Async**: Everything is async - always use `await` and `async with`
- **Banned**: `icecream` in production code (dev-only)

## Configuration

Environment variables (copy `.env.template` to `.env`):
- `NOVA_API` - Base URL of NOVA instance (required)
- `NOVA_ACCESS_TOKEN` - Authentication token (required for cloud, recommended for self-hosted)

## Commit Messages

PR titles and commits follow Angular convention: `chore|feat|fix[(scope)]: Description`
- `feat` → minor version bump
- `fix`/`chore` → patch version bump

## Testing

- Unit tests: `PYTHONPATH=. uv run pytest -rs -v -m "not integration"`
- Integration tests require running NOVA instance: `-m "integration"`
- Use `@pytest.mark.integration` marker for tests requiring NOVA
