# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Official Python SDK for the Wandelbots NOVA API. NOVA is a robot-agnostic operating system that enables developers to plan, program, control, and operate fleets of six-axis industrial robots through a unified API, across all major robot brands.
## Project Overview


**Requirements**: Python >=3.11, <3.13

## Build & Development Commands

```bash
# Install dependencies
uv sync --extra "nova-rerun-bridge"
# Format code
uv run ruff format

# Lint and import sorting
uv run ruff check --select I --fix
uv run ruff check .

# Type checking
uv run mypy

PYTHONPATH=. uv run pytest -rs -v -m "not integration"
# Run all unit tests (excluding integration tests)

# Run a single test file
PYTHONPATH=. uv run pytest -rs -v path/to/test_file.py

# Run a specific test
PYTHONPATH=. uv run pytest -rs -v path/to/test_file.py::test_function_name

# Pre-commit hooks
pre-commit run --files <changed files>

# Build package
uv build

# Download robot models for visualization
uv run download-models

## Architecture Overview


This SDK enables Python developers to control industrial robots through the Wandelbots NOVA API.
### Core Packages
- **nova/**: Main SDK package

  - `Nova` - High-level client entry point (uses async context manager pattern)
  - `Cell` - Represents a robot cell containing controllers
  - `Controller` - Manages robot controller hardware connections
  - `MotionGroup` - Handles motion planning and execution for a robot arm
  - `actions/` - Motion primitives: `ptp`, `lin`, `jnt`, `cir`, `collision_free`, `io_write`, `wait`
  - `types/` - Data types: `Pose`, `RobotState`, `MotionSettings`, `CollisionScene`
  - `program/` - `@program` decorator for defining executable robot programs
  - `api.py` - Auto-generated API client (via `wandelbots_api_client`)

- **novax/**: App framework for building FastAPI server applications
  - `Novax` - FastAPI integration with program registration and lifecycle management
  - Used with `nova app create` CLI to scaffold new apps

- **wandelscript/**: Domain-specific language for robot programming
  - Grammar defined in `wandelscript/grammar/` (ANTLR4)
  - Runtime execution in `runtime.py`
  - Built-in functions in `builtins/`
# Run Wandelscript CLI
uv run wandelscript my_script.ws
uv run ws my_script.ws  # shortcut
```

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
    cell = ctx.cell
async def my_program(ctx: nova.ProgramContext):
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
