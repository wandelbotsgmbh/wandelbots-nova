# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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

# Run all unit tests (excluding integration tests)
PYTHONPATH=. uv run pytest -rs -v -m "not integration"

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

# Run Wandelscript CLI
uv run wandelscript my_script.ws
uv run ws my_script.ws  # shortcut
```

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
  - `api.py` - Auto-generated API client (via `wandelbots_api_client`)
  - `program/` - `@program` decorator for defining executable robot programs

- **novax/**: App framework for building FastAPI server applications
  - `Novax` - FastAPI integration with program registration and lifecycle management
  - Used with `nova app create` CLI to scaffold new apps

- **wandelscript/**: Domain-specific language for robot programming
  - Grammar defined in `wandelscript/grammar/` (ANTLR4)
  - Runtime execution in `runtime.py`
  - Built-in functions in `builtins/`

- **nova_rerun_bridge/**: 3D visualization using rerun.io

### Key Patterns

**Async Context Manager**: All robot operations use async/await:
```python
async with Nova() as nova:
    cell = nova.cell()
    controller = await cell.controller("robot")
    motion_group = controller.motion_group()
```

**Actions-based Motion**: Movements are defined as action sequences:
```python
from nova.actions import ptp, lin
actions = [ptp(start_pose), lin(target_pose)]
await motion_group.run(actions, tcp="Flange")
```

**Program Decorator**: Robot programs use `@program` for lifecycle management:
```python
from nova import program
@program
async def my_robot_program():
    ...
```

### Test Markers

- Default tests: Unit tests that don't require NOVA instance
- `@pytest.mark.integration`: Tests requiring a running NOVA instance
- `@pytest.mark.xdist_group(name)`: Tests that must run sequentially

### PR Title Convention

`chore|feat|fix[(scope)]: Description`
