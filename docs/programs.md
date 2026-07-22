# Deploying Programs with NOVAx

This guide is a quickstart for writing robot **programs** and deploying them with
**NOVAx**, the app framework in the Wandelbots NOVA Python SDK. A NOVAx app is a
FastAPI service that discovers your `@nova.program` functions, exposes them over HTTP,
and registers them with a NOVA cell so they appear in the NOVA frontend.

- [Deploying Programs with NOVAx](#deploying-programs-with-novax)
  - [What is a program?](#what-is-a-program)
  - [Prerequisites](#prerequisites)
  - [1. Write a program](#1-write-a-program)
  - [2. Run it locally (no app needed)](#2-run-it-locally-no-app-needed)
  - [3. Scaffold a NOVAx app](#3-scaffold-a-novax-app)
  - [4. Register programs](#4-register-programs)
  - [5. Serve the app](#5-serve-the-app)
  - [6. Deploy to a NOVA instance](#6-deploy-to-a-nova-instance)
    - [Hot reload during development (Skaffold)](#hot-reload-during-development-skaffold)
  - [Configuration reference](#configuration-reference)
  - [Troubleshooting](#troubleshooting)

## What is a program?

A **program** is an `async` function decorated with `@nova.program`. The decorator turns
the function into an executable, self-describing unit: it derives an input schema from
the function signature, a name/description from the docstring, and can declare
**preconditions** (e.g. the controllers it needs). Programs register themselves in a
global registry when their module is imported, which is how NOVAx discovers them.

Every program takes a `ctx: nova.ProgramContext` as its **first parameter**. The context
gives you a connected `nova` instance, the `cell`, and cycle helpers.

## Prerequisites

- Python `>=3.11, <3.13`
- [`uv`](https://docs.astral.sh/uv/getting-started/installation/) installed
- The `novax` extra installed:

  ```bash
  uv pip install 'wandelbots-nova[novax]'
  # or in a project managed by uv:
  uv sync --extra novax
  ```

- Access to a NOVA instance. Set these environment variables (e.g. in a `.env` file):

  ```bash
  NOVA_API=https://your-instance.wandelbots.io   # base URL of your NOVA instance
  NOVA_ACCESS_TOKEN=...                           # access token
  CELL_NAME=cell                                  # NOVA cell to register programs in
  ```

## 1. Write a program

Create a file, e.g. `programs/hello_robot.py`:

```python
import nova
from nova import api
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.types import Pose


@nova.program(
    id="hello_robot",              # unique id; defaults to the function name
    name="Hello Robot",            # human-readable name shown in the UI
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        cleanup_controllers=False,  # keep the controller after the program ends
    ),
)
async def hello_robot(ctx: nova.ProgramContext, offset: float = 100.0):
    """Move the robot a bit along its x-axis and back.

    The docstring becomes the program description. Typed parameters (like
    ``offset``) become the program's input schema automatically.
    """
    cell = ctx.cell
    controller = await cell.controller("ur10e")
    motion_group = controller[0]

    home_joints = await motion_group.joints()
    tcp = (await motion_group.tcp_names())[0]
    current_pose = await motion_group.tcp_pose(tcp)
    target_pose = current_pose @ Pose((offset, 0, 0, 0, 0, 0))

    actions = [
        joint_ptp(home_joints),
        cartesian_ptp(target_pose),
        joint_ptp(home_joints),
    ]

    trajectory = await motion_group.plan(actions, tcp)
    await motion_group.execute(trajectory, tcp, actions=actions)
```

Key points:

- The first parameter **must** be named `ctx`.
- Typed parameters with defaults (like `offset`) become the program's input schema and
  are surfaced in the API and NOVA UI.
- `preconditions` declares controllers the program needs; NOVA ensures they exist before
  running. Use `virtual_controller(...)` for a simulated robot, or real controllers on
  your instance.

## 2. Run it locally (no app needed)

To iterate quickly without scaffolding an app, serve your programs straight from a short
script — no FastAPI boilerplate:

```python
from nova import Novax

if __name__ == "__main__":
    novax = Novax(programs_dir="programs")  # scan ./programs
    novax.serve(port=3000)                  # register programs and run uvicorn
```

Run it against your live NOVA, then open the API docs at `http://localhost:3000` and
execute the program from there.

You can also run a single program directly as a script, without any server:

```python
from nova import run_program

if __name__ == "__main__":
    run_program(hello_robot, inputs={"offset": 150.0})
```

## 3. Scaffold a NOVAx app

For a deployable app, scaffold one with the
[NOVA CLI](https://github.com/wandelbotsgmbh/nova-cli):

```bash
nova app create "your-nova-app" -g python_app
```

This creates a FastAPI app wired to NOVAx. The important line is:

```python
from pathlib import Path

from fastapi import FastAPI
from nova import Novax

app = FastAPI(title="your-nova-app")

# Include the programs router and scan a directory for @nova.program modules.
novax = Novax(app, programs_dir=Path(__file__).parent / "programs")
```

`Novax(app)` includes the programs router and auto-registers every already-imported
``@nova.program``. Pass ``programs_dir`` to also scan a directory for programs; without
it no directory is scanned.

## 4. Register programs

A program is only available once the module defining it has been imported. There are
three ways to make that happen — they can be combined:

- **Directory scanning.** When you pass ``programs_dir``, NOVAx recursively imports every
  `.py` file under it, so **dropping a new file into that directory is enough** — no manual
  import. Files whose name starts with `_` (e.g. `__init__.py`) are skipped; a missing
  directory is ignored.

  ```python
  Novax(app)                                     # no scanning (default)
  Novax(app, programs_dir="my_pkg/robot_progs")  # scan a directory
  Novax(app, programs_dir=None)                  # explicit: no scanning
  ```

- **Plain import.** Any program imported anywhere is registered:

  ```python
  import my_pkg.special_program  # noqa: F401  (registers on import)
  ```

- **Explicit.** Import a module/file and register its programs on demand:

  ```python
  novax.register_module("my_pkg.programs")
  ```

## 5. Serve the app

During development:

```bash
uv run app            # runs the app defined by the scaffold
# docs available at http://localhost:3000/docs
```

Or serve directly from a `Novax` instance without extra FastAPI boilerplate:

```python
from nova import Novax

novax = Novax(programs_dir="programs")  # scan ./programs
novax.serve(port=3000)   # builds the app, registers programs, runs uvicorn
```

When `CELL_NAME` is set, NOVAx connects to that cell on startup and syncs your programs
to the NOVA store so they show up in the NOVA frontend. If `CELL_NAME` is unset, the app
runs in local dev mode and programs are served but not synced to NOVA.

## 6. Deploy to a NOVA instance

Build, push, and install the app onto your instance:

```bash
nova app install .
```

This builds the container image, pushes it, and registers the app (including its
home-screen tile) on the cell. Your programs then appear in the NOVA frontend and can be
executed from there.

### Hot reload during development (Skaffold)

To develop against a real instance with hot reload, the scaffold ships a Skaffold setup.
Editing a program file syncs it into the running pod, `uvicorn --reload` restarts, and
the program re-registers on the instance — no image rebuild per change:

```bash
export KUBECONFIG=/path/to/kubeconfig
skaffold dev
```

Skaffold deploys an `App` custom resource that the in-cluster app-operator reconciles
into a Deployment/Service/Ingress and registers the home-screen tile — the same thing
`nova app install` does — so `skaffold dev` alone is enough.

## Configuration reference

| Variable            | Purpose                                                            |
| ------------------- | ------------------------------------------------------------------ |
| `NOVA_API`          | Base URL of the NOVA instance (required).                          |
| `NOVA_ACCESS_TOKEN` | Authentication token. Keep it out of committed `.env` files.       |
| `CELL_NAME`         | NOVA cell to register/sync programs in. Unset ⇒ local dev only.    |
| `BASE_PATH`         | ASGI root path / app name when served behind a proxy prefix.       |
| `LOG_LEVEL`         | Log verbosity (e.g. `info`, `debug`).                              |

Common `Novax` options:

| API                                  | Description                                              |
| ------------------------------------ | ------------------------------------------------------- |
| `Novax(app)`                         | Wire router and auto-register imported programs (no scan). |
| `Novax(app, programs_dir=...)`       | Also scan a directory; `None` disables it (the default). |
| `novax.serve(host=..., port=...)`    | Build the app and run uvicorn in one call.              |
| `novax.register_module(mod)`         | Import a module/file and register its programs.         |
| `novax.scan_programs(directory)`     | Import a directory's programs and register them.        |

## Troubleshooting

- **Program not showing up?** Make sure its module is imported — put the file under a
  directory passed as `programs_dir`, import it explicitly, or call `register_module`.
- **`ImportError` about the `novax` extra.** Install it with
  `uv pip install 'wandelbots-nova[novax]'` (or `uv sync --extra novax`).
- **Programs run locally but don't appear in NOVA.** Set `CELL_NAME` so NOVAx syncs them
  to the cell's program store on startup.
- **`Program function must have 'ctx' as its first parameter`.** The first argument of a
  `@nova.program` function must be named `ctx`.
- **No NOVA instance provided for execution.** Run programs through the app/CLI, via
  `run_program(...)`, or pass a connected `nova=...` instance.
