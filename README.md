# wandelbots-nova (Python SDK)

[![PyPI version](https://badge.fury.io/py/wandelbots-nova.svg)](https://badge.fury.io/py/wandelbots-nova)
[![License](https://img.shields.io/github/license/wandelbotsgmbh/wandelbots-nova.svg)](https://github.com/wandelbotsgmbh/wandelbots-nova/blob/main/LICENSE)
[![Build status](https://github.com/wandelbotsgmbh/wandelbots-nova/actions/workflows/nova-release.yaml/badge.svg)](https://github.com/wandelbotsgmbh/wandelbots-nova/actions/workflows/nova-release.yaml)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/wandelbotsgmbh/wandelbots-nova)

This library provides an SDK for the Wandelbots NOVA API.

The SDK will help you to build your own apps and services using Python on top of Wandelbots NOVA and makes programming a robot as easy as possible.

[417768496-f6157e4b-eea8-4b96-b302-1f3864ae44a9.webm](https://github.com/user-attachments/assets/ca7de6ba-c78d-414f-ae8f-f76d0890caf3)

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Quickstart](#quickstart)
- [Installation](#installation)
  - [Install with pip](#install-with-pip)
  - [Install with uv and rerun visualization](#install-with-uv-and-rerun-visualization)
  - [Configure environment variables](#configure-environment-variables)
- [Using the SDK](#using-the-sdk)
  - [API essentials](#api-essentials)
  - [Example gallery](#example-gallery)
- [Wandelscript](#wandelscript)
- [NOVAx](#novax)
- [Development](#development)
- [Release process](#release-process)
- [Additional resources](#additional-resources)

## Overview

[Wandelbots NOVA OS](https://www.wandelbots.com/) is a robot-agnostic operating system that enables developers to plan, program, control, and operate fleets of six-axis industrial robots through a unified API, across all major robot brands. It integrates modern development tools like Python and JavaScript APIs with AI-based control and motion planning, allowing developers to build automation tasks such as gluing, grinding, welding, and palletizing without needing to account for hardware differences. The software offers a powerful set of tools that support the creation of custom automation solutions throughout the entire automation lifecycle.

## Prerequisites

- A running NOVA instance (Get a Wandelbots NOVA account on [wandelbots.com](https://www.wandelbots.com/contact))
- Valid NOVA API credentials
- Python >=3.11

## Quickstart

1. Install the SDK using `pip` or set up a local `uv` project with extras for visualization. Refer to the [Installation](#installation) section for both options.
2. Copy `.env.template` to `.env` and fill in the base URL and access token for your NOVA deployment. Details are covered in [Configure environment variables](#configure-environment-variables).
3. Run an example to validate the setup, e.g. `uv run python examples/start_here.py`. Install the rerun extras and execute `uv run download-models` if you want interactive 3D visualization out of the box.

## Installation

### Install with pip

Install the library using pip:

```bash
pip install wandelbots-nova
```

### Install with uv and rerun visualization

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) on your system.

Initialize a new uv project with the following command.

```bash
uv init
```

Install the library with the `nova-rerun-bridge` extra to use the visualization tool [rerun](https://rerun.io/).
See [extension README.md](nova_rerun_bridge/README.md) for further details.

```bash
uv add wandelbots-nova --extra nova-rerun-bridge
```

Download the robot models to visualize them in the rerun viewer.

```bash
uv run download-models
```

### Configure Environment Variables

Copy the provided `.env.template` file and rename it to `.env`:

```bash
cp .env.template .env
```

Open the `.env` file in a text editor and fill in the values. Here's what each variable does:

| Variable            | Description                                                                      | Required | Default | Example                                          |
| ------------------- | -------------------------------------------------------------------------------- | -------- | ------- | ------------------------------------------------ |
| `NOVA_API`          | Base URL or hostname of the Wandelbots NOVA server instance                      | Yes      | None    | `https://nova.example.com` or `http://172.0.0.1` |
| `NOVA_ACCESS_TOKEN` | Pre-obtained access token for Wandelbots NOVA (cloud or self-hosted deployments) | Yes\*    | None    | `eyJhbGciOi...`                                  |

> **Note:**
>
> - `NOVA_API` is mandatory in every deployment. Always point it to the NOVA base URL you are targeting.
> - `NOVA_ACCESS_TOKEN` is the supported authentication mechanism. It is mandatory for the Wandelbots Cloud environment; for self-hosted deployments generate and supply a token with the required permissions.
> - Username/password authentication (`NOVA_USERNAME`/`NOVA_PASSWORD`) is deprecated and no longer supported.

## Using the SDK

### API essentials

Import the library in your code to get started.

```python
from nova import Nova
```

You can access the automatically generated NOVA API client using the `api` module.

```python
from nova import api
```

### Example gallery

Curated examples in this repository showcase typical SDK workflows:

1. **Basic usage**: [start_here.py](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/start_here.py)
2. **Robot movement and I/O control**: [plan_and_execute.py](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/plan_and_execute.py)
3. **Collision-free movement**: [collision_setup.py](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/collision_setup.py)

<img width="100%" alt="collision_free" src="https://github.com/user-attachments/assets/0416151f-1304-46e2-a4ab-485fcda766fc" />

4. **Multiple robot coordination**: [move_multiple_robots.py](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/move_multiple_robots.py)
5. **3D visualization with rerun**: [welding.py](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/welding.py)

> **Note**: Install [rerun extras](#install-with-uv-and-rerun-visualization) to enable visualization

<img width="1242" alt="pointcloud" src="https://github.com/user-attachments/assets/8e981f09-81ae-4e71-9851-42611f6b1843" />

6. **Custom TCPs (Tool Center Points)**: [visualize_tool.py](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/visualize_tool.py)

<img width="100%" alt="trajectory" src="https://github.com/user-attachments/assets/649de0b7-d90a-4095-ad51-d38d3ac2e716" />

7. **Custom mounting with multiple robots**: [robocore.py](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/robocore.py)

<img width="100%" alt="thumbnail" src="https://github.com/user-attachments/assets/6f0c441e-b133-4a3a-bf0e-0e947d3efad4" />

## Wandelscript

Wandelscript is a domain-specific language for programming robots.
It is a declarative language that allows you to describe the robot's behavior in a high-level way.
Wandelscript is suited to get yourself familiar with robot programming.

```bash
uv add wandelbots-nova --extra wandelscript
```

Here is a simple example of a Wandelscript program:

```python
robot = get_controller("controller")[0]
tcp("Flange")
home = read(robot, "pose")
sync

# Set the velocity of the robot to 200 mm/s
velocity(200)

for i = 0..3:
    move via ptp() to home
    # Move to a pose concatenating the home pose
    move via line() to (50, 20, 30, 0, 0, 0) :: home
    move via line() to (100, 20, 30, 0, 0, 0) :: home
    move via line() to (50, 20, 30, 0, 0, 0) :: home
    move via ptp() to home
```

To get started, use the [Quickstart](https://docs.wandelbots.io/latest/pathplanning-maintained/wandelscript/quickstart).
For implementation details or contributing to Wandelscript, refer to the [Wandelscript readme](/wandelscript/README.md).

## NOVAx

NOVAx is an app framework for building server applications on top of Wandelbots NOVA.
It provides common core concepts like the handling of programs and their execution.

You can create a new NOVAx app using the [NOVA CLI](https://github.com/wandelbotsgmbh/nova-cli) generator:

```bash
nova app create "your-nova-app" -g python_app
```

For more information on using NOVAx see the [README](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/your-nova-app/README.md). Explore [this example](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/your-nova-app/your-nova-app/app.py) to use the NOVAx entry point.

> **Important:** When using NOVAx, you must import the actual program functions from their respective Python files. Only importing the program files won't suffice. This ensures proper function registration and execution within the NOVAx runtime environment.

## Development

To install development dependencies, run

```bash
uv sync --extra "nova-rerun-bridge"
```

### Formatting

```bash
uv run ruff format
uv run ruff check --select I --fix
```

### Yaml linting

```bash
docker run --rm -it -v $(pwd):/data cytopia/yamllint -d .yamllint .
```

### Branch versions for testing

When working with feature branches or forks, it can be helpful to test the library as a dependency in other projects before merging.
You can specify custom sources in your pyproject.toml to pull the library from a specific branch:

Using PEP 621-style table syntax:

```toml
wandelbots-nova = { git = "https://github.com/wandelbotsgmbh/wandelbots-nova.git", branch = "fix/http-prefix" }
```

Using PEP 508 direct URL syntax:

```toml
wandelbots-nova @ git+https://github.com/wandelbotsgmbh/wandelbots-nova.git@fix/http-prefix
```

## Release process

### Branch behaviour overview

| Branch      | Purpose                                                | Published to                           | Example version      |
| ----------- | ------------------------------------------------------ | -------------------------------------- | -------------------- |
| `main`      | Stable releases (semantic versioning vX.Y.Z)           | PyPI (`pip install wandelbots-nova`)   | `v1.13.0`            |
| `release/*` | LTS-releases, pre-releases or hotfixes for older lines | PyPI (labeled with release suffix)     | `v1.8.7-release-1.x` |
| any other   | Development builds                                     | GitHub actions (not published to PyPI) | `e4c8af0647839...`   |

### Stable releases from `main`

Merging into main triggers the release workflow:

1. `semantic-release` analyzes commit messages and bumps the version automatically.
2. A source distribution and wheel are built and uploaded to PyPI.
3. A GitHub release is created (or updated) with the release assets.

### LTS releases from `release/\*`

If you're on older major versions or under a special LTS contract:

1. Use (or create) a branch like `release/1.x`, `release/customer-foo`, etc.
2. Every commit to these branches triggers the same workflow as on `main`.
3. Versions include the branch name to prevent collisions, e.g. `v1.8.7-release-1.x`

### Create a dev build (manual)

Need a temporary test build? Use GitHub actions:

1. Go to the [actions tab](https://github.com/wandelbotsgmbh/wandelbots-nova/actions).
2. Find **Nova SDK: Build dev wheel** and click `Run workflow`.
3. Select a branch and trigger the job.
4. After completion, open the [Installation step](#installation) to copy the ready-to-use `pip install` command:

   ```bash
       pip install "wandelbots-nova @ git+https://github.com/wandelbotsgmbh/wandelbots-nova.git@<commit>"
   ```

## Additional resources

- [Examples](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples) covering basic to advanced SDK scenarios
- [Technical wiki](https://deepwiki.com/wandelbotsgmbh/wandelbots-nova) with architecture notes and troubleshooting tips
- [Official documentation](https://docs.wandelbots.io/) for platform concepts and API guides
- [Code documentation](https://wandelbotsgmbh.github.io/wandelbots-nova/) generated from the latest SDK build
