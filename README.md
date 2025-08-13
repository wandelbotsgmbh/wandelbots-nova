# wandelbots-nova (Python SDK)

[![PyPI version](https://badge.fury.io/py/wandelbots-nova.svg)](https://badge.fury.io/py/wandelbots-nova)
[![License](https://img.shields.io/github/license/wandelbotsgmbh/wandelbots-nova.svg)](https://github.com/wandelbotsgmbh/wandelbots-nova/blob/main/LICENSE)
[![Build status](https://github.com/wandelbotsgmbh/wandelbots-nova/actions/workflows/nova-release.yaml/badge.svg)](https://github.com/wandelbotsgmbh/wandelbots-nova/actions/workflows/nova-release.yaml)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/wandelbotsgmbh/wandelbots-nova)

This library provides an SDK for the Wandelbots NOVA API.

The SDK will help you to build your own apps and services using Python on top of Wandelbots NOVA and makes programming a robot as easy as possible.

[417768496-f6157e4b-eea8-4b96-b302-1f3864ae44a9.webm](https://github.com/user-attachments/assets/ca7de6ba-c78d-414f-ae8f-f76d0890caf3)

## About

[Wandelbots NOVA](https://www.wandelbots.com/) is a robot-agnostic operating system that enables developers to plan, program, control, and operate fleets of six-axis industrial robots through a unified API, across all major robot brands. It integrates modern development tools like Python and JavaScript APIs with AI-based control and motion planning, allowing developers to build automation tasks such as gluing, grinding, welding, and palletizing without needing to account for hardware differences. The software offers a powerful set of tools that support the creation of custom automation solutions throughout the entire automation lifecycle.

## Prerequisites

- A running NOVA instance (Get a Wandelbots NOVA account on [wandelbots.com](https://www.wandelbots.com/contact))
- Valid NOVA API credentials
- Python >=3.10

## Installation

Install the library using pip:

```bash
pip install wandelbots-nova
```

### Recommended: uv project and Rerun Visualization

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

### Environment variables for NOVA configuration

Copy the provided `.env.template` file and rename it to `.env`:

```bash
cp .env.template .env
```

Open the `.env` file in a text editor and fill in the values. Here's what each variable does:

| Variable            | Description                                                                         | Required | Default | Example                                          |
| ------------------- | ----------------------------------------------------------------------------------- | -------- | ------- | ------------------------------------------------ |
| `NOVA_API`          | Base URL or hostname of the Wandelbots NOVA server instance                         | Yes      | None    | `https://nova.example.com` or `http://172.0.0.1` |
| `NOVA_USERNAME`     | Username credential used for authentication with the NOVA service                   | Yes\*    | None    | `my_username`                                    |
| `NOVA_PASSWORD`     | Password credential used in conjunction with `NOVA_USERNAME`                        | Yes\*    | None    | `my_password`                                    |
| `NOVA_ACCESS_TOKEN` | Pre-obtained access token for Wandelbots NOVA (if using token-based authentication) | Yes\*    | None    | `eyJhbGciOi...`                                  |

> **Note:**
> You can authenticate with Wandelbots NOVA using either **username/password credentials** or a **pre-obtained access token**, depending on your setup and security model:
>
> - **Username/password authentication**: Ensure both `NOVA_USERNAME` and `NOVA_PASSWORD` are set, and leave `NOVA_ACCESS_TOKEN` unset.
> - **Token-based authentication**: Ensure `NOVA_ACCESS_TOKEN` is set, and leave `NOVA_USERNAME` and `NOVA_PASSWORD` unset.
>
> **Use only one method at a time**. If both are set, token-based authentication takes precedence.

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
implementation details or contributing to Wandelscript, refer to the [Wandelscript readme](/wandelscript/README.md).

## NOVAx

NOVAx is an app framework for building server applications on top of Wandelbots NOVA.
It provides common core concepts like the handling of programs and their execution.

```bash
uv add wandelbots-nova --extra novax
```

To use NOVAx in your application, you need to create a new `Novax` instance as an entrypoint.

```python
from pathlib import Path
import uvicorn
from novax import Novax


@nova.program()
def simple_program():
    print("Hello World!")


def main(host: str = "0.0.0.0", port: int = 8000):
    # Create a new Novax instance
    novax = Novax()
    # Create a new FastAPI app
    app = novax.create_app()
    # Include the programs router
    novax.include_programs_router(app)

    # Register Python programs (existing functionality)
    novax.register_program(simple_program)
    # You can also register Wandelscript files
    novax.register_program(Path(__file__).parent / "programs" / "program2.ws")

    # Serve the FastAPI app
    uvicorn.run(
        app,
        host=host,
        port=port,
        reload=False,
        log_level=log_level,
        proxy_headers=True,
        forwarded_allow_ips="*",
    )
```

Inspect the API at `http://localhost:8000/docs`.

## 🚀 Quickstart

See the [examples](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples) for usage of this library including 3D visualization.

For more details check out the [technical wiki](https://deepwiki.com/wandelbotsgmbh/wandelbots-nova) (powered by deepwiki), the [official documentation](https://docs.wandelbots.io/) or the [code documentation](https://wandelbotsgmbh.github.io/wandelbots-nova/).

## Usage

Import the library in your code to get started.

```python
from nova import Nova
```

You can access the automatically generated NOVA API client using the `api` module.

```python
from nova import api
```

Check out the [basic](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/basic.py) and [plan_and_execute](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/plan_and_execute.py) examples to learn how to use the library.

## Examples

You can find different categories of examples in the repository:

- **[Advanced SDK usage](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples)**
- **[3D visualization](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples)**
- **[Advanced Rerun integration](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/nova_rerun_bridge/examples)**

### Basic usage

```python
import nova
from nova import Nova
from nova.program import ProgramPreconditions
from nova.cell import virtual_controller

@nova.program(
  preconditions=ProgramPreconditions(
    controllers=[
      virtual_controller(
        name="ur10",
        manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
        type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
      ),
      virtual_controller(
        name="kuka",
        manufacturer=api.models.Manufacturer.KUKA,
        type=api.models.VirtualControllerTypes.KUKA_MINUS_KR16_R1610_2,
      ),
    ],
  )
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        ur10 = await cell.controller("ur10")
        kuka = await cell.controller("kuka")

        async with ur10[0] as motion_group:
            tcp = "Flange"
            home_joints = await motion_group.joints()
            current_pose = await motion_group.tcp_pose(tcp)
```

### Robot motion examples

1. **Basic Point-to-Point movement**

```python
import nova
from nova import Nova
from nova.actions import cartesian_ptp, joint_ptp
from nova.types import Pose

@nova.program()
async def main():
    async with Nova() as nova:
        actions = [
            joint_ptp(home_joints),
            cartesian_ptp(current_pose @ Pose((100, 0, 0, 0, 0, 0))),  # Move 100mm in X
            joint_ptp(home_joints)
        ]
        trajectory = await motion_group.plan(actions, tcp)
```

2. **Collision-free movement**

```python
from nova.actions import collision_free
from nova.types import Pose, MotionSettings
from math import pi

actions = [
    collision_free(
        target=Pose((-500, -400, 200, pi, 0, 0)),
        collision_scene=collision_scene,
        settings=MotionSettings(tcp_velocity_limit=30)
    )
]
```

https://github.com/user-attachments/assets/0416151f-1304-46e2-a4ab-485fcda766fc

3. **Multiple robot coordination**

```python
import asyncio

async def move_robots():
    async with ur10[0] as ur_mg, kuka[0] as kuka_mg:
        await asyncio.gather(
            move_robot(ur_mg, "Flange"),
            move_robot(kuka_mg, "Flange")
        )
```

More information in [move_multiple_robots](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/move_multiple_robots.py).

### Advanced features

1. **Input/Output control**

```python
from nova.actions import io_write, joint_ptp, cartesian_ptp

actions = [
    joint_ptp(home_joints),
    io_write(key="digital_out[0]", value=False),  # Set digital output
    cartesian_ptp(target_pose),
    joint_ptp(home_joints)
]
```

2. **3D visualization with rerun**

```python
# Basic 3D visualization (default)
@nova.program(
    viewer=nova.viewers.Rerun()
)
async def main():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("robot1")

        async with controller[0] as motion_group:
            actions = [cartesian_ptp(target_pose)]
            trajectory = await motion_group.plan(actions, tcp)
            # Trajectory is automatically visualized in Rerun
```

```python
# Advanced visualization with detailed panels and tool models
@nova.program(
    viewer=nova.viewers.Rerun(
        show_details=True,  # Show detailed analysis panels
        show_safety_zones=True,  # Show robot safety zones
        show_collision_link_chain=True,  # Show collision geometry
        tcp_tools={
            "vacuum": "assets/vacuum_cup.stl",
            "gripper": "assets/parallel_gripper.stl"
        }
    )
)
```

> **Note**: Install [rerun extras](#recommended-uv-project-and-rerun-visualization) to enable visualization

<img width="1242" alt="pointcloud" src="https://github.com/user-attachments/assets/8e981f09-81ae-4e71-9851-42611f6b1843" />

3. **Custom TCPs (Tool Center Points)**

```python
import json

import nova
from nova import Nova
from nova.api import models
from nova.actions import cartesian_ptp
from nova.types import Pose

# Define TCP configuration
tcp_config = {
    "id": "vacuum_gripper",
    "readable_name": "Vacuum Gripper",
    "position": {"x": 0, "y": 0, "z": 100},  # 100mm in Z direction
    "rotation": {"angles": [0, 0, 0], "type": "EULER_ANGLES_EXTRINSIC_XYZ"}
}

@nova.program(
    name="Add TCP",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="robot1",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def setup_tcp():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("robot1")

        # Add TCP to virtual robot
        tcp_config_obj = models.RobotTcp.from_json(json.dumps(tcp_config))
        await nova._api_client.virtual_robot_setup_api.add_virtual_robot_tcp(
            cell.cell_id,
            controller.controller_id,
            motion_group_idx=0,
            tcp_config_obj
        )

        # Use the new TCP
        async with controller[0] as motion_group:
            current_pose = await motion_group.tcp_pose("vacuum_gripper")
            # Plan motions using the new TCP
            actions = [cartesian_ptp(current_pose @ Pose((100, 0, 0, 0, 0, 0)))]
            trajectory = await motion_group.plan(actions, "vacuum_gripper")
```

<img width="100%" alt="trajectory" src="https://github.com/user-attachments/assets/649de0b7-d90a-4095-ad51-d38d3ac2e716" />

4. **Common coordinate systems for multiple robots**

```python
from math import pi
import asyncio

import nova
from nova.api.models import CoordinateSystem, Vector3d, RotationAngles, RotationAngleTypes
from nova.actions import cartesian_ptp
from nova.types import Pose

@nova.program(
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            ),
            virtual_controller(
                name="kuka",
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_MINUS_KR16_R1610_2,
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def setup_coordinated_robots():
    async with Nova() as nova:
        cell = nova.cell()

        # Setup robots
        robot1 = await cell.controller("ur10")
        robot2 = await cell.controller("kuka")

        # Define common world coordinate system
        world_mounting = CoordinateSystem(
            coordinate_system="world",
            name="mounting",
            reference_uid="",
            position=Vector3d(x=0, y=0, z=0),
            rotation=RotationAngles(
                angles=[0, 0, 0],
                type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
            )
        )

        # Position robots relative to world coordinates
        await nova._api_client.virtual_robot_setup_api.set_virtual_robot_mounting(
            cell="cell",
            controller=robot1.controller_id,
            id=0,  # motion_group_id
            coordinate_system=CoordinateSystem(
                coordinate_system="world",
                name="robot1_mount",
                reference_uid="",
                position=Vector3d(x=500, y=0, z=0),  # Robot 1 at x=500mm
                rotation=RotationAngles(
                    angles=[0, 0, 0],
                    type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                )
            )
        )

        await nova._api_client.virtual_robot_setup_api.set_virtual_robot_mounting(
            cell="cell",
            controller=robot2.controller_id,
            id=0,  # motion_group_id
            coordinate_system=CoordinateSystem(
                coordinate_system="world",
                name="robot2_mount",
                reference_uid="",
                position=Vector3d(x=-500, y=0, z=0),  # Robot 2 at x=-500mm
                rotation=RotationAngles(
                    angles=[0, 0, pi],  # Rotated 180° around Z
                    type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                )
            )
        )

        # Now both robots can work in the same coordinate system
        async with robot1[0] as mg1, robot2[0] as mg2:
            # Movements will be relative to world coordinates
            await asyncio.gather(
                mg1.plan([cartesian_ptp(Pose((0, 100, 0, 0, 0, 0)))], "tcp1"),
                mg2.plan([cartesian_ptp(Pose((0, -100, 0, 0, 0, 0)))], "tcp2")
            )
```

<img width="100%" alt="thumbnail" src="https://github.com/user-attachments/assets/6f0c441e-b133-4a3a-bf0e-0e947d3efad4" />

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
