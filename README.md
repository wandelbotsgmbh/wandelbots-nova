# wandelbots-nova (Python SDK)

[![PyPI version](https://badge.fury.io/py/wandelbots-nova.svg)](https://badge.fury.io/py/wandelbots-nova)
[![License](https://img.shields.io/github/license/wandelbotsgmbh/wandelbots-nova.svg)](https://github.com/wandelbotsgmbh/wandelbots-nova/blob/main/LICENSE)
[![Build Status](https://github.com/wandelbotsgmbh/wandelbots-nova/actions/workflows/nova-release.yaml/badge.svg)](https://github.com/wandelbotsgmbh/wandelbots-nova/actions/workflows/nova-release.yaml)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/wandelbotsgmbh/wandelbots-nova)

This library provides an SDK for the Wandelbots NOVA API.

The SDK will help you to build your own apps and services using Python on top of NOVA and makes programming a robot as easy as possible.

https://github.com/user-attachments/assets/f6157e4b-eea8-4b96-b302-1f3864ae44a9

## Background

[Wandelbots NOVA](https://www.wandelbots.com/) is an agnostic robot operating system that enables developers to virtually plan their industrial six-axis robot fleet, as well as to program, control and operate your robots on the shopfloor - all independent on the robot brand and through a unified API. It combines modern development tools (Python, JavaScript APIs) with an AI-driven approach to robot control and motion planning, enabling developers to build applications like gluing, grinding, welding, and palletizing without worrying about underlying hardware differences. The holistic software offers a variety of tools to create unique automation solutions along the whole automation process.

## Prerequisites

- A running Nova instance (apply for access at [wandelbots.com](https://www.wandelbots.com/))
- Valid Nova API credentials
- Python >=3.10

## Installation

Install the library using pip:

```bash
pip install wandelbots-nova
```

### Recommended: uv project and Rerun Visualization

Firstly you need to install [uv](https://docs.astral.sh/uv/getting-started/installation/) to your system.

Initialize a new uv project with the following command.

```bash
uv init
```

We recommend installing the library with the `nova-rerun-bridge` extra to make usage of the visualization tool [rerun](https://rerun.io/).
See the [extension README.md](nova_rerun_bridge/README.md) for further details.

```bash
uv add wandelbots-nova --extra nova-rerun-bridge
```

You need to download the robot models to visualize the robot models in the rerun viewer.

```bash
uv run download-models
```

## Wandelscript

Wandelscript is a domain-specific language for programming robots. It is a declarative language that allows you to describe the robot's behavior in a high-level way.

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

To get more information about Wandelscript, check out the [Wandelscript documentation](/wandelscript/README.md).

## Novax

Novax is an app framework for building server applications on top of NOVA. It provides common core concepts like the handling of programs and their execution.

```bash
uv add wandelbots-nova --extra novax
```

To use Novax in your application, you need to create a new `Novax` instance as an entrypoint.

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
    # You can also register wandelscript files
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

Now you can inspect the API at `http://localhost:8000/docs`.

## 🚀 Quick Start

See the [examples](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples) for usage of this library including 3D visualization.

For more details check out the [technical wiki](https://deepwiki.com/wandelbotsgmbh/wandelbots-nova) (powered by deepwiki), the [official documentation](https://docs.wandelbots.io/) or the [code documentation](https://wandelbotsgmbh.github.io/wandelbots-nova/).

## Usage

Import the library in your code to get started.

```python
from nova import Nova
```

The SDK also includes an auto-generated API client for the NOVA API. You can access the API client using the `api` module.

```python
from nova import api
```

Checkout the [basic](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/basic.py) and [plan_and_execute](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/plan_and_execute.py) examples to learn how to use the library.

In [this](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples) directory are more examples to explain the advanced usage of the SDK.
For 3D visualization examples, see the main [examples](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples) folder. Advanced rerun integration examples are in the [nova_rerun_bridge examples folder](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/nova_rerun_bridge/examples).

## Examples

### Basic Usage

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

### Robot Motion Examples

1. **Simple Point-to-Point Movement**

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

2. **Collision-Free Movement**

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

3. **Multiple Robot Coordination**

```python
import asyncio

async def move_robots():
    async with ur10[0] as ur_mg, kuka[0] as kuka_mg:
        await asyncio.gather(
            move_robot(ur_mg, "Flange"),
            move_robot(kuka_mg, "Flange")
        )
```

See the [move_multiple_robots](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/move_multiple_robots.py) example for more details.

### Advanced Features

1. **I/O Control**

```python
from nova.actions import io_write, joint_ptp, cartesian_ptp

actions = [
    joint_ptp(home_joints),
    io_write(key="digital_out[0]", value=False),  # Set digital output
    cartesian_ptp(target_pose),
    joint_ptp(home_joints)
]
```

2. **3D Visualization with Rerun**

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

> **Note**: For visualization features, install the rerun extras:
>
> ```bash
> uv add wandelbots-nova --extra nova-rerun-bridge
> ```

<img width="1242" alt="pointcloud" src="https://github.com/user-attachments/assets/8e981f09-81ae-4e71-9851-42611f6b1843" />

3. **Adding and Using Custom TCP (Tool Center Point)**

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

4. **Using Common Coordinate Systems for Multiple Robots**

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

To install the development dependencies, run the following command

```bash
uv sync --extra "nova-rerun-bridge"
```

### Formatting

```bash
uv run ruff format
uv run ruff check --select I --fix
```

### Yaml Linting

```bash
docker run --rm -it -v $(pwd):/data cytopia/yamllint -d .yamllint .
```

### Using Branch Versions For Testing

When having feature branches or forks, or might be helpful to test the library as dependency in other projects first.
The pyproject.toml allows to pull the library from different sources.

```toml
wandelbots-nova = { git = "https://github.com/wandelbotsgmbh/wandelbots-nova.git", branch = "fix/http-prefix" }
```

```toml
wandelbots-nova @ git+https://github.com/wandelbotsgmbh/wandelbots-nova.git@fix/http-prefix
```

### Environment Variables for NOVA Configuration

1. **Copy the Template:** Make a copy of the provided `.env.template` file and rename it to `.env` with `cp .env.template .env`.
2. **Fill in the Values:** Open the `.env` file in a text editor and provide the necessary values for each variable. The table below describes each variable and its usage.

| Variable            | Description                                                               | Required | Default | Example                                          |
| ------------------- | ------------------------------------------------------------------------- | -------- | ------- | ------------------------------------------------ |
| `NOVA_API`          | The base URL or hostname of the NOVA server instance.                     | Yes      | None    | `https://nova.example.com` or `http://172.0.0.1` |
| `NOVA_USERNAME`     | The username credential used for authentication with the NOVA service.    | Yes\*    | None    | `my_username`                                    |
| `NOVA_PASSWORD`     | The password credential used in conjunction with `NOVA_USERNAME`.         | Yes\*    | None    | `my_password`                                    |
| `NOVA_ACCESS_TOKEN` | A pre-obtained access token for NOVA if using token-based authentication. | Yes\*    | None    | `eyJhbGciOi...`                                  |

> **Note on Authentication:**
> You can authenticate with NOVA using either **username/password** credentials or a pre-obtained **access token**, depending on your setup and security model:
>
> - If using **username/password**: Ensure both `NOVA_USERNAME` and `NOVA_PASSWORD` are set, and leave `NOVA_ACCESS_TOKEN` unset.
> - If using an **access token**: Ensure `NOVA_ACCESS_TOKEN` is set, and leave `NOVA_USERNAME` and `NOVA_PASSWORD` unset.
>
> **Only one method should be used at a time.** If both methods are set, the token-based authentication (`NOVA_ACCESS_TOKEN`) will typically take precedence.

## Release Process

### Overview

| Variable    | Description                                                            | Where                                | Example version      |
| ----------- | ---------------------------------------------------------------------- | ------------------------------------ | -------------------- |
| `main`      | Stable releases (normal semver vX.Y.Z)                                 | PyPI (`pip install wandelbots-nova`) | `v1.13.0`            |
| `release/*` | The username credential used for authentication with the NOVA service. | PyPI                                 | `v1.8.7-release-1.x` |
| any branch  | Development builds (not published to PyPI)                             | GitHub Actions                       | `e4c8af0647839...`   |

### Stable releases (main)

Every merge to main triggers the Release package workflow: 1. Semantic-release inspects the commit messages, bumps the version, builds the wheel/sdist. 2. The package is uploaded to PyPI. 3. A GitHub Release is created/updated with the assets.

### Long-term-support lines (release/\*)

For customers stuck on an older major or for special LTS contracts:

- Open (or keep) a branch named `release/1.x`, `release/customer-foo`, etc.
- Every commit on that branch triggers the same workflow and publishes stable numbers, but the git tag and PyPI version carry the branch slug so lines never collide.

### Create a dev build

If you only need a throw-away test build, go to the
[Actions](https://github.com/wandelbotsgmbh/wandelbots-nova/actions) tab → "**Nova SDK: Build dev wheel**" → Run workflow (pick the branch).
When it finishes, open the Install instructions job for a ready-to-copy `pip install "wandelbots-nova @ git+https://github.com/wandelbotsgmbh/wandelbots-nova.git@<commit>"` line.
