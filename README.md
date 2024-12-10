# wandelbots-nova

This library provides an SDK for the Wandelbots NOVA API.

The SDK will help you to build your own apps and services on top of NOVA and makes programming a robot as easy as possible.

## Requirements

This library requires
* Python >=3.10

## Installation

To use the library, first install it using the following command

```bash
pip install wandelbots-nova
```

Then import the library in your code

```python
from nova import Nova, MotionGroup
```

## Usage

Here is an example of how to use the library to connect to a robot controller and move the robot.

```python
from nova import Nova, pi, jnt, ptp
import asyncio


async def main():
    nova = Nova()
    cell = nova.cell()
    controller = await cell.controller("ur")

    # Define a home position
    home_joints = (0, -pi / 4, -pi / 4, -pi / 4, pi / 4, 0)

    # Connect to the controller and activate motion groups
    async with controller:
        motion_group = controller.get_motion_group()

        # Get current TCP pose and offset it slightly along the x-axis
        current_pose = await motion_group.tcp_pose("Flange")
        target_pose = current_pose @ Pose((100, 50, 0, 0, 0, 0))

        actions = [
            jnt(home_joints),
            ptp(target_pose),
            ptp(target_pose @ (200, 0, 0, 0, 0, 0)),
            jnt(home_joints),
        ]

        await motion_group.run(actions, tcp="Flange")


if __name__ == "__main__":
    asyncio.run(main())
```

Have a look at the [examples](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples) directory to see how to use the library.

## Development

To install the development dependencies, run the following command

```bash
poetry install
```

### Environment Variables for NOVA Configuration

1. **Copy the Template:** Make a copy of the provided `.env.template` file and rename it to `.env` with `cp .env.template .env`.
2. **Fill in the Values:** Open the `.env` file in a text editor and provide the necessary values for each variable. The table below describes each variable and its usage.

| Variable            | Description                                                               | Required | Default | Example                    |
|---------------------|---------------------------------------------------------------------------|----------|---------|----------------------------|
| `NOVA_HOST`         | The base URL or hostname of the NOVA server instance.                     | Yes      | None    | `https://nova.example.com` |
| `NOVA_USERNAME`     | The username credential used for authentication with the NOVA service.    | Yes*     | None    | `my_username`              |
| `NOVA_PASSWORD`     | The password credential used in conjunction with `NOVA_USERNAME`.         | Yes*     | None    | `my_password`              |
| `NOVA_ACCESS_TOKEN` | A pre-obtained access token for NOVA if using token-based authentication. | Yes*     | None    | `eyJhbGciOi...`            |

> **Note on Authentication:**  
> You can authenticate with NOVA using either **username/password** credentials or a pre-obtained **access token**, depending on your setup and security model:
> - If using **username/password**: Ensure both `NOVA_USERNAME` and `NOVA_PASSWORD` are set, and leave `NOVA_ACCESS_TOKEN` unset.
> - If using an **access token**: Ensure `NOVA_ACCESS_TOKEN` is set, and leave `NOVA_USERNAME` and `NOVA_PASSWORD` unset.
>  
> **Only one method should be used at a time.** If both methods are set, the token-based authentication (`NOVA_ACCESS_TOKEN`) will typically take precedence.



