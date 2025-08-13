# Nova Rerun Bridge

A visualization extension for [wandelbots-nova](https://github.com/wandelbotsgmbh/wandelbots-nova) that enables real-time 3D visualization of robot trajectories using [rerun.io](https://rerun.io).

[402951223-ab527bc4-720a-41f2-9499-54d6ed027163.webm](https://github.com/user-attachments/assets/b75f54d5-ce39-42ad-96b5-2fdefc780fa1)

## Prerequisites

- A running Nova instance (get access at [wandelbots.com](https://www.wandelbots.com/))
- [wandelbots-nova](https://pypi.org/project/wandelbots-nova/) Python package
- Valid Nova API credentials

## 🚀 Quick Start

Check out the [minimal example](https://github.com/wandelbotsgmbh/nova-rerun-bridge/tree/main/minimal_example):

```bash
# Add the package to your pyproject.toml
wandelbots-nova = { version = ">=0.12", extras = ["nova-rerun-bridge"] }
```

```bash
# Download required robot models
uv run download-models
```

```python
# Add credentials and instance to .env file
NOVA_API="https://your-instance.wandelbots.io"
NOVA_ACCESS_TOKEN="your-access-token"
```

```python
from nova_rerun_bridge import NovaRerunBridge
from nova import Nova

# Connect to your Nova instance (or use .env file)
nova = Nova(
    host="https://your-instance.wandelbots.io",
    access_token="your-access-token"
)
bridge = NovaRerunBridge(nova)

# Setup visualization
await bridge.setup_blueprint()

# Log a trajectory
await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
```

# ✨ Features

- 🤖 Real-time 3D robot visualization (see a [list](https://wandelbotsgmbh.github.io/wandelbots-js-react-components/?path=/story/3d-view-robot-supported-models) of supported robots)
- 🎯 Trajectory playback and analysis
- 💥 Collision scene visualization
- ⏱️ Motion timing analysis
- 🔄 Continuous monitoring mode

## 💻 Usage Examples

The python library can be used to feed data to the rerun desktop app. The library is built on top of the nova python library and provides a simple interface to feed data to the rerun desktop app. See the [minimal example](https://github.com/wandelbotsgmbh/nova-rerun-bridge/tree/main/minimal_example) on how to use the library.

### Basic Motion Logging

```python
# Log simple motion
await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
```

### Collision Scene Visualization

```python
# Log collision scenes
await bridge.log_collision_scenes()
```

### Stream current robot state

```python
# Start streaming robot state
await bridge.start_streaming(motion_group)

# Stop streaming all robot states
await bridge.stop_streaming()
```

### Log Actions

```python
# Log planned actions
await bridge.log_actions(actions)
```

## Setup

Adjust the `NOVA_API` and `NOVA_ACCESS_TOKEN` in the `.env` file to your instance URL (e.g. `https://unzhoume.instance.wandelbots.io`) and access token. You can find the access token in the developer portal.

## 📚 More Examples

Check out our [example repository](https://github.com/wandelbotsgmbh/nova-rerun-bridge/tree/main/nova_rerun_bridge/examples) for more detailed examples.

## ⚙️ Configuration

The bridge can be configured through environment variables:

- RECORDING_INTERVAL: Set visualization update interval (default: 0.1s)

## Download Robot Models

After installing the library, you need to download the robot models:

```bash
# If installed via uv
uv run download-models

# If installed via pip
python -m nova_rerun_bridge.models.download_models
```

This will download the robot models into your project folder. You can use the library without downloading the models, but you will not be able to visualize the robot models in the rerun viewer.

### Tools

Code formatting and linting is done with [ruff]

```bash
uv run ruff check scripts/. --fix
uv run ruff format
```

### Build

To build the package locally, run the following command

```bash
uv build
```

This will create a dist/ directory with the built package (.tar.gz and .whl files).

#### Install a Development Branch

```
nova-rerun-bridge = { git = "https://github.com/wandelbotsgmbh/nova-rerun-bridge.git", branch = "feature/branchname" }
```

# Run as Nova App

The easiest way to try it out is to install the app on your nova instance. Use the nova cli tool and run:

```bash
nova catalog install rerun
nova catalog install nova-rerun-bridge
```

The nova rerun bridge will be installed on your nova instance and automatically collect all planned motions (see `nova_rerun_bridge/polling/populate.py`). If you click on the rerun bridge app, it will start a download of a nova.rrd file. You can open this file with the rerun desktop app or the installed rerun app on the ipc and see the visualization.

## Development

### Deploy on local instance

- use the kubeconfig from your nova instance and run `export KUBECONFIG=kubeconfig`

- you can use [skaffold](https://skaffold.dev/) to build the image and change the deployment

```bash
skaffold dev --cleanup=false --status-check=false
```

## 📝 License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.
