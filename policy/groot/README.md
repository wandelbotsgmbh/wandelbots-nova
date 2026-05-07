# Gr00tPolicyClient

ZMQ transport for NVIDIA GR00T inference servers.

## Usage

```python
from policy import FeatureMap, FeatureGroup, TcpFormat
from policy.groot import Gr00tPolicyClient

feature_map = FeatureMap(groups=[
    FeatureGroup(
        motion_group=mg,
        name="left",
        joint_key="left_arm",
        tcp_key="left_eef_9d",
        tcp_format=TcpFormat.ROT6D,
        ios={"left_gripper": "digital_out[0]"},
    ),
])

client = Gr00tPolicyClient(host="gpu-server", port=5555)
```

The client uses `FeatureGroup` properties (`joint_key`, `tcp_key`, `tcp_format`, `ios`) to build GR00T-compatible numpy array observations and decode the returned action arrays.

## Wire Protocol

Uses the GR00T REQ/REP msgpack protocol:
- **Endpoints**: `ping`, `get_action`, `reset`, `get_modality_config`
- **Observations**: numpy arrays serialized as `.npy` bytes inside msgpack
- **Actions**: returned as `(action_dict, info_dict)` tuple

## FeatureGroup Keys for GR00T

Override the default key names to match your GR00T embodiment config:

```python
FeatureGroup(
    motion_group=mg,
    name="left",
    joint_key="left_arm",          # → obs["state.left_arm"]
    tcp_key="left_eef_9d",         # → obs["state.left_eef_9d"]
    ios={"left_gripper": "..."},   # → obs["state.left_gripper"]
    tcp_format=TcpFormat.ROT6D,    # position + 6D rotation
    model_dof=7,                   # pad/truncate to match model
)
```

## Example Apps

See [`examples/apps/zmq/`](../examples/apps/zmq/) for deployable Nova app examples.
