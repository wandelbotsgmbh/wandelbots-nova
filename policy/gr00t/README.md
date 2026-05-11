# Gr00tPolicyClient

ZMQ transport for [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) inference servers.

Implements the same REQ/REP msgpack protocol as `gr00t.policy.server_client.PolicyServer`, so `Gr00tPolicyClient` is a drop-in replacement for `gr00t.policy.server_client.PolicyClient` — but integrated with the NOVA `PolicyExecutor` lifecycle.

## Usage

```python
from policy import (
    BoolMapping, Gr00tPolicyClient, Observation, PolicyExecutor, PolicySchema, TcpFormat,
)

schema = PolicySchema(observations=[
    Observation.joint_positions("left_arm", source=mg_left),
    Observation.tcp("left_eef_9d", source=mg_left, format=TcpFormat.ROT6D),
    Observation.joint_positions("right_arm", source=mg_right),
    Observation.tcp("right_eef_9d", source=mg_right, format=TcpFormat.ROT6D),
    Observation.io("left_gripper", source=mg_left, io="digital_out[0]",
                   mapping=BoolMapping(on=100.0)),
    Observation.constant("language", value="Pick up the box."),
])

client = Gr00tPolicyClient(host="gpu-server", port=5555)

executor = PolicyExecutor(schema, client, timeout_s=30.0)
result = await executor.run()
```

The client uses `PolicySchema` observations to build GR00T-compatible numpy array observations and decode the returned action arrays.

## Wire Protocol

Uses the [GR00T REQ/REP msgpack protocol](https://github.com/NVIDIA/Isaac-GR00T/blob/main/gr00t/policy/server_client.py):

- **Endpoints**: `ping`, `get_action`, `reset`, `get_modality_config`
- **Observations**: numpy arrays serialized as `.npy` bytes inside msgpack
- **Actions**: returned as `(action_dict, info_dict)` tuple

## Observation Keys

The `key` argument in each `Observation.*()` call becomes the GR00T state key:

```python
Observation.joint_positions("left_arm", source=mg)     # → obs["state.left_arm"]
Observation.tcp("left_eef_9d", source=mg,
                format=TcpFormat.ROT6D)                 # → obs["state.left_eef_9d"]
Observation.io("left_gripper", source=mg,
               io="digital_out[0]")                     # → obs["state.left_gripper"]
```

## Example Apps

See [`examples/apps/gr00t/`](../examples/apps/gr00t/) for deployable Nova app examples.
