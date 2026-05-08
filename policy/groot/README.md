# Gr00tPolicyClient

ZMQ transport for NVIDIA GR00T inference servers.

## Usage

```python
from policy import Gr00tPolicyClient, Observation, PolicyExecutor, PolicySchema, TcpFormat

schema = PolicySchema(observations=[
    Observation.joint_positions("left_arm", source=mg_left),
    Observation.tcp("left_eef_9d", source=mg_left, format=TcpFormat.ROT6D),
    Observation.joint_positions("right_arm", source=mg_right),
    Observation.tcp("right_eef_9d", source=mg_right, format=TcpFormat.ROT6D),
    Observation.io("left_gripper", source=mg_left, io="digital_out[0]",
                   mapping=BoolMapping(on=100.0)),
    Observation.constant("language", value="Pick up the box."),
])

client = Gr00tPolicyClient(host="gpu-server", port=5555, language="Pick up the box.")

executor = PolicyExecutor(schema, client, timeout_s=30.0)
result = await executor.run()
```

The client uses `PolicySchema` observations to build GR00T-compatible numpy array observations and decode the returned action arrays. Joint position keys become `state.<key>`, TCP keys become `state.<key>`, IO keys become `state.<key>`.

## Wire Protocol

Uses the GR00T REQ/REP msgpack protocol:
- **Endpoints**: `ping`, `get_action`, `reset`, `get_modality_config`
- **Observations**: numpy arrays serialized as `.npy` bytes inside msgpack
- **Actions**: returned as `(action_dict, info_dict)` tuple

## Observation Keys for GR00T

The `key` argument in each `Observation.*()` call becomes the GR00T state key:

```python
Observation.joint_positions("left_arm", source=mg)     # → obs["state.left_arm"]
Observation.tcp("left_eef_9d", source=mg,
                format=TcpFormat.ROT6D)                 # → obs["state.left_eef_9d"]
Observation.io("left_gripper", source=mg,
               io="digital_out[0]")                     # → obs["state.left_gripper"]
```

Use `model_dof` on `Gr00tPolicyClient` to pad/truncate joints to match the model's expected DOF.

## Example Apps

See [`examples/apps/zmq/`](../examples/apps/zmq/) for deployable Nova app examples.
