# mock-groot-policy-service

Stateless mock of a GR00T inference server for a **dual-arm** policy.

It exposes the same core ZMQ endpoints as GR00T:

- `ping`
- `get_action`
- `reset`
- `get_modality_config`
- `kill`

The app also exposes HTTP endpoints for observability:

- `GET /health`
- `GET /status`

## Key behavior

- **Stateless inference**: predictions are a pure function of the current observation
- **GR00T-style transport**: ZMQ `REQ/REP` + `msgpack`
- **Dual-arm contract**: expects state keys `left_arm`, `right_arm`, `left_gripper`, `right_gripper`
- **Language contract**: expects `language.task`
- **Multi-step output**: returns action chunks with configurable horizon
- **No episode state**: `reset()` is a no-op and returns `{"stateless": true}`

## Install

```bash
nova app install . --omit-credentials
```

## Nova app networking

From another Nova app in the same cell, connect to:

```text
host=app-mock-groot-policy-service
port=5555
```

## Example with `nova_policy.Gr00tPolicyClient`

```python
from nova_policy import Gr00tPolicyClient

policy = Gr00tPolicyClient(
    host="app-mock-groot-policy-service",
    port=5555,
    decode_action=my_decoder,
)
```

## Action/Observation format

Observation:

```python
{
    "state": {
        "left_arm": np.ndarray[(1, 1, 6)],
        "right_arm": np.ndarray[(1, 1, 6)],
        "left_gripper": np.ndarray[(1, 1, 1)],
        "right_gripper": np.ndarray[(1, 1, 1)],
    },
    "language": {
        "task": [["Coordinate both arms and move smoothly."]],
    },
}
```

Response:

```python
(
    {
        "left_arm": np.ndarray[(1, T, 6)],
        "right_arm": np.ndarray[(1, T, 6)],
        "left_gripper": np.ndarray[(1, T, 1)],
        "right_gripper": np.ndarray[(1, T, 1)],
    },
    {"stateless": True, "dt_ms": 33.0, ...},
)
```

## Status endpoint

```bash
curl http://<IP>/cell/mock-groot-policy-service/status
```
