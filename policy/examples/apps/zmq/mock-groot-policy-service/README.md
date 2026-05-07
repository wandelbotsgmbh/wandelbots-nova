# mock-groot-policy-service

Stateless mock of a GR00T inference server for a dual-arm UR10e embodiment.

Drop-in replacement for a real NVIDIA GR00T server — same ZMQ protocol, same observation/action format.

## ZMQ Endpoints

| Endpoint | Description |
|----------|-------------|
| `ping` | Health check |
| `get_action` | Observation → action chunk |
| `reset` | No-op (returns `{}`) |
| `get_modality_config` | Embodiment metadata (with `__ModalityConfig_class__` envelope) |
| `kill` | Shutdown |

## HTTP Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/health` | HTTP health check |
| GET | `/status` | Request count, last keys received |

## Observation format

```python
{
    "state": {
        "left_arm":      np.ndarray(1, 1, 6),   # float32, joint positions (rad)
        "right_arm":     np.ndarray(1, 1, 6),   # float32
        "left_eef_9d":   np.ndarray(1, 1, 9),   # float32, XYZ (m) + rot6d
        "right_eef_9d":  np.ndarray(1, 1, 9),   # float32
        "left_gripper":  np.ndarray(1, 1, 1),   # float32, 0=open 100=closed
        "right_gripper": np.ndarray(1, 1, 1),   # float32
    },
    "video": {
        "flange": np.ndarray(1, 1, H, W, 3),   # uint8 RGB (at least one required)
    },
    "language": {
        "annotation.language.language_instruction": [["Pick up the object."]],
    },
}
```

## Action format

```python
(
    {
        "left_arm":      np.ndarray(1, 16, 6),  # absolute joint targets
        "right_arm":     np.ndarray(1, 16, 6),
        "left_gripper":  np.ndarray(1, 16, 1),  # 0-100 gripper position
        "right_gripper": np.ndarray(1, 16, 1),
    },
    {},  # info dict (empty)
)
```

## Transport

- **Protocol:** ZMQ REQ/REP
- **Serialization:** msgpack with `__ndarray_class__` numpy transport
- **Port:** 5555 (container-internal)

## Deploy

```bash
nova app install policy/examples/apps/zmq/mock-groot-policy-service --omit-credentials
```

Expose ZMQ port for inter-app access:

```bash
kubectl apply -n cell -f - <<EOF
apiVersion: v1
kind: Service
metadata:
  name: mock-groot-zmq
spec:
  selector:
    app: app-mock-groot-policy-service
  ports:
    - name: zmq
      port: 5555
      targetPort: 5555
EOF
```

Other apps connect to `mock-groot-zmq:5555`.

## Inference behavior

Time-based oscillation (not observation-dependent) that produces visible motion:
- Arms: sinusoidal joint offsets, per-joint amplitude scaling (base still, wrist moves most)
- Grippers: slow toggle at ~0.25 Hz
