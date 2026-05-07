# groot-robot-controller

Nova app that runs dual-arm policy execution against a GR00T ZMQ inference server.

Also registers as a Nova program (`groot_policy_controller`) in the program operator with adjustable parameters.

## What it does

1. Moves both UR10e arms to home (500 mm/s)
2. Connects WebRTC cameras (optional)
3. Opens PID jogging sessions on both arms
4. Builds GR00T observations using the `FeatureMap`:
   - `state.left_arm` / `state.right_arm` — joint positions (6-DOF)
   - `state.left_eef_9d` / `state.right_eef_9d` — TCP pose (XYZ meters + rot6d)
   - `state.left_gripper` / `state.right_gripper` — gripper IO
   - `video.*` — camera frames
   - `language.annotation.language.language_instruction`
5. Sends observation via ZMQ, receives absolute joint targets
6. Applies targets via PID velocity control

## Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/status` | Current phase and step count |
| POST | `/start` | Move home, connect cameras, run policy |
| POST | `/stop` | Stop execution |

The program operator also exposes `groot_policy_controller` with these adjustable parameters:
- `policy_host` — GR00T ZMQ server hostname
- `policy_port` — GR00T ZMQ server port
- `language` — language instruction
- `timeout_s` — execution timeout
- `motion_groups` — comma-separated motion group IDs
- `home_joints` — semicolon-separated home positions

## Deploy

```bash
cd policy/examples/apps/zmq/groot-robot-controller
cp -r ../../../../../policy .
nova app install . --omit-credentials
rm -rf policy
```

## Connecting to a GR00T server

### Option A: External GR00T server (recommended for production)

Point at a GR00T server running on a GPU machine with a directly reachable IP:

```bash
curl -X POST http://<instance>/cell/groot-robot-controller/start \
  -H 'Content-Type: application/json' \
  -d '{"policy_host": "172.31.11.129", "policy_port": 30555, "timeout_s": 10}'
```

This is the typical production setup — the GR00T inference server runs on a GPU machine accessible on the network.

### Option B: In-cluster mock GR00T service

The Nova app platform only exposes one port per app (HTTP 8080→3000). ZMQ port 5555 is **not** automatically reachable between apps. To reach the `mock-groot-policy-service` app's ZMQ port from within the cluster, you must create a k8s service manually:

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

Then start with the k8s service name:

```bash
curl -X POST http://<instance>/cell/groot-robot-controller/start \
  -H 'Content-Type: application/json' \
  -d '{"policy_host": "mock-groot-zmq", "policy_port": 5555, "timeout_s": 10}'
```

> **Note:** This requires `kubectl` access to the Nova instance's k8s cluster. On managed/cloud Nova instances you typically don't have this — use Option A instead.

## Usage

```bash
# Start with cameras + external GR00T server
curl -X POST http://<instance>/cell/groot-robot-controller/start \
  -H 'Content-Type: application/json' \
  -d '{
    "policy_host": "172.31.11.129",
    "policy_port": 30555,
    "timeout_s": 20,
    "language": "Pick up the red block.",
    "camera_server": "http://192.168.1.8:9100",
    "cameras": [
      {"name": "exterior_image_1_left", "device_id": "315122271048", "width": 224, "height": 224, "fps": 15},
      {"name": "wrist_image_left", "device_id": "314522065367", "width": 224, "height": 224, "fps": 15},
      {"name": "exterior_image_2_left", "device_id": "319522063360", "width": 224, "height": 224, "fps": 15}
    ]
  }'

# Start without cameras (will fail on real GR00T server that requires video)
curl -X POST http://<instance>/cell/groot-robot-controller/start \
  -H 'Content-Type: application/json' \
  -d '{"policy_host": "172.31.11.129", "policy_port": 30555, "timeout_s": 10}'

# Monitor
curl http://<instance>/cell/groot-robot-controller/status

# Stop
curl -X POST http://<instance>/cell/groot-robot-controller/stop
```

## Start request parameters

| Field | Default | Description |
|-------|---------|-------------|
| `policy_host` | `app-mock-groot-policy-service` | ZMQ server hostname |
| `policy_port` | `5555` | ZMQ port |
| `motion_groups` | `0@ur10e,0@ur10e-2` | Comma-separated motion group IDs |
| `home_joints` | `0,-1.571,1.571,-1.571,-1.571,0;...` | Semicolon-separated home positions |
| `gripper_ios` | `digital_out[0],digital_out[0]` | Gripper IO key per arm |
| `language` | `Coordinate both arms and move smoothly.` | Language instruction |
| `timeout_s` | `10.0` | Execution duration (seconds) |
| `camera_server` | (empty) | WebRTC camera server URL |
| `cameras` | `[]` | Camera configs (name, device_id, width, height, fps) |
