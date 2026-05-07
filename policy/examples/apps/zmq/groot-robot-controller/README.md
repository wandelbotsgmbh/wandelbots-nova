# groot-robot-controller

Nova app that runs dual-arm policy execution against a GR00T ZMQ inference server.

## What it does

1. Moves both UR10e arms to home (500 mm/s)
2. Connects WebRTC cameras
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

## Deploy

```bash
cd policy/examples/apps/zmq/groot-robot-controller
cp -r ../../../../../policy .
nova app install . --omit-credentials
rm -rf policy
```

A k8s service is needed to reach the mock's ZMQ port:

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

## Usage

```bash
# Start with cameras
curl -X POST http://<instance>/cell/groot-robot-controller/start \
  -H 'Content-Type: application/json' \
  -d '{
    "policy_host": "mock-groot-zmq",
    "policy_port": 5555,
    "timeout_s": 20,
    "camera_server": "http://172.31.11.80:9100",
    "cameras": [
      {"name": "flange", "device_id": "315122271048"},
      {"name": "left", "device_id": "314522065367"},
      {"name": "right", "device_id": "319522063360"}
    ]
  }'

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
