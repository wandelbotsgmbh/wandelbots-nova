# groot-robot-controller

Nova app that runs dual-arm policy execution against a GR00T ZMQ inference server.

Also registers as a Nova program (`groot_policy_controller`) in the program operator with adjustable parameters.

## What it does

1. Moves both UR5e arms to home (500 mm/s)
2. Connects WebRTC cameras (optional)
3. Opens PID jogging sessions on both arms
4. Builds GR00T observations via `PolicySchema`:
   - `state.left_arm` / `state.right_arm` — joint positions (6-DOF)
   - `state.left_eef_9d` / `state.right_eef_9d` — TCP pose (XYZ + rot6d)
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

## Usage

```bash
# Start with external GR00T server
curl -X POST http://<instance>/cell/groot-robot-controller/start \
  -H 'Content-Type: application/json' \
  -d '{"policy_host": "172.31.11.129", "policy_port": 30555, "timeout_s": 20}'

# Start with cameras
curl -X POST http://<instance>/cell/groot-robot-controller/start \
  -H 'Content-Type: application/json' \
  -d '{
    "policy_host": "172.31.11.129",
    "policy_port": 30555,
    "timeout_s": 20,
    "camera_server": "http://192.168.1.8:9100",
    "camera_devices": "exterior_image_1:315122271048,wrist_image:314522065367"
  }'

# Monitor
curl http://<instance>/cell/groot-robot-controller/status

# Stop
curl -X POST http://<instance>/cell/groot-robot-controller/stop
```

## ZMQ networking

The Nova app platform only exposes HTTP (8080→3000). ZMQ port 5555 is **not** automatically reachable between apps. For in-cluster mock GR00T, create a k8s service manually:

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

For production, point at a GR00T server on a GPU machine with a directly reachable IP.
