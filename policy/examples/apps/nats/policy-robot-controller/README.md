# Policy Robot Controller (NATS)

Nova app that manages robot lifecycle for policy execution via PID jogging. Queries a policy service over NATS request/reply.

## What it does

1. Connects to one or more motion groups
2. Moves all robots to home positions
3. Runs a policy episode via PID jogging (queries policy over NATS)
4. Detects e-stop, self-collision, joint limits

## Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/status` | Current phase (IDLE / HOMING / EXECUTING) |
| POST | `/start` | Start execution with given config |
| POST | `/stop` | Stop execution, return to IDLE |

## Start Request

```json
{
  "nats_subject": "nova.v2.cells.cell.apps.mock-policy-service.predict",
  "motion_groups": "0@ur10e,0@ur10e-2",
  "home_joints": "0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0",
  "timeout_s": 30,
  "camera_server": "",
  "cameras": []
}
```

## Deploy

```bash
cd policy/examples/apps/nats/policy-robot-controller
cp -r ../../../../../policy .
nova app install . --omit-credentials
rm -rf policy
```

## Usage

```bash
# Start (robots home first, then run policy for 30s)
curl -X POST http://<instance>/cell/policy-robot-controller/start \
  -H "Content-Type: application/json" \
  -d '{"timeout_s": 30}'

# Monitor
curl http://<instance>/cell/policy-robot-controller/status

# Stop early
curl -X POST http://<instance>/cell/policy-robot-controller/stop
```
