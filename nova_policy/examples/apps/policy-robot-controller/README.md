# Policy Robot Controller

Nova app that manages robot lifecycle for policy execution via PID jogging. Uses NATS to communicate with a policy service.

## What it does

- Connects to one or more motion groups on a Nova instance
- Opens PID jogging sessions
- Sends observations to a policy service via NATS request/reply
- Applies returned joint targets through PID velocity control
- Manages multi-episode lifecycle with home-position resets

## Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/status` | Current executor phase (IDLE, RESETTING, READY, EXECUTING) |
| POST | `/start` | Start the executor with given config |
| POST | `/stop` | Stop execution, close connections, return to IDLE |

## Start Request

```json
{
  "nats_subject": "nova.v2.cells.cell.apps.mock-policy-service.predict",
  "motion_groups": "0@ur10e,0@ur10e-2",
  "home_joints": "0,-1.571,1.571,-1.571,-1.571,0;0,-1.571,-1.571,-1.571,1.571,0",
  "timeout_s": 0
}
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `nats_subject` | `nova.v2.cells.cell.apps.mock-policy-service.predict` | NATS subject for policy inference |
| `motion_groups` | `0@ur10e,0@ur10e-2` | Comma-separated motion group IDs |
| `home_joints` | see above | Semicolon-separated home joints per MG |
| `timeout_s` | `0` | Episode timeout (0 = infinite) |

## Environment Variables

- `NATS_BROKER` — NATS server URL (injected automatically by Nova platform)
- `NATS_SUBJECT` — Default NATS subject (overridable per request)
- `BASE_PATH` — URL prefix (injected by Nova platform)

## Deploy

```bash
cd nova_policy/examples/apps/policy-robot-controller
cp -r ../../../../nova_policy .
nova app install . --omit-credentials
rm -rf nova_policy
```

## Usage

```bash
# Start execution
curl -X POST http://<instance>/cell/policy-robot-controller/start

# Check status
curl http://<instance>/cell/policy-robot-controller/status

# Stop
curl -X POST http://<instance>/cell/policy-robot-controller/stop
```
