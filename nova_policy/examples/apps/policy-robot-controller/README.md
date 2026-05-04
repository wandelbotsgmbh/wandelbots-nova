# policy-robot-controller

Prepares robots for policy execution and runs PID jogging when the policy starts.

## API

| Endpoint | Description |
|---|---|
| `POST /prepare` | Connect to robots, move to home, open jogging, connect to policy WS |
| `POST /stop` | Cancel execution, return to home |
| `GET /status` | Current phase (IDLE/PREPARING/READY/EXECUTING/STOPPING), step count |

## Build & Install

This app depends on `nova_policy` from the repo root. Copy it into the build context before installing:

```bash
cd nova_policy/examples/apps/policy-robot-controller
cp -r ../../../../nova_policy .
nova app install . --omit-credentials
rm -rf nova_policy  # cleanup
```

## Parameters (POST /prepare)

| Parameter | Default | Description |
|---|---|---|
| `policy_url` | `ws://app-mock-policy-service:8080/.../predict` | Policy WebSocket |
| `motion_groups` | `0@ur10e,0@ur10e-2` | Motion group IDs |
| `home_joints` | `0,-1.571,...;0,-1.571,...` | Home positions per group |
| `duration_s` | `120` | Max timeout |

## Local Development

For local development, install `nova_policy` as editable:

```bash
cd nova_policy/examples/apps/policy-robot-controller
uv pip install -e ../../../../nova_policy
uv run python -m policy_robot_controller
```
