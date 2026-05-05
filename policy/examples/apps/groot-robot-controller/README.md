# groot-robot-controller

Nova app that runs `PolicyExecutor` against a GR00T-style ZMQ policy service.

This example is wired for a **dual-arm** setup:

- left arm → first motion group
- right arm → second motion group
- observation keys sent to the policy:
  - `left_arm`
  - `right_arm`
  - `left_gripper`
  - `right_gripper`
- action keys expected back from the policy are the same

## Build & Install

This app depends on `policy` from the repo root. Copy it into the build context before installing:

```bash
cd policy/examples/apps/groot-robot-controller
cp -r ../../../../policy .
nova app install . --omit-credentials
rm -rf policy
```

## Run

```bash
curl -X POST http://<IP>/cell/groot-robot-controller/start \
  -H "Content-Type: application/json" \
  -d '{}'
```

Defaults:
- policy host: `app-mock-groot-policy-service`
- policy port: `5555`
- motion groups: `0@ur10e,0@ur10e-2`

## API

| Endpoint | Description |
|---|---|
| `POST /start` | Connect to robots, home them, start executor, connect to GR00T ZMQ service |
| `POST /stop` | Stop executor and close NOVA connection |
| `GET /status` | Current executor status |
