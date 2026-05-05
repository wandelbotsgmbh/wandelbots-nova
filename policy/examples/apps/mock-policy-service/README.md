# Mock Policy Service

Stateless NATS policy mock for `PolicyExecutor`.

## What it does

- Subscribes to a NATS subject (default `nova.v2.cells.cell.apps.mock-policy-service.predict`)
- Receives observation JSON via NATS request/reply
- Returns deterministic sinusoidal joint targets as `PolicyResponse` JSON
- Stateless: equal observations produce equal actions
- Configurable via `POST /configure`

## Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/health` | Health check |
| GET | `/status` | Shows NATS connection state, request count, config |
| POST | `/configure` | Update inference params (amplitude, chunk_size, etc.) |

## NATS Protocol

The service subscribes to `NATS_SUBJECT` (default `nova.v2.cells.cell.apps.mock-policy-service.predict`) and responds via request/reply:

```
→ Request (observation):
  {"joints": [0.1, -1.5, ...], "motion_group_id": "0@ur10e"}

← Reply (PolicyResponse):
  {"joints": {"0@ur10e": [[step0], [step1], ..., [step15]]}, "dt_ms": 33.0}
```

## Configuration

Environment variables:
- `NATS_BROKER` — NATS server URL (injected automatically by Nova platform)
- `NATS_SUBJECT` — Subject to subscribe to (default: `nova.v2.cells.cell.apps.mock-policy-service.predict`)
- `BASE_PATH` — URL prefix (injected by Nova platform)

## Deploy

```bash
nova app install policy/examples/apps/mock-policy-service --omit-credentials
```
