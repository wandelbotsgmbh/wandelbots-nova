# Mock Policy Service (NATS)

Stateless policy inference service for `PolicyExecutor`. Communicates via NATS request/reply.

## What it does

- Subscribes to a NATS subject (default `nova.policy.predict`)
- Receives observations (joint positions + optional images) via NATS request/reply
- Returns deterministic action chunks as `PolicyResponse` JSON
- Stateless: same observations always produce the same actions
- Also accepts images published on `<subject>.images.<camera_name>`

## Mock policy behavior

Uses a coupled oscillator: `sin(sum_of_all_joints * 3.0 + phase)`. This produces continuous motion that never converges to a fixed point (important for testing) while remaining completely stateless.

## Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/health` | Health check |
| GET | `/status` | NATS state, request count, config |
| POST | `/configure` | Update inference params (amplitude, chunk_size, etc.) |

## NATS Protocol

**Subject:** Configurable via `NATS_SUBJECT` env var (default `nova.policy.predict`)

```
→ Request (msgpack observation):
  scalars: {"0@ur10e": {"joints": [...], "motion_group_id": "0@ur10e"}}

← Reply (msgpack PolicyResponse):
  {"joints": {"0@ur10e": [[step0], [step1], ..., [step15]]}, "dt_ms": 33.0}
```

Images arrive on separate subjects: `nova.policy.predict.images.flange` (PNG-encoded).

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `NATS_BROKER` | (injected by Nova) | NATS server URL |
| `NATS_SUBJECT` | `nova.policy.predict` | Subject to subscribe to |
| `BASE_PATH` | (injected by Nova) | URL prefix for HTTP routes |

## Deploy

```bash
nova app install policy/examples/apps/nats/mock-policy-service --omit-credentials
```

## Verify

```bash
curl http://<instance>/cell/mock-policy-service/status
# → {"ready":true,"nats_connected":true,"nats_subject":"nova.policy.predict","nats_requests":0,...}
```
