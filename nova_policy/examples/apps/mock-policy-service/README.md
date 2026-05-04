# mock-policy-service

Simulates a policy inference server that outputs sinusoidal joint targets via WebSocket.

## API

| Endpoint | Description |
|---|---|
| `POST /start` | Start generating actions (amplitude, frequency, duration_s) |
| `POST /stop` | Stop generating actions |
| `GET /status` | Running state, connections, config |
| `WS /predict` | Single-step: send obs → receive joint target |
| `WS /predict_chunked` | Multi-step: send obs → receive 16-step chunk |

## Behavior

- When not started (or expired): WebSocket returns current position (hold still)
- When started: WebSocket returns sinusoidal targets around home position
- Multiple clients can connect simultaneously (one per robot)

## Install

```bash
nova app install . --omit-credentials
```
