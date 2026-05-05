# Example Apps

Two deployment patterns for policy execution on the Nova platform.

## `nats/` — NATS request/reply

App-to-app communication via NATS (built into every Nova instance).

- **`mock-policy-service`** — Stateless inference endpoint. Subscribes to a NATS subject and replies with action chunks.
- **`policy-robot-controller`** — Moves robots to home, then runs a policy episode via PID jogging. Queries the policy service over NATS.

## `zmq/` — ZeroMQ (GR00T)

Direct ZMQ REQ/REP for GR00T-compatible inference servers.

- **`mock-groot-policy-service`** — Stateless GR00T-compatible inference server (ZMQ + msgpack).
- **`groot-robot-controller`** — Dual-arm controller that queries the GR00T service and drives robots via PID jogging.
