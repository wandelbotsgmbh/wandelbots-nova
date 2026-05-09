# Example Apps

Deployable Nova apps demonstrating policy execution patterns.

## `gr00t/` — GR00T ZMQ

- **[`gr00t-dual-arm-controller`](gr00t/gr00t-dual-arm-controller/)** — Dual-arm UR5e controller with 4 Isaac Sim cameras, using GR00T ZMQ inference.

## `mock-camera-server/` — WebRTC camera mock

Local camera server for development without real cameras. Streams video from a HuggingFace dataset over WebRTC.

```bash
cd policy/examples/apps/mock-camera-server
uv run python -m mock_camera_server
# Open http://localhost:9100
```

## Architecture

```mermaid
flowchart LR
    Controller["Robot Controller\n(PolicyExecutor)"]
    Policy["Policy Server\n(stateless)"]
    Robot["Robot"]

    Controller <-->|"ZMQ / callback"| Policy
    Controller -->|"NOVA Jogging API"| Robot
```

The executor (robot controller) is always the **active side** — it owns PID jogging, safety, and lifecycle.
The policy service is always **passive/stateless** — it just replies to observation → action queries.
