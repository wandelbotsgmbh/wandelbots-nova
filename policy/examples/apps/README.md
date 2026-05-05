# Example: Policy Service + Robot Controller Apps

Nova apps demonstrating end-to-end policy execution on virtual robots.

Included examples:
- `mock-policy-service`: NATS-based mock for the JSON `PolicyResponse` protocol
- `policy-robot-controller`: robot-side executor app using NATS to query the mock
- `mock-groot-policy-service`: stateless GR00T-compatible **dual-arm** ZMQ mock service
- `groot-robot-controller`: robot-side executor app using `Gr00tPolicyClient` over ZMQ

## Architecture

The **PolicyExecutor is the active side**. It connects to the policy service,
sends observations, receives predictions, and drives PID jogging.

A policy service is treated as a **stateless inference endpoint**:
- it does not start robot motion
- it does not own the execution lifecycle
- same observation + same config => same action chunk

```
policy-robot-controller          mock-policy-service
┌──────────────────────┐         ┌──────────────────┐
│ POST /start          │── NATS ─►│ NATS subscribe  │
│ GET /status          │ req/rep │ POST /configure  │
│ POST /stop           │◄────────│ GET /status      │
└──────────┬───────────┘         └──────────────────┘
           │ PID jogging
           ▼
 ┌──────────────────────┐
 │  ur10e  +  ur10e-2   │
 └──────────────────────┘
```

## Setup

```bash
# 1. Create a virtual instance with two UR10e robots
nova virtual create
# Note the IP address (e.g. 172.31.12.106)

# 2. Point CLI at the instance
nova config set host http://<IP>

# 3. Create two virtual controllers
nova controller create ur10e --manufacturer universalrobots --type universalrobots-ur10e
nova controller create ur10e-2 --manufacturer universalrobots --type universalrobots-ur10e

# 4. Install mock policy service
cd policy/examples/apps/mock-policy-service
nova app install . --omit-credentials

# 5. Install robot controller (needs policy in build context)
cd ../policy-robot-controller
cp -r ../../../../policy .
nova app install . --omit-credentials
rm -rf policy

# 6. Verify
nova app list
curl http://<IP>/cell/mock-policy-service/health
curl http://<IP>/cell/policy-robot-controller/status
```

## Usage

```bash
# Optional: configure the stateless policy mapping.
# This does not start robot motion.
curl -X POST http://<IP>/cell/mock-policy-service/configure \
  -H "Content-Type: application/json" \
  -d '{"amplitude": 0.08, "joint_phase": 0.4, "step_phase": 0.2, "chunk_size": 16, "dt_ms": 33.0}'

# Start the executor. This is what starts motion:
# it connects via NATS and continuously queries inference.
curl -X POST http://<IP>/cell/policy-robot-controller/start \
  -H "Content-Type: application/json" \
  -d '{"nats_subject": "nova.v2.cells.cell.apps.mock-policy-service.predict", "timeout_s": 60}'

# Poll executor status
curl http://<IP>/cell/policy-robot-controller/status

# Stop execution
curl -X POST http://<IP>/cell/policy-robot-controller/stop
```

## GR00T Setup

```bash
# Install the stateless dual-arm GR00T mock
cd policy/examples/apps/mock-groot-policy-service
nova app install . --omit-credentials

# Install the GR00T robot controller (needs policy in build context)
cd ../groot-robot-controller
cp -r ../../../../policy .
nova app install . --omit-credentials
rm -rf policy

# Start dual-arm execution against the ZMQ mock
curl -X POST http://<IP>/cell/groot-robot-controller/start \
  -H "Content-Type: application/json" \
  -d '{}'
```

## API Reference

### mock-policy-service

| Endpoint | Description |
|---|---|
| `GET /health` | Liveness check |
| `GET /status` | NATS connection state, request count, config |
| `POST /configure` | Update the stateless observation → action mapping |
| NATS `nova.v2.cells.cell.apps.mock-policy-service.predict` | Request/reply inference |

### policy-robot-controller

| Endpoint | Description |
|---|---|
| `POST /start` | Connect → home → query policy via NATS → PID execute |
| `POST /stop` | Cancel, return home |
| `GET /status` | Phase and step count |

### mock-groot-policy-service

| Endpoint | Description |
|---|---|
| `GET /health` | Liveness check |
| `GET /status` | Request counters and expected dual-arm modality keys |
| `ZMQ ping` | Health check on port `5555` |
| `ZMQ get_action` | GR00T-style stateless inference |
| `ZMQ reset` | No-op, returns `{"stateless": true}` |
| `ZMQ get_modality_config` | Returns mock modality metadata |

### groot-robot-controller

| Endpoint | Description |
|---|---|
| `POST /start` | Connect → home → build GR00T obs → ZMQ get_action → PID execute |
| `POST /stop` | Stop executor and close NOVA connection |
| `GET /status` | Executor status |

## How It Works

1. Configure the policy service if needed; otherwise leave it in its default ready state
2. `POST /start` on `policy-robot-controller` creates the `PolicyExecutor`, homes robots, and connects to NATS
3. The executor sends observations via NATS request/reply and receives deterministic predictions
4. The executor converts those predictions into PID jogging commands and drives the robots
5. `POST /stop` stops the executor and returns robots to home
