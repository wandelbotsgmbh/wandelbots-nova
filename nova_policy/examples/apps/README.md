# Example: Policy Service + Robot Controller Apps

Two Nova apps demonstrating end-to-end policy execution on virtual robots.

## Architecture

```
mock-policy-service          policy-robot-controller
┌──────────────────┐         ┌──────────────────────┐
│ POST /start      │         │ POST /prepare        │
│ POST /stop       │◄── WS ──│ POST /stop           │
│ GET /status      │         │ GET /status          │
│ WS /predict      │         │                      │
└──────────────────┘         └──────────┬───────────┘
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
# Note the IP address (e.g. 172.31.10.154)

# 2. Point CLI at the instance
nova config set host http://<IP>

# 3. Create two virtual controllers
nova controller create ur10e --manufacturer universalrobots --type universalrobots-ur10e
nova controller create ur10e-2 --manufacturer universalrobots --type universalrobots-ur10e

# 4. Install mock policy service
cd nova_policy/examples/apps/mock-policy-service
nova app install . --omit-credentials

# 5. Install robot controller (needs nova_policy in build context)
cd ../policy-robot-controller
cp -r ../../../../nova_policy .
nova app install . --omit-credentials
rm -rf nova_policy

# 6. Verify
nova app list
curl http://<IP>/cell/mock-policy-service/health
curl http://<IP>/cell/policy-robot-controller/status
```

## Usage

```bash
# Step 1: Prepare robots (connect, home, open jogging, connect WS)
curl -X POST http://<IP>/cell/policy-robot-controller/prepare \
  -H "Content-Type: application/json" \
  -d '{"duration_s": 60}'

# Poll until EXECUTING (robots holding position, ready)
curl http://<IP>/cell/policy-robot-controller/status

# Step 2: Start policy (robots move immediately)
curl -X POST http://<IP>/cell/mock-policy-service/start \
  -H "Content-Type: application/json" \
  -d '{"amplitude": 0.08, "frequency": 0.3, "duration_s": 15}'

# Step 3: Stop (or wait for policy to expire)
curl -X POST http://<IP>/cell/policy-robot-controller/stop
```

## API Reference

### mock-policy-service

| Endpoint | Description |
|---|---|
| `POST /start` | Start producing actions (amplitude, frequency, duration_s) |
| `POST /stop` | Stop producing actions |
| `GET /status` | Running state, connections, elapsed time |
| `WS /predict` | Obs in → joint target out (holds position until `/start`) |

### policy-robot-controller

| Endpoint | Description |
|---|---|
| `POST /prepare` | Connect → home → jog → WS → wait for policy |
| `POST /stop` | Cancel, return home |
| `GET /status` | Phase (IDLE/PREPARING/EXECUTING/STOPPING), step count |

## How It Works

1. `/prepare` moves robots to home, opens PID jogging sessions, connects WebSocket to the policy service
2. While policy is not started: WebSocket returns current position → robots hold still
3. `/start` on policy: WebSocket returns sinusoidal targets → PID tracks them → robots move
4. Policy expires: WebSocket returns hold-position again → robots stop
5. `/stop`: cancels everything, returns robots to home
