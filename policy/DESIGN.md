# Policy Package — Design Document

## Goal

Provide a compact SDK module (`policy`) that enables AI policy inference loops to stream **action chunks** (joint positions + IO values) to one or more motion groups and have them executed in real-time via PID-controlled jogging.

The package bridges the gap between an AI policy (which outputs discrete joint/gripper targets at ~10–50 Hz) and NOVA's velocity-based jogging API (which requires continuous joint velocity commands).

> **Scope:** This is a client-side PID bridge. NOVA will eventually provide native position streaming
> (JoggingChunkRequest) making client-side PID obsolete. When that lands, only the internal
> transport changes — the public API (`PolicyExecutor.run()`) stays the same.

## Design Principles

1. **Policy is stateless**: `obs → actions`. No lifecycle, no "done" signal, no episode state.
2. **Executor owns everything**: start, stop, safety, timeout, homing, camera connection.
3. **Robot control stays on IPC**: Never on the remote GPU server.
4. **Single episode**: `run()` executes one episode. Caller handles multi-episode loops.
5. **Exceptions for abnormal stops**: `GuardStopError`, `EmergencyStopError`, `MotionError`.
6. **Normal returns for expected stops**: timeout, explicit stop.

## Data Flow

```
PolicyExecutor.run()
  │
  ├─ CameraSet.connect()        # WebRTC ICE negotiation (10-15s)
  ├─ FeatureMap.start()          # IO WebSocket streams
  ├─ PolicyRunner.__aenter__()   # opens jogging sessions
  │
  │  ┌─ loop at rate_hz ────────────────────────────────────┐
  │  │  1. runner.observe()     → dict[str, RobotState]     │
  │  │  2. feature_map.build()  → flat obs + IO values      │
  │  │  3. cameras.read()       → numpy images              │
  │  │  4. policy.get_actions() → ActionChunk               │
  │  │  5. runner.send(chunk)   → PID → velocity → robot    │
  │  │  6. check guards, estop, collision                   │
  │  └──────────────────────────────────────────────────────┘
  │
  ├─ PolicyRunner.__aexit__()    # zero velocity, close jogging
  ├─ FeatureMap.stop()           # close IO streams
  ├─ CameraSet.disconnect()      # close WebRTC
  └─ return ExecutionResult

```

## Transport Options

| Transport | Class | When to use |
|-----------|-------|-------------|
| NATS | `NatsPolicyClient` | App-to-app on Nova platform (recommended) |
| ZMQ | `Gr00tPolicyClient` | NVIDIA GR00T servers (local or remote GPU) |
| WebSocket | `WebSocketPolicyClient` | Local dev only (blocked by Nova proxy) |
| Local fn | `CallbackPolicyClient` | Tests, embedded models |

### Why not WebSocket on Nova?

The Nova ingress proxy (`api-gateway` nginx) performs auth via `auth_request` subrequest. WebSocket connections require the auth token in the `Sec-WebSocket-Protocol` header. Custom app routes get 403 without this. NATS bypasses the HTTP proxy entirely (injected as `NATS_BROKER` env var).

### NATS wire format

- Scalar observations: msgpack, sent via NATS request/reply
- Camera images: PNG-compressed, published on `<subject>.images.<name>` (separate subjects to stay under 1MB NATS max_payload)

## Safety Architecture

```
PID Jogging Tick (~100Hz)
  │
  ├─ Read joint state (WebSocket stream)
  ├─ Read IO values (WebSocket stream cache)
  ├─ Run safety guards (user-defined, access to state + IOs)
  │     └─ returns False → GuardStopError (zero velocity immediately)
  ├─ Check NOVA jogging state:
  │     └─ PAUSED_NEAR_COLLISION → MotionError (after 10 tick confirmation)
  │     └─ PAUSED_NEAR_JOINT_LIMIT → MotionError
  │     └─ PAUSED_NEAR_SINGULARITY → MotionError
  ├─ Check safety state:
  │     └─ not NORMAL/REDUCED → EmergencyStopError
  └─ PID compute → send velocity
```

## PID Controller

Pure math, no I/O:

```python
class VelocityController:
    def compute(self, current: list[float], target: list[float]) -> list[float]:
        """Returns clamped joint velocities."""
```

- P-gain: 3.0 (default, must match training)
- D-gain: 0.1
- I-gain: 0.0 (disabled)
- Feedforward: 0.0 (disabled, available for tuning)
- Anti-windup: clamp integral at ±2.0
- Velocity limit: 1.5 rad/s per joint
- `time.monotonic()` for stable dt

## IO Handling

- **Reads**: Via WebSocket stream (`stream_io_values`), shared cache per controller
- **Writes**: Fire-and-forget HTTP with deduplication (only write on value change)
- **Guard access**: Guards receive `ctx.io_values` dict at PID tick rate
- **Threshold**: Bool IO written when float feature crosses 0.5

## Migration Path

When NOVA adds native position streaming:

1. `VelocityController` → removed (server accepts positions)
2. `PidJoggingSession` → sends position chunks directly
3. `PolicyExecutor.run()` → unchanged
4. `ActionChunk` → unchanged (already has multi-step + `dt_ms`)

## File Structure

```
policy/
├── executor.py              # PolicyExecutor: run(), stop(), safety orchestration
├── runner.py                # PolicyRunner: manages multiple PidJoggingSessions
├── pid_jogging_session.py   # Per-group: WS jogging + PID + IO write + standstill
├── velocity_controller.py   # PID math (P, I, D, FF, anti-windup, velocity clamp)
├── feature_map.py           # FeatureMap + FeatureGroup + IO stream cache
├── cameras.py               # WebRTCCameraConfig, CameraSet, _WebRTCConnection
├── policy_client.py         # PolicyClient protocol + WS/Callback implementations
├── nats_client.py           # NatsPolicyClient (NATS request/reply)
├── nats_wire.py             # msgpack + PNG serialization
├── gr00t_client.py          # Gr00tPolicyClient (ZMQ + msgpack + numpy)
├── types.py                 # ActionChunk, PolicyResponse, GuardState, errors
└── tests/
    ├── test_policy.py       # Unit tests for executor + runner
    ├── test_nats_client.py  # NATS client tests
    └── test_gr00t_client.py # GR00T client tests
```
