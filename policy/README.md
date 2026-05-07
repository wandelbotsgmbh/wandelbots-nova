# policy

PID-controlled jogging for executing learned policies (imitation learning, reinforcement learning) on industrial robots via [Wandelbots NOVA](https://wandelbots.com).

Converts joint position targets from a policy into joint velocity commands streamed through the NOVA Jogging API.

## Architecture

The core design principle: **robot control lives on the IPC, not on the (potentially remote) GPU server running the policy.**

```mermaid
flowchart LR
    subgraph GPU["GPU Server"]
        Policy["Policy Model\n(stateless)"]
    end

    subgraph IPC["IPC (at the robot)"]
        Executor["PolicyExecutor"]
        PID["PID velocity control"]
        Safety["Safety guards"]
        Cameras["WebRTC cameras"]
        Jogging["NOVA Jogging API"]
        Robot["Robot"]
    end

    subgraph CamServer["Camera Server"]
        WebRTC["WebRTC streams"]
    end

    Policy <-->|"NATS / ZMQ"| Executor
    WebRTC <-->|"WebRTC"| Cameras
    Executor --> PID --> Jogging --> Robot
    Executor --> Safety
    Cameras --> Executor
```

The policy is a **stateless pure function**: `obs → actions`. It never controls lifecycle.
The executor decides **when** to start, **when** to stop, and handles all safety.

## Install

```bash
pip install wandelbots-nova[policy]
```

## Quick Start

```python
import asyncio
import math
from typing import Any

from nova import Nova
from policy import ActionChunk, CallbackPolicyClient, PolicyExecutor


async def my_policy(obs: dict[str, Any]) -> ActionChunk:
    """Stateless policy: obs in → actions out."""
    joints = {}
    for mg_id, state in obs.items():
        current = list(state.joints)
        target = [j + 0.05 * math.sin(j * 3.0 + i * 0.4) for i, j in enumerate(current)]
        joints[mg_id] = [target]
    return ActionChunk(joints=joints)


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        ctrl = await cell.controller("ur10e")
        mg = ctrl[0]

        executor = PolicyExecutor(
            motion_groups=[mg],
            policy=CallbackPolicyClient(my_policy),
            timeout_s=10.0,
        )

        result = await executor.run()
        print(f"Done: {result.reason}, {result.steps} steps, {result.duration_s:.1f}s")


asyncio.run(main())
```

## API

### PolicyExecutor

```python
executor = PolicyExecutor(
    feature_map=feature_map,
    policy=my_policy_client,
    cameras=camera_set,             # WebRTC cameras
    timeout_s=10.0,                 # 0 = run until stop()
    safety_guards=[guard_fn],
    rate_hz=30,
)

# Blocking — runs until timeout/stop/error:
result = await executor.run()

# Non-blocking stop (call from another task, signal handler, HTTP endpoint):
executor.stop()
```

### Execution terminates when

| Trigger | Behavior |
|---------|----------|
| `timeout_s` expires | Returns `ExecutionResult(reason="timeout")` |
| `executor.stop()` called | Returns `ExecutionResult(reason="stopped")` |
| Safety guard returns `False` | Raises `GuardStopError` |
| E-stop / protective stop | Raises `EmergencyStopError` |
| Self-collision / joint limit | Raises `MotionError` |
| Connection lost | Raises `RuntimeError` |

### Policy Clients

| Client | Transport | Use case |
|--------|-----------|----------|
| `NatsPolicyClient` | NATS request/reply | App-to-app on Nova platform |
| `CallbackPolicyClient` | Local function | Testing, local models |
| `Gr00tPolicyClient` | ZMQ (msgpack) | NVIDIA GR00T inference servers |

### Wire format (PolicyResponse)

Policy services return msgpack-encoded responses:

```python
# Single-step action:
{"joints": {"0@ur10e": [[j1, j2, j3, j4, j5, j6]]}, "ios": {"0@ur10e": {"digital_out[0]": True}}, "dt_ms": 33.0}

# Multi-step chunk (ACT, Diffusion Policy, RTC):
{"joints": {"0@ur10e": [[step0], [step1], ..., [step15]]}, "dt_ms": 33.0}

# Flat features (FeatureMap mode):
{"features": {"left_joint_position_1": 0.1, "left_gripper": 50.0}}
```

## FeatureMap

Decouples the policy from hardware topology using flat named features. The policy operates on a flat dictionary — it never knows about motion groups, controllers, or hardware topology. Feature names are the contract, defined at training time.

### What the policy sees

```python
# Observation:
{
    "left_joint_position_1": 0.1,
    "left_joint_position_2": -1.5,
    ...
    "left_gripper": 0.0,
    "right_joint_position_1": 0.2,
    ...
    "right_gripper": 1.0,
}

# Action (same structure):
{
    "left_joint_position_1": 0.15,
    ...
    "left_gripper": 1.0,
    "right_joint_position_1": 0.25,
    ...
}
```

### FeatureGroup

```python
@dataclass
class FeatureGroup:
    motion_group: MotionGroup
    name: str                          # default prefix for feature keys
    ios: dict[str, str] | None         # policy_name → hardware_io_key
    joint_key: str = ""                # override (default: "{name}_joint_position")
    tcp_key: str = ""                  # override (default: "{name}_tcp")
    tcp_format: TcpFormat = NONE       # NONE, ROTATION_VECTOR, QUATERNION, ROT6D
    model_dof: int = 0                 # expected DOF (0 = auto from robot)
    io_threshold: float = 0.5          # bool conversion threshold for IO actions
```

Key resolution:
- **Joints**: `{joint_key}_{i}` → e.g. `left_joint_position_1`
- **TCP**: `{tcp_key}_{i}` (only if `tcp_format != NONE`)
- **IOs**: dict keys used directly as feature names

### Usage (LeRobot flat features)

```python
from policy import FeatureMap, FeatureGroup, PolicyExecutor, NatsPolicyClient

feature_map = FeatureMap(groups=[
    FeatureGroup(
        motion_group=mg1,
        name="left",
        ios={"left_gripper": "digital_out[0]"},
    ),
    FeatureGroup(
        motion_group=mg2,
        name="right",
        ios={"right_gripper": "digital_out[0]"},
    ),
])

executor = PolicyExecutor(
    feature_map=feature_map,
    policy=NatsPolicyClient(nc, subject="nova.policy.predict"),
    timeout_s=10.0,
)
result = await executor.run()
```

### Usage (GR00T array-based)

Override keys to match the GR00T server's expected modality config:

```python
from policy import FeatureMap, FeatureGroup, Gr00tPolicyClient, TcpFormat

feature_map = FeatureMap(groups=[
    FeatureGroup(
        motion_group=mg1,
        name="left",
        joint_key="left_arm",
        tcp_key="left_eef_9d",
        tcp_format=TcpFormat.ROT6D,
        ios={"left_gripper": "digital_out[0]"},
    ),
    FeatureGroup(
        motion_group=mg2,
        name="right",
        joint_key="right_arm",
        tcp_key="right_eef_9d",
        tcp_format=TcpFormat.ROT6D,
        ios={"right_gripper": "digital_out[0]"},
    ),
])

client = Gr00tPolicyClient(host="gpu-server", port=5555, feature_map=feature_map)
```

### IO Handling

- **Reads**: `FeatureMap.start()` opens one stream per controller. Values update at controller rate. Guards and observations read from this shared cache.
- **Writes**: Fire-and-forget with deduplication (only writes on value change to avoid 429s).
- **Threshold**: Bool IO written when float feature crosses `io_threshold` (default 0.5).

## Cameras

WebRTC cameras are attached to the executor. Images are included in every observation:

```python
from policy import CameraSet, WebRTCCameraConfig

cameras = CameraSet(configs={
    "flange": WebRTCCameraConfig(api_url="http://localhost:9100", device_id="315122271048"),
    "left": WebRTCCameraConfig(api_url="http://localhost:9100", device_id="314522065367"),
})

executor = PolicyExecutor(
    feature_map=feature_map,
    cameras=cameras,
    policy=client,
    timeout_s=10.0,
)
```

Images arrive as `numpy.ndarray` (H×W×3, uint8, RGB) in the observation dict under the camera name.

## Safety Guards

Guards run on every PID tick at the controller's state stream rate. They have access to joint state and streamed IO values:

```python
from policy import GuardState

def workspace_guard(ctx: GuardState) -> bool:
    """Return False to immediately stop the robot."""
    return ctx.state.pose.position[2] > 100  # stop if Z < 100mm

def io_guard(ctx: GuardState) -> bool:
    """Stop if an external sensor triggers."""
    sensor = ctx.io_values.get("digital_in[3]")
    return sensor != 1  # stop if sensor goes high

executor = PolicyExecutor(..., safety_guards=[workspace_guard, io_guard])
```

## Collision & Limit Detection

The executor uses NOVA's jogging state signals to detect when the robot is blocked:

- **Self-collision** → raises `MotionError("Jogging paused: PAUSED_NEAR_COLLISION")`
- **Joint limit** → raises `MotionError("Jogging paused: PAUSED_NEAR_JOINT_LIMIT")`
- **Singularity** → raises `MotionError("Jogging paused: PAUSED_NEAR_SINGULARITY")`

No heuristics — uses the actual controller state reported by NOVA.

## NATS Transport (Nova Platform)

On the Nova platform, apps communicate via NATS (injected as `NATS_BROKER` env var into every app container).

```python
import nats
from policy import NatsPolicyClient

nc = await nats.connect(servers="nats://localhost:4222")
client = NatsPolicyClient(nc, subject="nova.policy.predict", timeout=5.0)
```

NATS wire format:
- Scalar observations: msgpack, sent via request/reply
- Camera images: PNG-compressed, published on `<subject>.images.<name>` (separate subjects to stay under 1MB NATS max_payload)

## Examples

| Example | Description |
|---------|-------------|
| [`execute_policy_on_dualarm.py`](examples/execute_policy_on_dualarm.py) | Two UR10e robots, FeatureMap, cameras, safety guards |
| [`execute_groot_single_arm.py`](examples/execute_groot_single_arm.py) | Single arm with GR00T ZMQ inference server |
| [`apps/nats/`](examples/apps/nats/) | NATS mock policy + robot controller (deployable Nova apps) |
| [`apps/zmq/`](examples/apps/zmq/) | GR00T ZMQ mock policy + robot controller (deployable) |
| [`apps/mock-camera-server/`](examples/apps/mock-camera-server/) | WebRTC camera server for development without real cameras |

---

## Design Internals

> The following sections document internal architecture for contributors.

### Design Principles

1. **Policy is stateless**: `obs → actions`. No lifecycle, no "done" signal, no episode state.
2. **Executor owns everything**: start, stop, safety, timeout, camera connection.
3. **Robot control stays on IPC**: Never on the remote GPU server.
4. **Single episode**: `run()` executes one episode. Caller handles multi-episode loops.
5. **Exceptions for abnormal stops**: `GuardStopError`, `EmergencyStopError`, `MotionError`.
6. **Normal returns for expected stops**: timeout, explicit stop.

### Data Flow

```mermaid
flowchart TD
    run["PolicyExecutor.run()"] --> cam["CameraSet.connect()"]
    cam --> io["FeatureMap.start() — IO streaming"]
    io --> jog["PolicyRunner — open jogging sessions"]
    jog --> loop

    subgraph loop["Loop at rate_hz"]
        direction TB
        observe["1. runner.observe() → RobotState"] --> build["2. feature_map.build() → flat obs + IOs"]
        build --> read["3. cameras.read() → numpy images"]
        read --> infer["4. policy.get_actions() → ActionChunk"]
        infer --> send["5. runner.send(chunk) → PID → velocity"]
        send --> check["6. check guards, estop, collision"]
    end

    loop --> cleanup["Close jogging, IO streams, cameras"]
    cleanup --> result["return ExecutionResult"]
```

### Safety Architecture

```mermaid
flowchart TD
    tick["PID Jogging Tick"] --> state["Read joint state"]
    state --> ioread["Read IO values from stream cache"]
    ioread --> guards["Run safety guards"]
    guards -->|"returns False"| gstop["GuardStopError"]
    guards -->|"ok"| jstate["Check NOVA jogging state"]
    jstate -->|"PAUSED_NEAR_COLLISION\nPAUSED_NEAR_JOINT_LIMIT\nPAUSED_NEAR_SINGULARITY"| merr["MotionError"]
    jstate -->|"ok"| safety["Check safety state"]
    safety -->|"not NORMAL/REDUCED"| estop["EmergencyStopError"]
    safety -->|"ok"| pid["PID compute → send velocity"]
```

### PID Controller

Pure math, no I/O:

- P-gain: 3.0 (must match training recording)
- D-gain: 0.1
- I-gain: 0.0 (disabled)
- Anti-windup: clamp integral at ±2.0
- Velocity limit: 1.5 rad/s per joint

### Migration Path

When NOVA adds native position streaming (JoggingChunkRequest):

1. `VelocityController` → removed (server accepts positions directly)
2. `PidJoggingSession` → sends position chunks directly
3. `PolicyExecutor.run()` → unchanged
4. `ActionChunk` → unchanged (already has multi-step + `dt_ms`)

### File Structure

```
policy/
├── executor.py              # PolicyExecutor: run(), stop(), safety orchestration
├── runner.py                # PolicyRunner: manages multiple PidJoggingSessions
├── pid_jogging_session.py   # Per-group: jogging + PID + IO write + standstill
├── velocity_controller.py   # PID math (P, I, D, FF, anti-windup, velocity clamp)
├── feature_map.py           # FeatureMap + FeatureGroup + IO stream cache
├── cameras.py               # WebRTCCameraConfig, CameraSet
├── policy_client.py         # PolicyClient protocol + Callback implementation
├── nats_client.py           # NatsPolicyClient (NATS request/reply)
├── nats_wire.py             # msgpack + PNG serialization
├── gr00t_client.py          # Gr00tPolicyClient (ZMQ + msgpack + numpy)
├── pose.py                  # TCP pose conversion utilities
├── types.py                 # ActionChunk, PolicyResponse, GuardState, errors
└── tests/
```
