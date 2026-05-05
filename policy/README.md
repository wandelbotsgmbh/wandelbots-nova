# policy

PID-controlled jogging for executing learned policies (imitation learning, reinforcement learning) on industrial robots via [Wandelbots NOVA](https://wandelbots.com).

Converts joint position targets from a policy into joint velocity commands streamed through the NOVA Jogging API.

## Architecture

The core design principle: **robot control lives on the IPC, not on the (potentially remote) GPU server running the policy.**

```
┌─────────────────────┐         ┌──────────────────────────────┐
│  GPU Server          │         │  IPC (at the robot)           │
│                     │  NATS/  │                              │
│  Policy Model       │◄───────►│  PolicyExecutor              │
│                     │  ZMQ    │    ├─ PID velocity control   │
│                     │         │    ├─ Safety guards           │
└─────────────────────┘         │    └─ E-stop detection       │
                                │         │                     │
                                │         ▼                     │
                                │  NOVA Jogging API → Robot     │
                                └──────────────────────────────┘
```

The policy is a **stateless pure function**: `obs → actions`. It never controls lifecycle.
The executor decides **when** to start, **when** to stop, and handles safety.

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
    """Policy: receives robot state, returns joint targets."""
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
            timeout_s=10.0,  # run for 10 seconds
        )

        result = await executor.run()
        print(f"Done: {result.reason}, {result.steps} steps, {result.duration_s:.1f}s")


asyncio.run(main())
```

## API

### PolicyExecutor

```python
executor = PolicyExecutor(
    motion_groups=[mg1, mg2],   # or feature_map=... for flat features
    policy=my_policy_client,
    timeout_s=10.0,             # 0 = run until stop()
    safety_guards=[guard_fn],
    rate_hz=30,                 # how often the policy is queried (Hz)
)

# Blocking — runs until timeout/stop/error:
result = await executor.run()

# Non-blocking stop (call from another task, signal handler, HTTP endpoint):
executor.stop()
```

### Execution terminates when

| Trigger | `result.reason` |
|---------|----------------|
| `timeout_s` expires | `"timeout"` |
| `executor.stop()` called | `"stopped"` |
| Safety guard returns `False` | `"safety_guard"` |
| E-stop detected | `"estop"` |
| Exception | `"error"` |

### PolicyClient protocol

A policy is a stateless pure function. The client just transports obs/actions:

```python
class PolicyClient(Protocol):
    async def connect(self, motion_group_ids: list[str]) -> None: ...
    async def get_actions(self, obs: dict[str, Any]) -> ActionChunk | dict[str, float]: ...
    async def close(self) -> None: ...
```

Built-in clients:

| Client | Transport | Use case |
|--------|-----------|----------|
| `NatsPolicyClient` | NATS request/reply | App-to-app on Nova platform |
| `CallbackPolicyClient` | Local function | Testing, local models |
| `WebSocketPolicyClient` | WebSocket | Local dev (not through Nova proxy) |
| `Gr00tPolicyClient` | ZMQ (msgpack) | NVIDIA GR00T inference servers |

### Wire format (PolicyResponse)

Policy services in any language return this JSON:

```json
{
  "joints": {"0@ur10e": [[j1, j2, j3, j4, j5, j6]]},
  "ios": {"0@ur10e": {"digital_out[0]": true}},
  "dt_ms": 33.0
}
```

For multi-step chunks (ACT, Diffusion Policy):
```json
{
  "joints": {"0@ur10e": [[step0], [step1], ..., [step15]]},
  "dt_ms": 33.0
}
```

For flat features (FeatureMap mode):
```json
{
  "features": {"left_joint_1.pos": 0.1, "left_gripper.pos": 50.0}
}
```

## FeatureMap

Decouples the policy from hardware topology by mapping motion groups to flat named features. The policy never sees motion group IDs — only semantic role-based names:

```python
from policy import FeatureMap, FeatureGroup

feature_map = FeatureMap(groups=[
    FeatureGroup(motion_group=mg1, role="left", num_joints=6, gripper_io="digital_out[0]"),
    FeatureGroup(motion_group=mg2, role="right", num_joints=6, gripper_io="digital_out[0]"),
])

executor = PolicyExecutor(feature_map=feature_map, policy=client, timeout_s=10.0)
```

Policy sees: `{"left_joint_1.pos": ..., "right_gripper.pos": ...}`

## Safety Guards

```python
from policy import GuardState

def workspace_guard(ctx: GuardState) -> bool:
    """Return False to immediately stop the robot."""
    return ctx.state.pose.position[2] > 100  # stop if Z < 100mm

executor = PolicyExecutor(..., safety_guards=[workspace_guard])
```

Guards run on every PID tick (~100Hz). No network dependency.

## Examples

| Example | Description |
|---------|-------------|
| [`execute_policy_on_dualarm.py`](examples/execute_policy_on_dualarm.py) | Two robots, FeatureMap, safety guards |
| [`apps/mock-policy-service`](examples/apps/mock-policy-service/) | Stateless NATS policy service (deployable) |
| [`apps/policy-robot-controller`](examples/apps/policy-robot-controller/) | Robot controller app using NATS (deployable) |
| [`apps/mock-groot-policy-service`](examples/apps/mock-groot-policy-service/) | GR00T ZMQ mock service |
| [`apps/groot-robot-controller`](examples/apps/groot-robot-controller/) | GR00T robot controller app |

## Package structure

```
policy/
├── executor.py              # PolicyExecutor (run/stop, timeout, safety)
├── policy_client.py         # PolicyClient protocol + WS/Callback clients
├── nats_client.py           # NatsPolicyClient
├── gr00t_client.py          # Gr00tPolicyClient (ZMQ/msgpack)
├── feature_map.py           # FeatureMap for flat named features
├── runner.py                # PolicyRunner (low-level PID orchestrator)
├── pid_jogging_session.py   # Per-group jogging WebSocket + PID
├── velocity_controller.py   # PID controller (pure math)
├── types.py                 # ActionChunk, PolicyResponse, GuardState
└── tests/
```
