# policy

> **⚠️ EXPERIMENTAL** — This package is under active development and not ready for production use. Expect breaking changes between releases.

PID-controlled jogging for executing learned policies (imitation learning, reinforcement learning) on industrial robots via [Wandelbots NOVA](https://wandelbots.com).

Converts joint position targets from a policy into joint velocity commands streamed through the NOVA Jogging API.

## Architecture

**Robot control lives on the IPC, not on the (potentially remote) GPU server running the policy.**

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

    Policy <-->|"ZMQ / HTTP / custom"| Executor
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

A policy is just an async function: observations in, actions out.

```python
import asyncio
from nova import Nova
from policy import Observation, PolicyExecutor, PolicySchema


async def my_policy(obs):
    """Nudge each joint by a small offset."""
    return {k: v + 0.01 for k, v in obs.items() if k.startswith("arm_")}


async def main():
    async with Nova() as nova:
        cell = nova.cell()
        ctrl = await cell.controller("ur10e")
        mg = ctrl[0]

        schema = PolicySchema(observations=[
            Observation.joint_positions("arm", source=mg),
        ])

        executor = PolicyExecutor(schema, my_policy, timeout_s=10.0)
        result = await executor.run()
        print(f"Done: {result.reason}, {result.steps} steps, {result.duration_s:.1f}s")


asyncio.run(main())
```

Any async callable that maps `dict → dict` works — call a remote GPU server, run a local model, or return constants. The executor owns all complexity (PID control, safety, IO streaming, e-stop detection).

▶ [`execute_custom_policy_on_dual_arm.py`](examples/execute_custom_policy_on_dual_arm.py) — two UR5e robots with cameras, IOs, and safety guards\
▶ [`execute_gr00t_dual_arm.py`](examples/execute_gr00t_dual_arm.py) — dual arm with GR00T ZMQ + 4 cameras

## PolicySchema

Decouples the policy from hardware topology. The policy sees a flat dictionary of named features — it never knows about motion groups, controllers, or hardware IO keys.

```python
from policy import BoolMapping, Observation, PolicySchema

schema = PolicySchema(observations=[
    Observation.joint_positions("left", source=mg_left),
    Observation.joint_positions("right", source=mg_right),
    Observation.io("left_gripper", source=mg_left, io="digital_out[0]",
                   mapping=BoolMapping(on=100.0)),
    Observation.io("right_gripper", source=mg_right, io="digital_out[0]",
                   mapping=BoolMapping(on=100.0)),
])
```

This produces observations like:

```python
{
    "left_1": 0.1, "left_2": -1.5, ..., "left_6": 0.3,
    "right_1": 0.2, ..., "right_6": -0.1,
    "left_gripper": 0.0,      # closed
    "right_gripper": 100.0,   # open
}
```

The policy returns the same keys with target values. Joints go through PID jogging, IOs get written to hardware with the mapping applied in reverse.

### Cameras

```python
from policy import Observation, WebRTCCameras

cameras = WebRTCCameras(api_url="http://192.168.1.8:9100", width=640, height=480, fps=15)

schema = PolicySchema(observations=[
    Observation.joint_positions("arm", source=mg),
    Observation.image("flange", source=cameras.device("315122271048")),
    Observation.image("left", source=cameras.device("314522065367")),
])
```

Images arrive as `numpy.ndarray` (H×W×3, uint8, RGB) in the observation dict.

### Safety Guards

Guards run on every PID tick with access to joint state and streamed IO values:

```python
from policy import GuardState

def workspace_guard(ctx: GuardState) -> bool:
    """Return False to immediately stop the robot."""
    return ctx.state.pose.position[2] > 100  # stop if Z < 100mm

executor = PolicyExecutor(schema, policy, safety_guards=[workspace_guard])
```

### Execution lifecycle

| Trigger                      | Behavior                                    |
| ---------------------------- | ------------------------------------------- |
| `timeout_s` expires          | Returns `ExecutionResult(reason="timeout")` |
| `executor.stop()` called     | Returns `ExecutionResult(reason="stopped")` |
| Safety guard returns `False` | Raises `GuardStopError`                     |
| E-stop / protective stop     | Raises `EmergencyStopError`                 |
| Self-collision / joint limit | Raises `MotionError`                        |
| Connection lost              | Raises `RuntimeError`                       |

## PID Jogging (without a policy)

The PID jogging layer can be used standalone — no policy, no schema, no cameras:

```python
from policy import jog_joints

async with jog_joints(mg) as jogger:
    jogger.set_target([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    async for state in jogger:
        print(state.joints)
```

See [`JOGGING.md`](JOGGING.md) for joint/TCP modes, dual-arm control, chunking, error handling, and PID tuning.\
▶ [`jogging_dual_arm.py`](examples/jogging_dual_arm.py)

## GR00T

Built-in `Gr00tPolicyClient` for [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) inference servers over ZMQ. See [`gr00t/README.md`](gr00t/README.md).

---

## Advanced Schema Features

### IO mappings

By default, `Observation.io(...)` entries are bidirectional — the policy observes and controls them. The `mapping` converts between hardware values and policy values:

```python
# Policy sees 0.0 (closed) or 100.0 (open)
# Hardware reads/writes True/False on digital_out[0]
Observation.io("gripper", source=mg, io="digital_out[0]",
               mapping=BoolMapping(on=100.0))
```

For read-only sensors, set `action=False`:

```python
Observation.io("sensor", source=mg, io="digital_in[0]", action=False)
```

If observation and action need different hardware keys, use an explicit `Action.io()`:

```python
from policy import Action

schema = PolicySchema(
    observations=[
        Observation.io("gripper", source=mg, io="analog_in[0]", action=False),
    ],
    actions=[
        Action.io("gripper", target=mg, io="digital_out[0]",
                  mapping=BoolMapping(on=1.0)),
    ],
)
```

### Relative actions

Joint and TCP observations support `mode="relative"`. The mode controls how the policy's action output is interpreted:

| Mode | Policy returns | Executor sends to PID |
|------|----------------|----------------------|
| `"absolute"` (default) | target positions | as-is |
| `"relative"` | offsets from current | `current + offset` |

```python
Observation.joint_positions("arm", source=mg, mode="relative")
```

### TCP actions

Policies that output Cartesian targets instead of joint positions. Set `action=True` on `Observation.tcp()` — the executor creates a Cartesian PID jogging session for that motion group:

```python
Observation.tcp("eef_pose", source=mg, action=True)
```

The policy receives named values (`eef_pose_x`, `eef_pose_y`, `eef_pose_z`, `eef_pose_rx`, `eef_pose_ry`, `eef_pose_rz`) in mm and radians, and returns target values in the same format. Combine with `mode="relative"` for offset-based Cartesian control.

### Computed observations and actions

For external data sources (OPC UA, PLC, databases) not covered by the built-in types:

```python
async def read_force_sensor(obs: dict) -> dict:
    values = await opcua_client.read(["ns=2;s=ForceZ"])
    return {"force_z": values[0]}

schema = PolicySchema(observations=[
    Observation.joint_positions("arm", source=mg),
    Observation.computed(read_force_sensor),
])
```

Computed actions trigger external side effects when the policy returns:

```python
async def write_plc(action: dict) -> None:
    await plc_client.write("ns=2;s=ConveyorSpeed", action.get("conveyor_speed", 0.0))

schema = PolicySchema(
    observations=[Observation.joint_positions("arm", source=mg)],
    actions=[Action.computed(write_plc)],
)
```
