# Schema Reference

Advanced `PolicySchema` features. For the basics (joint positions, cameras, stop
conditions) see the [README](../README.md). The schema decouples the policy from
hardware topology: the policy sees a flat dict of named features and never knows
about motion groups, controllers, or hardware IO keys.

## IO mappings

By default, `Observation.io(...)` entries are bidirectional — the policy observes
and controls them. The `mapping` converts between hardware values and policy
values:

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

If observation and action need different hardware keys, use an explicit
`Action.io()`:

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

## Relative actions

Joint and TCP observations support `mode="relative"`. The mode controls how the
policy's action output is interpreted:

| Mode                   | Policy returns       | Executor sends to jogging |
| ---------------------- | -------------------- | ------------------------- |
| `"absolute"` (default) | target positions     | as-is                     |
| `"relative"`           | offsets from current | `current + offset`        |

```python
Observation.joint_positions("arm", source=mg, mode="relative")
```

## TCP actions

Policies that output Cartesian targets instead of joint positions. Set
`action=True` on `Observation.tcp()` — the executor sends `PoseWaypointsRequest`
for that motion group, and the server handles inverse kinematics internally:

```python
Observation.tcp("eef_pose", source=mg, tcp="Flange", action=True)
```

The policy receives named values (`eef_pose_x`, `eef_pose_y`, `eef_pose_z`,
`eef_pose_rx`, `eef_pose_ry`, `eef_pose_rz`) in mm and radians (NOVA's native TCP
format), and returns target values in the same format. Combine with
`mode="relative"` for delta-based Cartesian control.

## Computed observations and actions

For external data sources (OPC UA, PLC, databases) not covered by the built-in
types:

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

## Rerun visualization

See [RERUN.md](RERUN.md) for real-time 3D visualization of execution.
