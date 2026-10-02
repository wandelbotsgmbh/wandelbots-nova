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
Observation.io("gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=100.0))
```

For read-only sensors, set `action=False`:

```python
Observation.io("sensor", source=mg, io="digital_in[0]", action=False)
```

If observation and action need different hardware keys, use an explicit
`Action.io()`:

```python
from novapolicy import Action

schema = PolicySchema(
    observations=[
        Observation.io("gripper", source=mg, io="analog_in[0]", action=False),
    ],
    actions=[
        Action.io("gripper", target=mg, io="digital_out[0]", mapping=BoolMapping(on=1.0)),
    ],
)
```

## Units

NOVA speaks radians and millimetres. A dataset does not necessarily, and a policy trained on
degrees normalizes cleanly against its own statistics before moving the arm to the wrong place —
because those statistics are in the *recording* robot's units.

Declare the conversion once and it applies in both directions: forward on the observation,
inverted on the action.

```python
from novapolicy import Clamp, Observation, Rad2Deg, Scale

Observation.joint_positions("arm", source=mg, ops=[Rad2Deg()])

# Position is millimetres and orientation radians, so they convert apart.
Observation.tcp(
    "eef",
    source=mg,
    action=True,
    position_ops=[Scale(0.001)],  # mm -> m
    orientation_ops=[Rad2Deg()],
)
```

| Operator | Direction | Notes |
| --- | --- | --- |
| `Rad2Deg()` | bijective | Exact both ways. |
| `Scale(factor)` | bijective | A zero or non-finite factor is rejected at construction. |
| `Clamp(low, high)` | bidirectional | Bounds the observation forward *and* the command on the way back — limiting a command is the safety-relevant direction. |

Operators run front-to-back on the observation and back-to-front on the action, so
`[Scale(0.001), Clamp(-1, 1)]` clamps in metres in both directions. They are element-wise, and an
operator that declares itself forward-only is rejected on a channel the policy writes — the
executor would have nothing to send back.

The inversion happens in the executor **before** relative targets resolve: a delta in degrees added
to a state in radians is exactly the error this prevents.

This is **not** normalization. A policy server applies the checkpoint's own normalization and
unnormalization; duplicating it here would corrupt the values twice over. What is unguarded, and
what these operators are for, is units and scale.

Rotation-representation conversion — rotation vector to quaternion or Euler — is out of scope;
these are element-wise scalars. Operators do not apply to `computed` or `constant` observations.

`ops` are distinct from the IO `Mapping` above on purpose: a `Mapping` crosses a *type* boundary
(hardware bools and analogue levels to the policy's floats) and is applied by each policy client as
it decodes its own wire format, while a `ValueOp` is a float-to-float *units* transform the schema
itself applies at one seam.

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
`action=True` on `Observation.tcp()` — the executor sends `POSE` waypoints for
that motion group, and the server handles inverse kinematics internally:

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


schema = PolicySchema(
    observations=[
        Observation.joint_positions("arm", source=mg),
        Observation.computed(read_force_sensor),
    ]
)
```

Computed actions trigger external side effects after each policy call, receiving the returned `ActionChunk`:

```python
async def journal(chunk: ActionChunk) -> None:
    await db.write(chunk.joints)


schema = PolicySchema(
    observations=[Observation.joint_positions("arm", source=mg)],
    actions=[Action.computed(journal)],
)
```

## Rerun visualization

See [rerun.md](rerun.md) for real-time 3D visualization of execution.
