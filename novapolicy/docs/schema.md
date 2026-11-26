# Schema reference

A `PolicySchema` says what a policy observes and what it commands. The policy sees a flat dict of
named features. It never sees motion groups, controllers or hardware IO keys. For a first schema
(joint positions, cameras, stop conditions) see the [README](../README.md).

## Observations

Each observation adds named features to the dict the policy sees.

| Observation | Observes | The policy sees | Details |
| --- | --- | --- | --- |
| `Observation.joint_positions(key, source=mg)` | joint positions of one or more motion groups | `key_1 .. key_n`, rad | [Units](#units) |
| `Observation.joint_velocities(key, source=mg)` | joint velocities | `key_1 .. key_n`, rad/s | [below](#joint-velocities) |
| `Observation.tcp(key, source=mg, tcp=...)` | a motion group's TCP pose | `key_x .. key_rz` (mm, rotation vector), or the `orientation_type` given | [TCP actions](#tcp-actions), [Rotations and frames](#rotation-representation-and-coordinate-systems) |
| `Observation.pose(key, source=...)` | the pose of an object or target, from a source | like `Observation.tcp` | [below](#object-poses) |
| `Observation.io(key, source=mg, io=...)` | an IO value | `key`, through its `Mapping` | [IO mappings](#io-mappings) |
| `Observation.image(key, source=camera)` | a camera frame | `key`, an H×W×3 uint8 RGB array | [below](#images) |
| `Observation.constant(key, value)` | a fixed value, such as a language instruction | `key` | |
| `Observation.computed(fn, keys=[...])` | values from an async function | the keys it returns | [Computed observations](#computed-observations-and-actions) |

### Joint velocities

```python
Observation.joint_velocities("joint_vel", source=mg)  # joint_vel_1 .. joint_vel_n, rad/s
```

The controller's measured velocities are used where it sends them. Otherwise they are
finite-differenced from consecutive joint positions, which is noisier and logged once. The
executor starts the measurement with the run. Outside a run, wrap `build_observation` in
`await schema.start_sources()` and `await schema.stop_sources()`.

### Object poses

`Observation.pose` observes the pose of anything that is not a motion group: an object, a target,
a fixture. Its source gives a NOVA `Pose` (mm, rotation vector). The policy sees it as
`Observation.tcp` shows a TCP, converted by `orientation_type` and the operators:

```python
Observation.pose(
    "cube",
    source=detector.cube_pose,  # a Pose, an async callable returning one, or a PoseSource
    orientation_type=OrientationType.QUATERNION,  # cube_x, cube_y, cube_z, cube_qx .. cube_qw
    position_ops=[Scale(0.001)],  # mm -> m
)
```

| Source | For |
| --- | --- |
| `Pose(...)` | a fixed target |
| async callable returning a `Pose` | an estimator fast enough to ask on every step, such as a CNN |
| `PoseSource`: `async start()`, `async stop()`, `read() -> Pose` | a detector running at its own rate: started with the run, its latest pose read on every step |
| `OmniverseScene(api_url).prim(path, relative_to=None, max_age_s=1.0)` | a prim of an Isaac Sim scene, through the Wandelbots NOVA extension API |

`OmniverseScene` takes the extension's full base URL, so Isaac Sim can run on another machine:
`OmniverseScene("http://isaac-host:8011/omniservice/api/v2")`. A prim is polled in the background
(`poll_hz`, default 60). With `relative_to`, the pose is expressed in that prim's frame, for
example the robot's base prim. `read()` returns the latest polled pose. When polling keeps
failing and that pose is older than `max_age_s`, `read()` raises `RuntimeError`, which ends the run,
as for a stale camera frame.

A `PoseSource` is any object with these three methods. It does not subclass anything:

```python
class Detector:
    async def start(self) -> None: ...  # called when the run starts: open the camera, etc.
    async def stop(self) -> None: ...  # called when the run ends
    def read(self) -> Pose: ...  # the latest pose, without waiting; called on every step
```

A pose written down in metres and a quaternion `(x, y, z, w)` becomes a NOVA `Pose` like this:

```python
from nova.types import Pose
from scipy.spatial.transform import Rotation


def pose_from(position_m, quaternion_xyzw) -> Pose:
    rotation_vector = Rotation.from_quat(quaternion_xyzw).as_rotvec()
    return Pose((*(1000.0 * v for v in position_m), *rotation_vector))
```

With `orientation_type=OrientationType.QUATERNION` the schema turns it back into a quaternion with
`w ≥ 0`. A quaternion and its negative are the same rotation, but different numbers to a policy.
At a half turn (`w = 0`) the sign follows the rotation vector, so check the policy's observation
against its training data.

### Images

Camera streams run in the Camera App on the NOVA instance. `WebRTCCameras` receives frames from a
running stream:

```python
from novapolicy import Observation, WebRTCCameras

cameras = WebRTCCameras(api_url="http://<nova-host>:8011/webrtc-streamer", resize=(224, 224))
Observation.image("wrist", source=cameras.device("<device-id>"))
```

`resize=(width, height)` scales every frame on read. `resize_mode="crop"` centre-crops to the
target's aspect ratio first, instead of stretching. `max_age_s=` on `Observation.image` sets the
oldest frame that channel may contribute; by default the executor's `camera_max_age_s` applies.

## IO mappings

By default, `Observation.io(...)` entries are bidirectional: the policy observes and controls
them. The `mapping` converts between hardware values and policy values:

```python
# Policy sees 0.0 (closed) or 100.0 (open)
# Hardware reads/writes True/False on digital_out[0]
Observation.io("gripper", source=mg, io="digital_out[0]", mapping=BoolMapping(on=100.0))
```

For read-only sensors, set `action=False`:

```python
Observation.io("sensor", source=mg, io="digital_in[0]", action=False)
```

If observation and action need different hardware keys, declare the action with `Action.io()`:

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

NOVA uses radians and millimetres. A dataset may not. A policy trained in other units still runs,
but it moves the arm to the wrong place. Declare the conversion once and it applies in both
directions: forward on the observation, inverted on the action.

```python
from novapolicy import Clamp, Observation, Rad2Deg, Scale

Observation.joint_positions("arm", source=mg, ops=[Rad2Deg()])

# Position is millimetres and orientation radians, so they convert separately.
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
| `Scale(factor)` | bijective | `Scale(0.001)` is mm to m. A zero or non-finite factor is rejected. |
| `Clamp(low, high)` | bidirectional | Bounds the observation, and bounds the command on the way back. |

Operators run front to back on the observation and back to front on the action, so
`[Scale(0.001), Clamp(-1, 1)]` clamps in metres in both directions. They are element-wise. A
forward-only operator is rejected on a channel the policy writes. Operators do not apply to
`computed` or `constant` observations.

The executor inverts the operators before relative targets resolve, so a relative step in degrees
is converted to radians before it is added to the robot's position.

Operators are not normalization. A policy server applies the checkpoint's own normalization, so
do not add it here. Use operators for units and scale only.

Operators and IO mappings do different jobs. A `Mapping` converts between hardware types (bools,
analogue levels) and the policy's floats. An operator (`ValueOp`) converts float units.

## Rotation representation and coordinate systems

NOVA reports TCP poses as `[x, y, z, rx, ry, rz]` (mm, rotation vector) in the controller's world.
A policy trained on another rotation representation or another frame declares it on the TCP
entry. The conversion is then the same for every policy client, on the observation and on the
action:

```python
from novapolicy import Action, Observation, OrientationType

Observation.tcp(
    "eef",
    source=mg,
    orientation_type=OrientationType.QUATERNION,  # eef_qx, eef_qy, eef_qz, eef_qw
    coordinate_system="table",  # a coordinate system of the controller
)
Action.tcp("eef", target=mg, orientation_type=OrientationType.QUATERNION, coordinate_system="table")
```

`orientation_type` takes any of NOVA's `OrientationType` values: rotation vector, quaternion
`[x, y, z, w]` or one of the intrinsic or extrinsic Euler conventions (named `_e1 .. _e3`).
Operators on quaternion components are rejected. `coordinate_system` expresses poses, and relative
offsets, in that coordinate system. The executor fetches the coordinate systems at the start of a
run; for a dry run call `await schema.load_coordinate_systems()`.

## Relative actions

Joint and TCP observations support `mode="relative"`. The mode controls how the policy's action
output is used:

| Mode | Policy returns | Executor sends |
| --- | --- | --- |
| `"absolute"` (default) | target positions | as returned |
| `"relative"` | offsets from the current position | `current + offset` |

```python
Observation.joint_positions("arm", source=mg, mode="relative")
```

Relative joint steps accumulate. Relative TCP steps are offsets from the pose at inference time.
Rotation offsets turn the tool about the TCP; they are composed as rotations, not added as
vectors. Two options on `Action.tcp` cover per-step end-effector deltas, as policies trained with a
differential-IK controller output them:

```python
Action.tcp(
    "eef_delta",
    target=mg,
    mode="relative",
    accumulate=True,  # each step applies to the pose the previous one reached
    use_tool_coordinate_system=False,  # True: offsets in the tool coordinate system
    position_ops=[Scale(0.001)],  # the policy's positions are in metres
)
```

`accumulate` and `use_tool_coordinate_system` apply to `mode="relative"` only. With an absolute
TCP action they raise `ValueError`.

If the training controller realized only part of each commanded step, add a `Scale` that converts
commanded to realized motion. For example, `position_ops=[Scale(0.001), Scale(1 / 0.24)]` plays
24 % of each commanded position step, and `orientation_ops=[Scale(1 / 0.3)]` 30 % of each
rotation.

## TCP actions

For policies that output Cartesian targets instead of joint positions, set `action=True` on
`Observation.tcp()`. The executor sends `POSE` waypoints for that motion group, and NOVA solves
the inverse kinematics:

```python
Observation.tcp("eef_pose", source=mg, tcp="Flange", action=True)
```

The policy receives `eef_pose_x`, `eef_pose_y`, `eef_pose_z`, `eef_pose_rx`, `eef_pose_ry` and
`eef_pose_rz` in mm and radians, and returns targets in the same format. With `orientation_type`,
`coordinate_system` or operators on the entry, it receives and returns them converted, and the
executor converts the targets back. Combine with `mode="relative"` for delta-based Cartesian
control.

`Observation.joint_positions` is writable by default. To observe a motion group's joints and
command its TCP, set `action=False` on the joint observation.

## Actions

Most actions come from their observation: joint and IO observations are writable by default, and
`Observation.tcp` with `action=True`. Declare an action explicitly when its key, mode or hardware
key differs from the observation's, or when nothing observes it:

| Action | Commands | Details |
| --- | --- | --- |
| `Action.joint_positions(key, target=mg, mode=..., ops=...)` | joint targets of one or more motion groups | its `ops` win over the observation's; [Relative actions](#relative-actions) |
| `Action.tcp(key, target=mg, mode=..., accumulate=..., use_tool_coordinate_system=...)` | TCP targets, sent as `POSE` waypoints | [Relative actions](#relative-actions), [TCP actions](#tcp-actions) |
| `Action.io(key, target=mg, io=..., mapping=...)` | an IO write | [IO mappings](#io-mappings) |
| `Action.computed(fn, keys=[...])` | an async side effect with the returned chunk | [Computed actions](#computed-observations-and-actions) |

## Computed observations and actions

For external data sources (OPC UA, PLC, databases) that the built-in types do not cover:

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

Computed actions run after each policy call and receive the returned `ActionChunk`:

```python
async def journal(chunk: ActionChunk) -> None:
    await db.write(chunk.joints)


schema = PolicySchema(
    observations=[Observation.joint_positions("arm", source=mg)],
    actions=[Action.computed(journal)],
)
```

An actuator outside NOVA, such as a gripper on its own bus, uses policy outputs that drive nothing
in NOVA. Declare their keys on both sides. Every policy client then fills them the same way:

```python
schema = PolicySchema(
    observations=[
        Observation.joint_positions("arm", source=mg),
        Observation.computed(gripper.observe, keys=["gripper"]),  # returns {"gripper": ...}
    ],
    actions=[Action.computed(gripper.act, keys=["gripper"])],  # reads chunk.values["gripper"]
)
```

[`examples/computed/`](../examples/computed/) runs both hooks on a virtual robot.

## Troubleshooting

| Symptom | What to do |
| --- | --- |
| The arm moves to the wrong place, but the policy runs | Check units. Compare one observation with the training data, and add `ops` (`Rad2Deg`, `Scale`). |
| Orientation is off, positions are right | Check `orientation_type` and the quaternion order. Check the quaternion sign near a half turn. |
| `ValueError` for `accumulate` or `use_tool_coordinate_system` | Use them with `mode="relative"` only. |
| An operator is rejected on a quaternion | Declare the representation with `orientation_type`; operators are element-wise. |
| A forward-only operator is rejected | The channel is also an action. Set `action=False`, or use an operator that works both ways. |
| Joint velocities look noisy | The controller sends no measured velocities, so they are differenced from positions. |
| Outside a run, `build_observation` fails with "not started" | Call `await schema.start_sources()` first and `await schema.stop_sources()` after. |
| A policy client reports a missing computed key | Pass `keys=[...]` to `Observation.computed` or `Action.computed`. |

## Rerun visualization

See [rerun.md](rerun.md) for real-time 3D visualization of a run.
