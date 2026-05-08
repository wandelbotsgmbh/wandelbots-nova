# Proposal: Policy Schema API

> **Status: IMPLEMENTED** — This proposal has been implemented as `PolicySchema` in `policy/schema.py`.
> The old `FeatureMap`/`FeatureGroup`/`CameraSet` API described below has been removed.
> This document is kept for historical context.

## Problem

The current public API exposes implementation-oriented concepts (`FeatureMap`, `FeatureGroup`, `CameraSet`) that are hard to explain to users. Users usually start from a trained policy or dataset and know the names the policy expects:

- `left_joint_positions`
- `left_eef_9d`
- `left_gripper`
- `observation.images.context_camera`
- `action`

What they need to configure is not a "feature group". They need to declare:

1. which observation keys the policy receives,
2. where each observation value comes from,
3. how controllable observation keys map back to robot actions,
4. which extra action keys the policy outputs (mainly IOs or policies whose action keys differ from observation keys),
5. how values are converted between hardware representation and policy representation.

The high-level concept should be the policy **schema**: observations first, with actions inferred where possible and declared explicitly only where needed.

## Proposed user-facing model

Introduce a `PolicySchema` centered around explicit `Observation` entries.

For joints, the common case should not require a separate `Action.joint_positions(...)`: if a policy observes `Observation.joint_positions("left_joint_positions", source=mg_left)`, the executor can treat `left_joint_positions` in the policy output as the target for `mg_left` by default. This is what users expect for most policies.

Explicit `Action` entries are only needed when the default mirroring is not enough:

- IO writes (`digital_out[0]`, analog outputs, string outputs), because the executor must know the hardware IO and conversion mapping.
- Different observation/action names, e.g. dataset-style `observation.state` → `action`.
- Concatenated actions targeting multiple motion groups.
- Future action-specific semantics such as relative actions or action scaling.

```python
schema = PolicySchema(
    observations=[
        Observation.joint_positions("left_joint_positions", source=mg_left),
        Observation.joint_positions("right_joint_positions", source=mg_right),

        Observation.tcp(
            "left_eef_9d",
            source=mg_left,
            tcp="Flange",
            format=TcpFormat.ROT6D,
        ),

        Observation.io(
            "left_gripper_state",
            source=mg_left,
            io="digital_out[0]",
            mapping=BoolMapping(false_value=0.0, true_value=100.0, threshold=50.0),
        ),

        Observation.image("context_camera", source=context_camera),
        Observation.image("left_wrist_camera", source=left_wrist_camera),
    ],
    actions=[
        # Joint actions are inferred from Observation.joint_positions(...) because the
        # policy outputs the same keys: left_joint_positions/right_joint_positions.
        # Only IO writes need to be declared explicitly here.
        Action.io(
            "left_gripper",
            target=mg_left,
            io="digital_out[0]",
            mapping=BoolMapping(false_value=0.0, true_value=100.0, threshold=50.0),
        ),
    ],
)

executor = PolicyExecutor(
    schema=schema,
    policy=policy_client,
    timeout_s=30,
)
```

Each observation line reads as:

> The policy key `<name>` comes from this NOVA source.

For controllable joint observations, the executor can also use the same key in the policy output as the action target. Explicit `Action` entries are for cases where the executor cannot infer the write target safely.

The first argument is always the exact policy key. No implicit TCP key derivation, no hidden `{name}_tcp`, no `FeatureGroup` vocabulary.

## Camera configuration

A flat list of image observations is clear, but repeated WebRTC settings would be annoying. Shared camera configuration should be handled by a source factory, not by a separate high-level schema concept.

```python
webrtc = WebRTCCameras(
    api_url=CAMERA_SERVER,
    width=224,
    height=224,
    fps=15,
)

schema = PolicySchema(
    observations=[
        Observation.image("context_camera", source=webrtc.device(CAM_CONTEXT)),
        Observation.image("left_wrist_camera", source=webrtc.device(CAM_LEFT_WRIST)),
        Observation.image("right_wrist_camera", source=webrtc.device(CAM_RIGHT_WRIST)),
    ],
    actions=[],  # no joint/IO actions in this camera-only snippet
)
```

This keeps `Observation` as the user-facing concept while avoiding repeated camera resolution/fps/server settings.

The executor should continue to depend on a generic `CameraSource` protocol. WebRTC is the current implementation, not the conceptual camera API.

`PolicyExecutor` owns camera lifecycle: connect all unique camera sources before jogging starts, read frames during the policy loop, and disconnect on cleanup. This keeps camera ownership out of policy clients.

## IO value mappings

IOs require explicit value mapping. Hardware values and policy values are not always the same:

- hardware `bool` ↔ policy `0.0 / 1.0`
- hardware `bool` ↔ policy `0.0 / 100.0`
- hardware analog value ↔ normalized policy float
- hardware string/int values ↔ policy vocabulary

Proposed mapping primitives:

```python
IdentityMapping()
BoolMapping(false_value=0.0, true_value=1.0, threshold=0.5)
BoolMapping(false_value=0.0, true_value=100.0, threshold=50.0)
ScaleMapping(hardware_min=0.0, hardware_max=10.0, policy_min=0.0, policy_max=1.0)
EnumMapping({"open": True, "closed": False})
```

The same mapping can be used in both directions:

- observation: hardware value → policy value
- action: policy value → hardware value

Although IOs need this most often, mappings should not be limited to IOs. The same concept is useful for any value whose policy representation differs from NOVA's representation, for example normalized joint positions, degrees instead of radians, or normalized analog signals.

Examples:

```python
Observation.io(
    "pressure",
    source=mg,
    io="analog_in[0]",
    mapping=ScaleMapping(hardware_min=0.0, hardware_max=10.0, policy_min=0.0, policy_max=1.0),
)

Action.io(
    "gripper",
    target=mg,
    io="digital_out[0]",
    mapping=BoolMapping(false_value=0.0, true_value=100.0, threshold=50.0),
)
```

Do not use `action=True` on observations. Observations and actions are separate concepts.

## TCP observations

TCP mapping must make the policy key explicit.

Current API risk:

```python
FeatureGroup(motion_group=mg, name="arm", tcp="Flange")
```

This looks like it should include TCP, but if `tcp_format` is not set, TCP is silently omitted.

Proposed API:

```python
Observation.tcp(
    "eef_9d",
    source=mg,
    tcp="Flange",
    format=TcpFormat.ROT6D,
)
```

This is explicit:

- policy receives key `eef_9d`,
- value comes from motion group `mg`,
- physical TCP is `Flange`,
- representation is ROT6D.

For flat-feature clients, the adapter may expand this to `eef_9d_1`, ..., `eef_9d_9`. The schema key is still explicit.

## Joints and action shape

Policies differ in how they name joint arrays:

- GR00T-style: `left_joint_positions` as an array
- LeRobot flat style: `left_joint_positions_1`, ..., `left_joint_positions_6`
- dataset style: `observation.state` / `action` as concatenated arrays

The schema should describe the logical policy key. The policy client decides how to serialize it for its transport/model convention.

Examples:

```python
# Separate per-arm arrays: actions are inferred from the joint observations.
Observation.joint_positions("left_joint_positions", source=mg_left)
Observation.joint_positions("right_joint_positions", source=mg_right)

# Concatenated dataset-style vector: explicit action needed because the policy
# observes "observation.state" but outputs "action". Disable mirrored action
# inference for the observation key.
Observation.joint_positions("observation.state", source=[mg_left, mg_right], writable=False)
Action.joint_positions("action", target=[mg_left, mg_right])
```

Decision: support concatenated multi-motion-group observations/actions in the first version because real datasets often use single keys such as `observation.state` and `action`.

## Static and task observations

Some policy inputs are not live robot state, images, or IOs. GR00T-style policies often expect a language instruction key such as `annotation.language.language_instruction`. Datasets may also contain task IDs or subtask IDs.

Support constant observations for this:

```python
Observation.constant(
    "annotation.language.language_instruction",
    value="Pick up the box and place it onto the conveyor.",
)
Observation.constant("task_index", value=0)
```

These values are included in every observation and never appear in actions.

## Action semantics

Joint position actions should default to absolute joint targets in radians, because that is what the PID jogging runner currently consumes.

Some policies, especially GR00T configurations, output relative deltas. This should be explicit on the action mapping, not hidden in a client:

```python
Action.joint_positions(
    "left_joint_delta",
    target=mg_left,
    mode="relative",
)
```

Initial scope:

- `mode="absolute"` for inferred and explicit joint-position actions.
- `mode="relative"` supported by adding the current observed joint positions before sending to the runner.
- TCP actions are out of scope for the first schema version. TCP can be observed, but policy execution still commands joint targets and IOs.

## Units and value conventions

The schema must document default units clearly:

| Value | Default policy representation |
| --- | --- |
| joint positions | radians |
| joint torques | controller-provided units |
| joint currents | controller-provided units |
| TCP/flange position | meters |
| TCP/flange rotation vector | radians |
| images | RGB `uint8` arrays |
| bool IOs | defined by the configured mapping |

If a policy was trained with different units or normalized values, use a mapping on the observation/action entry.

## Schema validation

`PolicySchema` should validate eagerly at construction time:

- no duplicate observation keys,
- no duplicate action keys,
- action keys must map to writable targets,
- inferred writable joint keys must not conflict with explicit `Action` entries,
- concatenated sources/targets must have deterministic order,
- IO actions must target writable IO hardware keys,
- unsupported optional state fields must either have a default or raise a clear error.

Missing optional NOVA fields such as `joint_torque` or `joint_current` should not silently become zeros unless the user explicitly configured a default:

```python
Observation.joint_torques("left_joint_torques", source=mg_left, default=[0.0] * 6)
```

Without a default, missing required observations should raise a clear runtime error so the policy does not run with corrupted inputs.

## Implementation note: raw state access

`RobotState` should expose commonly useful numeric state directly: pose, TCP name, joint positions, joint torques, and joint currents. To support less common status/metadata fields such as `joint_limit_reached`, `standstill`, `execute.details.state.kind`, `timestamp`, and `sequence_number` through `Observation.state_field(...)`, the schema implementation must still keep access to the raw NOVA `MotionGroupState` from the state stream or enrich the internal state object further.

This is an implementation requirement, not a user-facing concept.

## Transport guidance

The schema is transport-independent. Built-in clients are adapters:

- bare async function / `CallbackPolicyClient`: bring your own transport,
- `NatsPolicyClient`: convenient for Nova app-to-app scalar policies,
- `Gr00tPolicyClient`: ZMQ/msgpack/NumPy adapter for GR00T servers.

NATS is not ideal for image-heavy policies because of the platform payload limit. It can work by sending images separately, but production multimodal policies should generally use ZMQ or a custom client/transport.

## Migration direction

Current public concepts:

```python
FeatureMap(groups=[FeatureGroup(...)])
CameraSet(...)
```

Proposed public concepts:

```python
PolicySchema(
    observations=[Observation...],
    actions=[Action...],  # optional for mirrored joint actions
)
```

Internally we can still compile `PolicySchema` into the existing structures used by the executor, IO streams, and clients. The change is mainly about the public API and mental model.

## First implementation scope

Implement first:

- `PolicySchema`, `Observation`, `Action` public API.
- `Observation.joint_positions`, `Observation.tcp`, `Observation.flange`, `Observation.io`, `Observation.image`, `Observation.constant`, `Observation.state_field`.
- `Observation.joint_torques` and `Observation.joint_currents` backed by optional `RobotState.joint_torques` / `RobotState.joint_currents` fields.
- `Action.joint_positions` and `Action.io`.
- Inferred absolute joint actions for writable `Observation.joint_positions`.
- Explicit `writable=False` for observation-only joint keys.
- `IdentityMapping`, `BoolMapping`, `ScaleMapping`, `EnumMapping`.

Defer:

- TCP actions.
- Derived joint velocities.
- Dedicated helpers for status/metadata fields.
- More specialized camera transports beyond the `CameraSource` protocol.

## Benefits

- Matches how datasets and policies are described: observation keys and action keys.
- Makes TCP policy key explicit.
- Avoids confusing `FeatureMap` / `FeatureGroup` terminology.
- Keeps the common joint case concise by inferring mirrored joint actions.
- Separates observations from non-obvious actions such as IO writes.
- Makes IO value conversion explicit and reusable.
- Keeps camera transport pluggable through `CameraSource`.
- Lets policy clients remain simple adapters from schema → transport-specific wire format.

## Additional NOVA state observations

Yes, the schema should support more than joint positions, TCP poses, IOs, and images. NOVA's motion-group state stream exposes additional values that may be useful for learned policies or diagnostics.

The previous `Observation.joints(...)` name would be too broad once we add more joint-related values. Prefer explicit constructors:

```python
Observation.joint_positions("left_joint_positions", source=mg_left)
Observation.joint_torques("left_joint_torques", source=mg_left)
Observation.joint_currents("left_joint_currents", source=mg_left)
Observation.tcp("left_eef_9d", source=mg_left, tcp="Flange", format=TcpFormat.ROT6D)
Observation.flange("left_flange_pose", source=mg_left, format=TcpFormat.ROTATION_VECTOR)
```

NOVA currently provides these relevant motion-group state fields:

| NOVA state field | Proposed observation helper | Notes |
| --- | --- | --- |
| `joint_position` | `Observation.joint_positions(...)` | Main controllable joint observation. Writable by default. |
| `joint_torque` | `Observation.joint_torques(...)` | Optional; omit or default if controller does not provide it. Observation-only. |
| `joint_current` | `Observation.joint_currents(...)` | Optional; useful for contact/load-related policies. Observation-only. |
| `tcp_pose` | `Observation.tcp(...)` | TCP pose for a selected physical TCP. Observation-only. |
| `flange_pose` | `Observation.flange(...)` | Flange pose independent of selected tool. Observation-only. |
| `joint_limit_reached` | `Observation.state_field(...)` | Per-joint limit flags/status. Rare policy input; use generic field mapping if needed. |
| `standstill` | `Observation.state_field(...)` | Boolean motion status. Rare policy input; usually handled internally by safety logic. |
| `execute.details.state.kind` | `Observation.state_field(...)` | Status/debug signal, e.g. paused near collision/limit/singularity. Usually handled internally. |
| `timestamp`, `sequence_number` | `Observation.state_field(...)` | Metadata; only expose through generic field mapping if a policy explicitly needs it. |

Joint velocities are **not** currently a first-class value in `MotionGroupState`. If needed, we can add derived velocities later:

```python
Observation.joint_velocities("left_joint_velocities", source=mg_left, derived=True)
```

But derived velocities should be clearly marked as derived, because they depend on stream rate and timestamp quality.

For less common fields, add a generic escape hatch instead of creating helpers for everything:

```python
Observation.state_field("left_payload", source=mg_left, field="payload")
Observation.state_field("left_coordinate_system", source=mg_left, field="coordinate_system")
```

Rule of thumb:

- common numeric arrays get named helpers (`joint_positions`, `joint_torques`, `joint_currents`),
- pose values get explicit helpers (`tcp`, `flange`),
- status/metadata values use `state_field(...)` instead of dedicated helpers.
