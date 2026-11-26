# Gr00tPolicyClient

`Gr00tPolicyClient` runs a policy served by an [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T)
inference server with the NOVA `PolicyExecutor`.

It speaks the same ZMQ REQ/REP msgpack protocol as `gr00t.policy.server_client.PolicyServer`, so
it replaces `gr00t.policy.server_client.PolicyClient`. The wire format, serialization and array
shapes match Isaac GR00T N1.7 (`novapolicy.providers.gr00t.GROOT_PROTOCOL_VERSION`). N1 and N1.5
servers (`gr00t/eval/service.py`) take a flat observation and are not supported.

Install the client with `uv sync --extra novapolicy-gr00t`.

## Usage

```python
from novapolicy import (
    BoolMapping,
    Gr00tPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
)

schema = PolicySchema(
    observations=[
        Observation.joint_positions("left_arm", source=mg_left),
        # TCP state goes to GR00T as metres + rot6d by default (the client's tcp_format)
        Observation.tcp("left_eef_9d", source=mg_left),
        Observation.joint_positions("right_arm", source=mg_right),
        Observation.tcp("right_eef_9d", source=mg_right),
        Observation.io(
            "left_gripper", source=mg_left, io="digital_out[0]", mapping=BoolMapping(on=100.0)
        ),
        Observation.constant("language", value="Pick up the box."),
    ]
)

client = Gr00tPolicyClient(host="gpu-server", port=5555, control_freq=15)

executor = PolicyExecutor(
    schema,
    client,
    execution=SequentialExecution(),  # continuous: see step 5 below
    n_action_steps=8,
)
result = await executor.run()
```

The client builds GR00T's numpy array observation from the `PolicySchema` and decodes the returned
action arrays. An action key that the server leaves out (one the policy was not trained on) is
skipped. A key that is present without the expected array raises an error.

## Setting up a policy on a real robot

How to take a trained GR00T checkpoint behind a running inference server to a supervised run on a
real robot. A deployment is a plain `PolicySchema`, a `Gr00tPolicyClient` and a `PolicyExecutor`.
[`examples/execute_gr00t_single_arm_pick_and_place`](../examples/execute_gr00t_single_arm_pick_and_place/)
is the worked example: an IsaacLab-trained cube pick-and-place on a UR10e with a Zimmer gripper.

### 1. Reach the server

The GR00T inference server speaks ZMQ (port 5555 in the container, often a NodePort outside it).
`await Gr00tPolicyClient(host, port=...).get_server_info()` lists the state, action, video and
language keys and the action horizon; see [Inspecting a server](#inspecting-a-server). The client
takes the language key and the state keys from the server itself.

### 2. Know what the policy's actions mean

The server tells the keys, not their meaning. From the training setup, find out:

| Question | Where it goes |
|---|---|
| Joint targets, absolute TCP targets, or TCP deltas? | `Action.joint_positions`, `Action.tcp(mode="absolute")`, or `Action.tcp(mode="relative")` |
| Deltas per step, applied to the pose the previous step reached? | `accumulate=True` |
| Deltas in the world or in the tool coordinate system? | `use_tool_coordinate_system=False` / `True`, as in NOVA's TCP jogging |
| Poses in another coordinate system, rotations as quaternions or Euler angles? | `coordinate_system=`, `orientation_type=OrientationType...` |
| Units (metres, degrees)? | `position_ops=[Scale(0.001)]`, `ops=[Rad2Deg()]` |
| Which point on the tool (TCP) do the poses refer to? | `Observation.tcp(..., tcp=...)` |
| Rate, camera resolution, prompt | `Gr00tPolicyClient(control_freq=...)`, `WebRTCCameras(resize=..., resize_mode="crop")`, `Observation.constant("language", ...)` |
| How is the gripper driven, and what is its state? | `Action.io`, or `Action.computed` / `Observation.computed` with `keys=` for one on its own bus |

The training config usually states these. A policy trained with a differential-IK controller
(IsaacLab's relative pose command) outputs commands, and the simulated robot realized only part of
each one. An extra `Scale` converts commanded to realized motion; see
[Relative actions](schema.md#relative-actions).

One training episode tells most of this. Run its recorded joints through the robot's forward
kinematics and compare the path with the recorded actions. That shows whether the actions are
joint or TCP targets, absolute or deltas, their coordinate system, the scale (units and training
gain), the gripper polarity and values, and the home joints. The pick-and-place example keeps the
result in
[`episode_settings.json`](../examples/execute_gr00t_single_arm_pick_and_place/episode_settings.json),
loaded by its `episode_settings.py`, so none of it is typed into the run script.

The TCP must be known. Rotations pivot about it, so a TCP that is d mm off shifts the position by
about d × angle while the tool rotates.

Pass the checkpoint's `statistics.json` as `statistics=`. The first observation is then held
against the training range, and a degrees-for-radians mix-up stops before the robot moves. The
server does not serve the file, so the client needs a local copy.

### 3. Check the cell against the training scene

If the policy was trained in simulation, replay one training episode's recorded joint path on the
real robot before running the policy (a `jog_joints` chunk of the episode's joints at its rate). A
replay that picks and places shows that the robot frame, table and target positions match. It
does not check the TCP: joints replay the same path whatever the TCP is.

### 4. Write the run script

```python
from novapolicy import (
    Action,
    Gr00tPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
    WebRTCCameras,
)
from episode_settings import EpisodeSettings  # the pick-and-place example's loader

settings = EpisodeSettings.load("episode_settings.json")
arm = settings.robots["arm"]
mg = (await nova.cell().controller(controller)).motion_group(arm.motion_group)

client = Gr00tPolicyClient(
    host, port=port, control_freq=settings.rate_hz, statistics="checkpoint/statistics.json"
)
webrtc = WebRTCCameras(api_url=camera_server, resize=(320, 240), resize_mode="crop")
schema = PolicySchema(
    observations=[
        Observation.joint_positions(arm.joints_key, source=mg, action=False),
        Observation.tcp("tcp_pose", source=mg, tcp=arm.tcp),  # the TCP relative actions use
        Observation.computed(gripper.observe, keys=[arm.gripper_state_key]),
        *(Observation.image(key, source=webrtc.device(serial)) for key, serial in cameras.items()),
        Observation.constant("language", value=prompt),
    ],
    actions=[
        arm.action(mg),  # the action as read from the episode: kind, units and gains
        Action.computed(gripper.act, keys=[arm.gripper_action_key]),
    ],
)
executor = PolicyExecutor(
    schema,
    client,
    execution=SequentialExecution(),
    n_action_steps=8,
    timeout_s=300,
)
result = await executor.run()
```

A gripper on its own bus enters through `Observation.computed` (its state, with `keys=` so the
schema check counts it) and `Action.computed` (its command). With `keys=[...]`, every policy
client puts those action outputs into `chunk.values[key]`. The example's `ZimmerGripper` is a
complete one. It maps the jaw gap onto the training state values and switches at the step of the
chunk that asks for it. Its `HoldWhileGripping` keeps the arm still while the jaws move.

### 5. Choose the execution mode

- **Sequential** (start here): run `n_action_steps` of each chunk to standstill, observe, infer
  again. Every chunk is checked on its own, and the arm stops between chunks.
- **Continuous:** infer again as fast as the server answers. Wrap the client in
  `QueuedPolicyClient(client)` and use `ContinuousExecution()` with `n_action_steps=0`. The queue
  keeps the steps that ran during inference, blends the overlap and smooths it. Without the queue,
  every new chunk makes the arm jump.

### 6. Run

- Limits for moves NOVA plans (such as the move home) are `MotionSettings`. Streamed policy steps
  run within the controller's limits and safety zones.
- Dry run first: one real observation and one inference, the targets printed, nothing
  commanded. `resolve_actions(schema, chunk, states)` gives the targets the executor would send.
- Then a supervised run with a hand on the e-stop. Before moving home, move straight up when the
  tool is low, so the joint move cannot sweep it over the table.

### Things to keep in mind

- **Rotation vectors:** near a half turn the same orientation reads as `+v` or `-v`. Relative TCP
  actions and `QueuedPolicyClient` unwrap them. Do the same
  (`novapolicy.mapping.poses.unwrap_tcp_rotations`) before you interpolate rotation vectors
  yourself.
- **Commanded vs. realized actions:** how much of a command the training controller realized can
  only be approximated on another robot. Policies trained on the motion that actually happened
  (next joint positions or poses) avoid this.
- **Sim-to-real:** a policy trained on rendered images reads real camera images less precisely.
  Once a replay confirms the cell, remaining placement errors come from the policy, not from the
  deployment settings.

## Reference

### Gr00tPolicyClient

`Gr00tPolicyClient(host, *, port=5555, timeout_ms=15000, api_token=None, control_freq=None,
tcp_format=TcpFormat.ROT6D, statistics=None, embodiment=None)`

| Argument | Meaning |
| --- | --- |
| `host` | host name of the GR00T ZMQ server |
| `port` | ZMQ port |
| `timeout_ms` | ZMQ send and receive timeout in ms |
| `api_token` | API token for a server that requires one |
| `control_freq` | rate of the policy's action steps in Hz (the dataset's frame rate); default 30. A `dt_ms` in the server's action info wins. |
| `tcp_format` | TCP format sent unless the `Observation.tcp` declares an `orientation_type`; `TcpFormat` is in `novapolicy.providers.gr00t.eef` |
| `statistics` | path to the checkpoint's `statistics.json`; the first observation is checked against it |
| `embodiment` | embodiment tag in `statistics.json`; default: the one whose state keys are exactly the server's |

### Wire protocol

The client uses the [GR00T REQ/REP msgpack protocol](https://github.com/NVIDIA/Isaac-GR00T/blob/main/gr00t/policy/server_client.py):

- **Endpoints**: the client calls `ping`, `get_modality_config` and `get_action`; the server also
  serves `reset`.
- **Observations**: numpy arrays serialized as `.npy` bytes inside msgpack.
- **Actions**: returned as an `(action_dict, info_dict)` tuple.

### Observation keys

The `key` argument in each `Observation.*()` call becomes the GR00T state key, each value a
float32 array of shape `(1, 1, D)`:

```python
Observation.joint_positions("left_arm", source=mg)  # → obs["state"]["left_arm"]
Observation.tcp("left_eef_9d", source=mg)  # → obs["state"]["left_eef_9d"] (metres + rot6d)
Observation.io("left_gripper", source=mg, io="digital_out[0]")  # → obs["state"]["left_gripper"]
Observation.image("wrist", source=camera)  # → obs["video"]["wrist"], (1, 1, H, W, 3)
```

A TCP goes as metres + rot6d (the client's `tcp_format`) unless the `Observation.tcp` declares an
`orientation_type`. A TCP observation with a `coordinate_system` must declare an
`orientation_type`; `validate_schema` checks this.

`validate_schema` (the executor calls it before the run) reads the server's modality config. From
then on the client sends only the state keys the server expects. An observation the model does not
take, such as a TCP pose that only sets the TCP for relative actions, stays out of the request. The
`Observation.constant("language", ...)` prompt goes under the server's language key.

### Computed observations and actions

`Action.computed` runs for every policy; the executor runs it after each call. For an actuator
outside NOVA, declare the action keys it uses, `Action.computed(fn, keys=["gripper"])`. The client
puts their values into `chunk.values["gripper"]`, one row per step.

`Observation.computed` also runs for GR00T. The hook gets the flat observation, as with every
client, and its values are merged into GR00T's numeric state payload. Each computed value must be
a scalar or a sequence of floats keyed to a model state input. Pass `keys=[...]` so
`validate_schema` counts them. Non-numeric outputs (strings, images) work only with
`CallbackPolicyClient`.

## Inspecting a server

Before writing your schema, query the server to see what it expects:

```python
import asyncio

from novapolicy import Gr00tPolicyClient


async def main():
    client = Gr00tPolicyClient(host="gpu-server", port=5555)
    await client.connect([])

    info = await client.get_server_info()
    print(info)
    # {
    #   'state_keys': ['left_joint_positions', 'right_joint_positions'],
    #   'action_keys': ['left_joint_positions', 'right_joint_positions'],
    #   'video_keys': ['exterior_image_1', 'exterior_image_2', 'left_wrist_image', 'right_wrist_image'],
    #   'language_keys': ['annotation.language.language_instruction'],
    #   'action_horizon': 16,
    #   'action_configs': [
    #     {'rep': 'RELATIVE', 'type': 'NON_EEF', 'format': 'DEFAULT', 'state_key': 'left_joint_positions'},
    #     {'rep': 'RELATIVE', 'type': 'NON_EEF', 'format': 'DEFAULT', 'state_key': 'right_joint_positions'},
    #   ],
    # }

    await client.close()


asyncio.run(main())
```

Use it to:

- match your `Observation.*()` keys to `state_keys` and `video_keys`;
- confirm the `action_horizon` (chunk size);
- check `action_configs` for `RELATIVE` or `ABSOLUTE` actions.

The server's modality config does not report the step timing. Set it on the client to match the
training data rate: for data recorded at 15 Hz, `control_freq=15`. A `dt_ms` in the server's action
info, where a server adds one, wins. The default is 30 Hz.

```python
client = Gr00tPolicyClient(host="gpu-server", control_freq=15)
```

## Diagnosing chunk timing

To diagnose timing problems (snap-back, overshoot, wrong playback speed), wrap the client's
`get_actions` to log per-step values:

```python
import math, time

_orig_get = client.get_actions
_step_count = 0
_last_time = time.monotonic()


async def _logged_get(states, schema, images, io_values):
    global _step_count, _last_time
    _step_count += 1
    gap = time.monotonic() - _last_time

    t0 = time.monotonic()
    result = await _orig_get(states, schema, images, io_values)
    inference_ms = (time.monotonic() - t0) * 1000
    _last_time = time.monotonic()

    if hasattr(result, "joints") and result.joints:
        skip = inference_ms / result.dt_ms if result.dt_ms > 0 else 0
        n = len(next(iter(result.joints.values())))
        print(
            f"Step {_step_count} | gap={gap:.2f}s | inference={inference_ms:.0f}ms | skip~{skip:.1f}/{n}"
        )

        for mg_id, steps in result.joints.items():
            obs = states.get(mg_id)
            if obs and steps:
                current = list(obs.joints)
                snap = max(abs(math.degrees(steps[0][j] - current[j])) for j in range(len(current)))
                eff_idx = min(int(skip), len(steps) - 1)
                eff_delta = max(
                    abs(math.degrees(steps[eff_idx][j] - current[j])) for j in range(len(current))
                )
                print(f"  {mg_id}: snap={snap:.2f}°, effective[{eff_idx}] delta={eff_delta:.2f}°")

    return result


client.get_actions = _logged_get
```

What to watch:

- **snap**: how far step 0 is from the current position. It should be below 1°. Large values
  point to a wrong `control_freq` (`dt_ms`).
- **skip**: steps skipped during inference. It should stay below the action horizon.
- **effective delta**: distance from the current position to the step the robot will actually
  target. It should be positive and growing: the robot moves forward, not back.

## Troubleshooting

| Symptom | What to do |
| --- | --- |
| `TypeError: GR00T get_action response must be a 2-tuple of (action, info)` | The server speaks an older protocol (N1, N1.5). Use an N1.7 server. |
| `Schema does not satisfy GR00T server requirements: Missing state observations` | Add an observation for each listed state key, named exactly like it. |
| `... Missing image observations` | Add an `Observation.image` for each listed video key. |
| `Observation.tcp(...) has a coordinate_system but no orientation_type` | Add `orientation_type=` to that TCP observation. |
| `statistics.json has no embodiment ...` or several embodiments match | Pass `embodiment=` with one of the tags the message lists. |
| `GR00T action '...' must be an array of shape ([batch,] horizon, dof)` | The server answered in another format. Check that server and client protocol versions match. |
| `Observations are implausibly far outside what this checkpoint was trained on` | The observation is far outside the training range, most often a unit mismatch. Add `ops=` to the observation. |
| The arm plays the chunk too fast or too slow, or snaps back | Set `control_freq` to the training data's frame rate. Use [chunk timing](#diagnosing-chunk-timing) to check. |
| The arm jumps between chunks in continuous execution | Wrap the client in `QueuedPolicyClient` and pass `n_action_steps=0`. |
| The tool lands offset while it rotates | The TCP is wrong. Set the TCP the training poses refer to. |
