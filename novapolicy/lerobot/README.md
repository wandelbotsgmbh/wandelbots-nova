# Run a LeRobot policy with NOVA PolicyExecutor

This guide is for users who want to execute a trained LeRobot policy through NOVA.

The setup has two parts:

1. Start LeRobot's async inference server where the policy checkpoint is available.
2. Configure `LeRobotPolicyClient` on the NOVA/robot side and run it with `PolicyExecutor`.

The NOVA side sends observations to the LeRobot server and receives action chunks. Model weights are
not uploaded by the client; `pretrained_name_or_path` is interpreted by the LeRobot server.

## Quickstart

### 1. Install the LeRobot policy extra

Install this in the environment that runs NOVA policy execution:

```bash
uv add wandelbots-nova --extra novapolicy-lerobot
```

If you run the LeRobot server in a separate Python environment, install LeRobot there too. LeRobot
currently requires Python 3.12:

```bash
mamba create -y -n lerobot-server python=3.12 pip
conda activate lerobot-server
python -m pip install --upgrade pip
python -m pip install 'wandelbots-nova[novapolicy-lerobot]'
```

### 2. Set the cell up

The LeRobot examples here run on a UR10e carrying the UMI gripper, whose
`umi_corrected` TCP is the frame the demonstrations were recorded in — not the cell's `Flange`,
221.7 mm further down the tool. Fed the wrong frame a policy does not fail, it stalls in a hover.

```bash
./novapolicy/examples/setup_cell.sh http://<nova-host> umi
```

[`pick_and_place_umi_ur10e.py`](../examples/pick_and_place_umi_ur10e.py) drives the same task by
hand. Run it first: if the scripted version cannot pick the cube up, no checkpoint will.

### 3. Put the checkpoint where the LeRobot server can read it

Use either:

- a server-local checkpoint path, for example `/models/pick_place_act/pretrained_model`, or
- a Hugging Face model id supported by `policy_class.from_pretrained(...)`.

For a server-local checkpoint, copy the whole pretrained model directory to the server host:

```bash
scp -r ./pretrained_model user@gpu-server:/models/my_lerobot_policy
```

The NOVA client will later pass that same server-side path:

```python
pretrained_name_or_path = "/models/my_lerobot_policy"
```

### 4. Start the LeRobot async server

Run the server on the machine that has the checkpoint and should execute model inference. The server
starts without a checkpoint argument; it loads the checkpoint path sent by the NOVA client when that
client connects.

```bash
# Terminal 1 — inference host
conda activate lerobot-server

python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080
```

Keep this process running. A successful startup logs that the gRPC server is listening on port 8080.
The first client connection sends `policy_type`, checkpoint path, action-chunk size, and inference
device. Loading happens while `PolicyExecutor` is in `CONNECTING`, before its execution timeout
starts.

The server also accepts `--fps`, `--inference_latency`, and `--obs_queue_timeout`. They are not
required for the basic setup. If you set server `--fps`, keep it aligned with the client's `fps`.

### 5. Execute the policy through NOVA

Save the following as `run_lerobot_policy.py` on the NOVA client machine. Replace the host names,
controller, camera device, checkpoint paths, and IO key with values for your cell and checkpoint.
Model weights only need to exist on the inference server. The client reads the checkpoint's
`config.json` to derive its settings, and its normalization statistics too when they are there —
see [checkpoint execution settings](#checkpoint-execution-settings).

```python
import asyncio

from nova import Nova
from nova.config import NovaConfig
from novapolicy import (
    BoolMapping,
    LeRobotPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
    WebRTCCameras,
)

NOVA_HOST = "http://<nova-host>"
LEROBOT_SERVER = "<lerobot-server-host>:8080"
SERVER_CHECKPOINT = "/models/my_lerobot_policy"
CLIENT_CHECKPOINT_CONFIG = "./my_lerobot_policy/config.json"
CAMERA_API = "http://<camera-host>:8011/webrtc-streamer"
CAMERA_DEVICE = "<scene-camera-device-id>"
FPS = 15.0
PLAYBACK_SPEED = 1.0


async def main() -> None:
    # Configure the camera stream at the checkpoint's resolution; add
    # resize=(w, h) only if the stream cannot be reconfigured.
    cameras = WebRTCCameras(api_url=CAMERA_API)

    async with Nova(config=NovaConfig(host=NOVA_HOST)) as nova:
        arm = (await nova.cell("cell").controller("cobot"))[0]
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("arm", source=arm),
                Observation.io(
                    "gripper",
                    source=arm,
                    io="digital_out[0]",
                    mapping=BoolMapping(),
                ),
                Observation.image(
                    "cam_scene_1",
                    source=cameras.device(CAMERA_DEVICE),
                ),
            ]
        )
        policy = LeRobotPolicyClient(
            server_address=LEROBOT_SERVER,
            pretrained_name_or_path=SERVER_CHECKPOINT,
            # The server loads SERVER_CHECKPOINT; the client reads this local
            # copy to derive policy_type, chunking, and the camera frame size.
            config_path=CLIENT_CHECKPOINT_CONFIG,
            fps=FPS,
            playback_speed=PLAYBACK_SPEED,
            device="cuda",
        )
        executor = PolicyExecutor(
            schema,
            policy,
            execution=SequentialExecution(),
            timeout_s=80,
        )
        result = await executor.run()
        print(f"Stopped: {result.reason}; steps={result.steps}; duration={result.duration_s:.1f}s")


if __name__ == "__main__":
    asyncio.run(main())
```

Run it while the async server is still listening:

```bash
# Terminal 2 — NOVA client host
python run_lerobot_policy.py
```

This example uses settled ACT execution: infer a chunk, execute the checkpoint-defined action
horizon, reach standstill, and infer again. If the checkpoint has no camera input, remove
`WebRTCCameras` and `Observation.image(...)`. Observation names and ordering must still match the
checkpoint dataset.

### Continuous asynchronous execution

To keep a fixed-rate LeRobot action queue while NOVA continuously replaces the active lookahead,
keep the same schema and use the following policy and executor options:

```python
from novapolicy import ContinuousExecution
from novapolicy.lerobot import AsyncQueueAggregation

policy = LeRobotPolicyClient(
    server_address=LEROBOT_SERVER,
    pretrained_name_or_path=SERVER_CHECKPOINT,
    config_path=CLIENT_CHECKPOINT_CONFIG,
    fps=FPS,
    playback_speed=PLAYBACK_SPEED,
    device="cuda",
    use_async_queue=True,
    async_queue_aggregation=AsyncQueueAggregation.AVERAGE,
    async_queue_smoothing=True,
)
executor = PolicyExecutor(
    schema,
    policy,
    execution=ContinuousExecution(rate_hz=FPS * PLAYBACK_SPEED),
    n_action_steps=0,
    timeout_s=80,
)
result = await executor.run()
```

Use sequential execution first. Enable the continuous queue only when the checkpoint and robot task
have been validated with overlapping action chunks.

## Configuration reference

### `server_address`

Address of the LeRobot async inference server in `"host:port"` form.

Examples:

```python
server_address = "127.0.0.1:8080"
server_address = "gpu-server.internal:8080"
```

### `pretrained_name_or_path`

Checkpoint path or Hugging Face model id passed to the LeRobot server.

Important: for a remote server, this is resolved on the server machine, not on the NOVA client.
The client does not upload model weights.

```python
pretrained_name_or_path = "/models/my_lerobot_policy"
pretrained_name_or_path = "org/my-policy"
```

### `policy_type`

LeRobot policy type sent to the server. Read from the checkpoint's `type`; pass it only to
override, and only with a value that agrees with the checkpoint:

```python
policy_type = "act"
```

The NOVA client-side decoder is policy-architecture agnostic as long as the server returns a flat
action vector matching the schema-derived joint, TCP, and IO layout.

### `fps`

Control/dataset frequency used by the NOVA client to time returned actions:

```python
fps = 15  # ActionChunk.dt_ms = 1000 / 15
```

This should match the policy's intended control rate. This is separate from the LeRobot server's
optional `--fps` CLI flag. The server flag is used by LeRobot when it creates `TimedAction`
timestamps; `LeRobotPolicyClient` does not use those timestamps. It decodes the returned action
tensors into NOVA `ActionChunk`s and sets the chunk `dt_ms` from this client-side `fps`.
`PolicyExecutor` then uses that `dt_ms` when scheduling the returned chunk.

### `playback_speed`

Explicit physical playback speed relative to the dataset rate:

```python
playback_speed = 0.75  # execute 25% slower: 15 Hz dataset actions use 88.89 ms intervals
```

The dataset frequency remains `fps=15`; only the physical `ActionChunk.dt_ms` is scaled:

```text
dt_ms = 1000 / (fps * playback_speed)
```

Keep this at `1.0` for nominal dataset timing. Values below `1.0` are useful when the NOVA
best-effort waypoint tracker follows the learned actions more aggressively than the original data
collection controller.

### Checkpoint execution settings

`LeRobotPolicyClient` reads what the checkpoint already declares instead of asking for it:

| Derived from the checkpoint | Read from |
| --- | --- |
| `policy_type` | `type` |
| `actions_per_chunk` | `len(action_delta_indices)` |
| `n_action_steps` (used by `PolicyExecutor`) | `n_action_steps` |

Values are read through LeRobot's own `PreTrainedConfig`, not by parsing `config.json` field by
field, so **every LeRobot policy works without a per-policy table here**. Policies disagree about
what the action chunk is called — ACT says `chunk_size`, diffusion says `horizon`, fastwam says
`action_horizon`, vqbet names it nothing — but every policy config must implement the abstract
`action_delta_indices` property, and its length is the chunk length for all of them. A new or
renamed LeRobot policy needs no change in novapolicy.

A policy that declares no action chunk at all (`action_delta_indices is None`) needs
`actions_per_chunk` explicitly.

`fps` and `playback_speed` cannot be derived — frame rate is a property of the dataset, not the
policy — so they stay explicit.

Pass any of the derived arguments only to override it. An override that contradicts the checkpoint
is reported rather than silently accepted: a different `policy_type`, or a chunk length above the
checkpoint's action chunk, both raise. A *shorter* horizon is allowed with a warning, since a
receding horizon is a legitimate choice.

The client also holds the schema against the checkpoint's `input_features` / `output_features`
before the setup message goes out: the observation key set, the shape behind each key, and the
width of the flat action vector. A mismatch fails at startup rather than after the robot has
started moving. This runs automatically; there is nothing to call. A checkpoint that declares no
features (they were inferred from the dataset at train time) is warned about and skipped.

One exception: a **camera frame size** that disagrees only warns. See
[camera resolution](#camera-resolution) below.

### Checking the observation against the checkpoint's statistics

Feature shapes prove the schema is the right *width*. They cannot prove it carries the right
*numbers* — `PolicyFeature` has no per-dimension names, so a permuted joint order or a unit
mismatch passes the contract check untouched.

The checkpoint's own training statistics settle it. When `policy_preprocessor.json` and its
safetensors are readable — alongside `config.json`, on the Hub or locally — the client holds the
first live observation against them:

- **An order of magnitude larger** than anything in training raises before the robot moves. That is
  what a unit mismatch looks like; a pose does not land there.
- **Outside the demonstrated range** warns. Starting from a pose the demonstrations never visited is
  normal — a robot at its park pose before homing will warn, and should.

A checkpoint shipped as `config.json` alone skips the check. One asymmetry worth knowing: bounds
catch values that are too *large* for the training range — degrees where radians were expected. The
reverse sits comfortably inside a degree range and cannot be caught this way.

If the dataset genuinely used different units than NOVA reports, declare the conversion rather than
working around the warning — see [Units](../docs/schema.md#units).

**Derivation needs a checkpoint the client can read.** If `pretrained_name_or_path` names a path
that exists only on the inference server, the NOVA client cannot inspect it — LeRobot's async RPC
has no checkpoint-metadata method. Pass `config_path` pointing at a client-local copy of the
checkpoint directory or its `config.json`:

```python
policy = LeRobotPolicyClient(
    server_address=LEROBOT_SERVER,
    pretrained_name_or_path="/models/my_lerobot_policy",  # read by the server
    config_path="./my_lerobot_policy/config.json",  # read by the client
    fps=15.0,
)
```

Without one, the client warns that neither derivation nor validation could run and falls back to
the explicit arguments; `policy_type` and `actions_per_chunk` then both have to be supplied. The
client never assumes a policy type it could not read — sending the wrong one makes the server build
the wrong policy class.

`load_execution_settings` remains available for reading the same values directly:

```python
from novapolicy.lerobot import load_execution_settings

settings = load_execution_settings("./pretrained_model")
# settings.chunk_size == 11
# settings.n_action_steps == 8
# settings.input_features["observation.state"].shape == (7,)
```

### `actions_per_chunk`

Number of action steps requested from the server. Defaults to the checkpoint's `chunk_size`, so the
full prediction stays available for logging and visualization while `n_action_steps` limits what is
actually executed. Pass it only to request a shorter chunk; a value above `chunk_size` raises.

The LeRobot async server does not infer this. It is part of LeRobot's `RemotePolicyConfig`, and the
server slices `policy.predict_action_chunk(...)` to the requested length.

### Settled ACT chunks

For NOVA waypoint jogging, execute the checkpoint-defined ACT execution horizon and wait for NOVA
to report that its waypoint buffer reached standstill before the next inference:

```python
PolicyExecutor(
    schema,
    policy,
    execution=SequentialExecution(),
    # n_action_steps defaults to the checkpoint's execution horizon,
    # which LeRobotPolicyClient reports. Pass a value to override it,
    # or 0 to execute the whole predicted chunk.
)
```

This prevents the policy observation from being captured while the robot is still moving and avoids
executing the lower-confidence tail beyond the checkpoint's configured execution horizon. The
`SequentialExecution` automatically waits for exact NOVA standstill and, if ACT's first target is
farther away than the spacing inside its executed horizon, prepends a
same-`dt_ms` interpolated bridge to one continuous motion request. IO and computed actions fire when
NOVA's server clock reaches policy waypoint zero; the
robot does not stop at that boundary. Endpoint interpolation allocates additional same-`dt_ms`
intervals for acceleration and braking of each settled request. See
[`docs/executor.md`](../docs/executor.md#bridging-a-distant-first-waypoint).

### `use_async_queue`

Enable this only when the robot transport can track LeRobot's fixed-rate client queue:

```python
from novapolicy.lerobot import AsyncQueueAggregation, LeRobotPolicyClient

policy = LeRobotPolicyClient(
    ...,
    use_async_queue=True,
    async_queue_aggregation=AsyncQueueAggregation.WEIGHTED_AVERAGE,
    async_queue_refill_threshold=0.75,
    async_queue_smoothing=False,
)
```

Aggregation is applied only when an old and a new action target the same future timestep:

| Mode | Merge |
|---|---|
| `WEIGHTED_AVERAGE` | `0.3 * old + 0.7 * new` (LeRobot default) |
| `AVERAGE` | arithmetic mean of every prediction received for the timestep |

The client defaults to `AVERAGE`: physical UR3 plug-task runs and a simulated UR10e
pick-and-place both measured lower peak path curvature with it than with LeRobot's
weighted average, which stays available as `WEIGHTED_AVERAGE`. ``async_queue_smoothing``
is also on by default; it applies the generic ``novapolicy.smooth_action_chunk(...)``
transform to the outgoing aggregated lookahead. The four-point active prefix is restored
unchanged after filtering, and IO action values are never filtered. Disable it for tasks
that need sharp contact transitions.

The client normally consumes one action each policy control tick, requests a refill when 75% of the
previous chunk remains by default, and merges overlapping actions using the selected enum mode.
Refills use LeRobot's `must_go` flag because the server's default one-radian observation similarity
tolerance would otherwise defer most ACT inference until queue depletion. Configure `PolicyExecutor` with
`execution=ContinuousExecution(rate_hz=fps * playback_speed)` and `n_action_steps=0`.
Continuously replaced chunks do not expose per-chunk endpoint ramps.

NOVA's jogger clock advances independently of the Python control loop. Before consuming an action,
the executor maps the latest acknowledged raw NOVA controller timestamp to the absolute LeRobot
timestep; if local work delayed a tick, the client drops every action whose execution time elapsed.
Threshold-triggered inference remains asynchronous while NOVA executes its published lookahead. It
is merged on a later controller-synchronized tick instead of blocking after a timestep has already
been selected. The client then prepends the predecessor from NOVA's published trajectory and retains
the selected action plus two immutable successors. The replacement therefore contains an exact
four-point seam preserving position, velocity, and one-step acceleration context before aggregation
begins. IO remains sourced from the selected current action, not the prepended predecessor.

Between inference updates the client consumes actions internally without resending a shrinking
tail, so existing NOVA waypoints keep their original timeline. The initial queue prediction receives
a measured-state bridge and execution waits for its exact policy-zero boundary. The timestamp
already assigned to policy waypoint zero becomes the immutable action-timestep origin. Later merged
lookaheads use ``origin + action_timestep * policy_dt`` directly in the raw controller-timer domain.
Client wall time, server/client speed-ratio estimation, and post-boundary re-origining are not part of
queue timestamp calculation. Only each integer timestamp sent to NOVA is quantized. The overlapping
prefix is retained instead of being restarted from a measured-state hold at ``now``. This prevents
both catch-up motion and repeated zero-velocity braking/acceleration. In Rerun, the four-point
retained replacement seam is shown in Nova Violet while fresh policy output remains orange. This is
still LeRobot async ACT queue execution, not model-side RTC.

## Observation and action mapping

`LeRobotPolicyClient` maps the NOVA `PolicySchema` into LeRobot feature metadata.

For example:

```python
Observation.joint_positions("arm", source=arm)
Observation.io("gripper", source=arm, io="digital_out[0]", mapping=BoolMapping())
Observation.image("cam_scene_1", source=camera)
```

becomes:

```python
{
    "observation.state": {
        "dtype": "float32",
        "shape": (7,),
        "names": ["arm_1", "arm_2", "arm_3", "arm_4", "arm_5", "arm_6", "gripper"],
    },
    "observation.images.cam_scene_1": {
        "dtype": "image",
        "shape": (240, 320, 3),
        "names": ["height", "width", "channels"],
    },
}
```

Image shape comes from the first camera frame.

### Camera resolution

**Camera resolution stays yours to choose.** Configure the stream in the Camera App at the
resolution and aspect ratio the checkpoint was trained on where the hardware allows it — cameras
do not offer arbitrary resolutions, so novapolicy never overrides your choice. It only tells you
when the frames disagree with the checkpoint.

When the client can read the checkpoint, it compares the first frame against the declared `VISUAL`
shape and warns, distinguishing two cases:

| Difference | What it means |
| --- | --- |
| **Resolution**, same aspect ratio | Frames rescale cleanly. You are paying network and decode cost for pixels that get downscaled again, and losing some detail. Worth fixing when the camera supports it. |
| **Aspect ratio** | Rescaling cannot fix this. `resize=` is a plain rescale, not a crop or a pad, so a 16:9 stream squeezed into a 4:3 target is stretched. pi0, pi0_fast, pi05, smolvla and xvla resize with padding internally and tolerate it; **ACT does not resize at all** and sees the distorted geometry directly. |

Neither stops the run.

```python
# Preferred: the camera stream is already at the checkpoint's resolution.
WebRTCCameras(api_url=CAMERA_API)

# Rescale on read when the hardware cannot deliver the checkpoint's resolution.
WebRTCCameras(api_url=CAMERA_API, resize=(320, 240))
```

`resize=` scales frames on read, after they have crossed the network, so it fixes the shape the
policy sees but not the bandwidth. Match the aspect ratio at the stream even when the resolution
cannot match exactly.

Returned LeRobot actions use a fixed flat layout: joint targets first, then TCP targets, then IO
actions. Joint targets are split according to the schema's joint action motion groups. Each TCP
target contributes six values in NOVA's native format: `[x, y, z, rx, ry, rz]` in millimetres and
rotation-vector radians. IO actions are written once per returned chunk using the first action step.

A motion group can be controlled through joints or TCP, but not both. To observe joints while
controlling the same robot in Cartesian space, disable the inferred joint action explicitly:

```python
schema = PolicySchema(
    observations=[
        Observation.joint_positions("arm", source=arm, action=False),
        Observation.tcp("eef", source=arm, tcp="Flange", action=True),
        Observation.io("gripper", source=arm, io="digital_out[0]", mapping=BoolMapping()),
    ]
)
```

For this schema the action vector is `[eef_x, eef_y, eef_z, eef_rx, eef_ry, eef_rz, gripper]`.

## Protocol notes

The LeRobot async server flow, implemented inside `LeRobotPolicyClient`, is:

1. `Ready` when the policy client connects
2. `SendPolicyInstructions` during the optional policy `prepare(...)` step, while the executor is
   still in `CONNECTING`
3. `SendObservations` with the latest consumed action timestep
4. `GetActions` in a background refill task
5. Consume one queued action per control tick and blend overlapping future timesteps

`PolicyExecutor` triggers this indirectly by calling the `PolicyClient` methods; it does not send
LeRobot RPCs itself. The executor switches to `EXECUTING` after preparation, so
readiness/model-loading time is excluded from `timeout_s`.

`SendPolicyInstructions` contains a pickled LeRobot `RemotePolicyConfig` with:

- `policy_type`
- `pretrained_name_or_path`
- `lerobot_features`
- `actions_per_chunk`
- `device`
- `rename_map` (left empty by this client)

The server does not expose model metadata before setup, which is why the client reads the
checkpoint itself. Keep `fps` in your deployment configuration next to the checkpoint path, and
make sure the client can reach a copy of `config.json` — see
[Checkpoint execution settings](#checkpoint-execution-settings).

## Troubleshooting

### The server cannot load the checkpoint

Check that `pretrained_name_or_path` exists on the LeRobot server machine, not just on the NOVA
client machine.

### `PolicySchema does not match the LeRobot checkpoint`

The schema does not produce what the checkpoint declares, and the client says exactly which keys or
shapes disagree. Common causes: an observation the checkpoint expects and the schema omits, an image
key that does not match the dataset's name, or a state width that is off.

The flat action vector must contain the total DOF of all joint action motion groups, six values for
every TCP action target, and one value for every IO action. A single 6-axis arm with one gripper IO
returns seven values per step; a TCP-controlled arm with one gripper IO also returns seven — six
Cartesian components followed by the IO value.

This check needs a client-readable checkpoint. Without one it cannot run, and the client warns that
it was skipped.

### `Observations are implausibly far outside what this checkpoint was trained on`

The live observation does not resemble the training data — most often a unit mismatch. Compare the
values in the message against the training ranges it prints. If the dataset used different units,
declare the conversion with `ops=` on the observation; see [Units](../docs/schema.md#units).

Warnings about being *outside the training distribution*, as opposed to this error, are expected
before homing: the robot is simply somewhere the demonstrations never visited.

### Missing image errors

`LeRobotPolicyClient` needs the first image frame to declare LeRobot feature metadata, so the camera
has to be connected and streaming before the first inference. A frame older than the executor's
`camera_max_age_s` (1 s by default) counts as stale — see
[stale inputs](../docs/executor.md#stale-inputs-and-how-a-run-ends) for what happens then.
