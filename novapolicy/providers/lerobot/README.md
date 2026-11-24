# Run a LeRobot policy with NOVA PolicyExecutor

This guide shows how to run a trained [LeRobot](https://github.com/huggingface/lerobot) policy on a
NOVA robot.

The setup has two parts:

1. Start LeRobot's async inference server where the policy checkpoint is available.
2. Configure `LeRobotPolicyClient` on the NOVA side and run it with `PolicyExecutor`.

The NOVA side sends observations to the LeRobot server and receives action chunks. The client does
not upload model weights; the LeRobot server resolves `pretrained_name_or_path`.

## Quickstart

### 1. Install

On the NOVA client machine, from this repository:

```bash
uv sync --extra novapolicy-lerobot
```

In your own uv project, add the override below to your `pyproject.toml` before you add the
package. The core package needs Pillow 13 or newer, while LeRobot pins an older one, so pip cannot
install this extra.

```toml
[tool.uv]
override-dependencies = ["pillow>=13.0.0"]
```

```bash
uv add "wandelbots-nova[novapolicy-lerobot]"
```

On the inference server, install LeRobot with its async extra. LeRobot requires Python 3.12:

```bash
mamba create -y -n lerobot-server python=3.12 pip
conda activate lerobot-server
python -m pip install --upgrade pip
python -m pip install 'lerobot[async]>=0.6,<0.7'
```

### 2. Set the cell up

The LeRobot examples in this repository run on a UR10e with the UMI gripper. Use the TCP the
demonstrations were recorded in, `umi_corrected`, not the cell's `Flange`. With the wrong TCP the
policy does not fail; it hovers above the task.

```bash
./novapolicy/examples/setup_cell.sh http://<nova-host> umi
```

[`pick_and_place_umi_ur10e.py`](../../examples/pick_and_place_umi_ur10e.py) runs the same task as a
script. Run it first: if the scripted version cannot pick the cube up, no checkpoint will.

### 3. Put the checkpoint where the LeRobot server can read it

Use either:

- a server-local checkpoint path, for example `/models/pick_place_act/pretrained_model`, or
- a Hugging Face model id supported by `policy_class.from_pretrained(...)`.

For a server-local checkpoint, copy the whole pretrained model directory to the server host:

```bash
scp -r ./pretrained_model user@gpu-server:/models/my_lerobot_policy
```

The NOVA client later passes that same server-side path:

```python
pretrained_name_or_path = "/models/my_lerobot_policy"
```

Keep a copy of the checkpoint's `config.json` (and, if present, `policy_preprocessor.json` with its
safetensors) on the client machine too. The client reads it to derive its settings; see
[checkpoint execution settings](#checkpoint-execution-settings).

### 4. Start the LeRobot async server

Run the server on the machine that has the checkpoint and runs model inference. The server starts
without a checkpoint argument. It loads the checkpoint path the NOVA client sends when it connects.

```bash
# Terminal 1 — inference host
conda activate lerobot-server

python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080
```

Keep this process running. On startup it logs that the gRPC server is listening on port 8080.
The model loads while `PolicyExecutor` is connecting, before its `timeout_s` starts.

The server also accepts `--fps`, `--inference_latency` and `--obs_queue_timeout`. The basic setup
does not need them. If you set the server's `--fps`, keep it equal to the client's `control_freq`.

### 5. Execute the policy through NOVA

Save the following as `run_lerobot_policy.py` on the NOVA client machine. Replace the host names,
controller, camera device, checkpoint paths and IO key with values for your cell and checkpoint.
Model weights only need to exist on the inference server.

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
            control_freq=FPS,  # the dataset's fps
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

This runs one chunk at a time: infer a chunk, execute the checkpoint's action horizon, reach
standstill, infer again. If the checkpoint has no camera input, remove `WebRTCCameras` and
`Observation.image(...)`. Observation names and order must still match the checkpoint's dataset.

### Continuous execution

To infer again while the robot moves, keep the same schema and wrap the client in
`QueuedPolicyClient`:

```python
from novapolicy import ContinuousExecution, QueuedPolicyClient
from novapolicy.providers.lerobot import AsyncQueueAggregation

policy = QueuedPolicyClient(
    LeRobotPolicyClient(
        server_address=LEROBOT_SERVER,
        pretrained_name_or_path=SERVER_CHECKPOINT,
        config_path=CLIENT_CHECKPOINT_CONFIG,
        control_freq=FPS,  # the dataset's fps
        device="cuda",
    ),
    aggregation=AsyncQueueAggregation.AVERAGE,
)
executor = PolicyExecutor(
    schema,
    policy,
    execution=ContinuousExecution(rate_hz=FPS),
    n_action_steps=0,
    timeout_s=80,
)
result = await executor.run()
```

Start with sequential execution. Switch to continuous execution once the checkpoint and the task
work with overlapping action chunks.

## Configuration reference

`LeRobotPolicyClient(server_address, pretrained_name_or_path, *, config_path=None,
actions_per_chunk=None, n_action_steps=None, policy_type=None, task=None, control_freq=None,
device="cpu", timeout_s=15.0)`

| Argument | Meaning | Default |
| --- | --- | --- |
| `server_address` | the LeRobot async inference server, `"host:port"` | required |
| `pretrained_name_or_path` | checkpoint path or Hugging Face model id, resolved on the server | required |
| `config_path` | a client-readable copy of the checkpoint directory or its `config.json` | `pretrained_name_or_path` |
| `control_freq` | rate the returned actions play at, in steps per second: the dataset's `fps` | 30, with a warning |
| `task` | language instruction for checkpoints that condition on one | none |
| `device` | inference device on the server, such as `"cuda"` | `"cpu"` |
| `policy_type` | LeRobot policy type; read from the checkpoint | from the checkpoint |
| `actions_per_chunk` | action steps requested per chunk | the checkpoint's chunk length |
| `n_action_steps` | steps of each chunk that run in sequential execution | the checkpoint's `n_action_steps` |
| `timeout_s` | timeout for one server call, in s | 15 |

### `server_address`

Address of the LeRobot async inference server in `"host:port"` form:

```python
server_address = "127.0.0.1:8080"
server_address = "gpu-server.internal:8080"
```

### `pretrained_name_or_path`

Checkpoint path or Hugging Face model id passed to the LeRobot server. For a remote server, the
path is resolved on the server machine, not on the NOVA client. The client does not upload model
weights.

```python
pretrained_name_or_path = "/models/my_lerobot_policy"
pretrained_name_or_path = "org/my-policy"
```

### `policy_type`

LeRobot policy type sent to the server. It is read from the checkpoint's `type`. Pass it only to
override, and only with a value that agrees with the checkpoint:

```python
policy_type = "act"
```

The client decodes any policy architecture, as long as the server returns a flat action vector
that matches the schema's joint, TCP and IO layout.

### `task`

The language instruction for checkpoints that condition on one: SmolVLA, pi0, pi0.5, X-VLA and any
other policy whose preprocessor pipeline contains LeRobot's `tokenizer_processor` step. It is sent
with every observation as `observation["task"]`, as LeRobot's own robot client does:

```python
client = LeRobotPolicyClient(
    "127.0.0.1:8080",
    "lerobot/smolvla_base",
    task="pick up the cube and place it on the marker",
    control_freq=10,
)
```

Phrase it like the instructions in the training data. Whether a checkpoint needs one is read from
its `policy_preprocessor.json`. `connect()` refuses to start an instruction-conditioned checkpoint
without a task. Checkpoints that need no instruction (ACT, diffusion) ignore it.

### `control_freq`

The rate returned actions play at, in steps per second. It is not how often the policy is asked:
that is `ContinuousExecution(rate_hz=...)`. LeRobot calls this `fps`. It is the dataset's frame
rate (`fps` in its `meta/info.json`) and the `fps` of LeRobot's own robot client. Pass that value
as `control_freq`:

```python
control_freq = 15  # the dataset's fps; ActionChunk.dt_ms = 1000 / 15
```

The default is 30 Hz, with a warning. The frame rate belongs to the dataset, not the policy, so it
cannot be read from the checkpoint. Keep it in your deployment configuration next to the checkpoint
path.

The LeRobot server's own `--fps` flag is separate. The client sets each chunk's `dt_ms` from its
own `control_freq`, and `PolicyExecutor` uses that `dt_ms` to schedule the chunk.

### Checkpoint execution settings

`LeRobotPolicyClient` reads what the checkpoint already declares:

| Derived from the checkpoint | Read from |
| --- | --- |
| `policy_type` | `type` |
| `actions_per_chunk` | `len(action_delta_indices)` |
| `n_action_steps` (used by `PolicyExecutor`) | `n_action_steps` |

Values are read through LeRobot's own `PreTrainedConfig`, so every LeRobot policy works, including
new ones. A policy that declares no action chunk at all (`action_delta_indices is None`) needs
`actions_per_chunk` explicitly.

Pass a derived argument only to override it. An override that contradicts the checkpoint raises:
a different `policy_type`, or a chunk length above the checkpoint's action chunk. A shorter
horizon is allowed, with a warning.

Before the setup message goes out, the client also holds the schema against the checkpoint's
`input_features` and `output_features`: the observation keys, the shape behind each key and the
width of the flat action vector. A mismatch fails at startup, before the robot moves. This runs
automatically. A checkpoint that declares no features is warned about and skipped. Only `STATE`
and `VISUAL` inputs are compared.

A camera frame size that disagrees with the checkpoint only warns; see
[camera resolution](#camera-resolution).

**Derivation needs a checkpoint the client can read.** If `pretrained_name_or_path` names a path
that exists only on the inference server, the client cannot read it. Pass `config_path`, pointing
at a client-local copy of the checkpoint directory or its `config.json`:

```python
policy = LeRobotPolicyClient(
    server_address=LEROBOT_SERVER,
    pretrained_name_or_path="/models/my_lerobot_policy",  # read by the server
    config_path="./my_lerobot_policy/config.json",  # read by the client
    control_freq=15.0,
)
```

Without one, the client warns that it could neither derive nor validate, and uses the explicit
arguments. `policy_type` and `actions_per_chunk` then both have to be passed.

`load_execution_settings` reads the same values directly:

```python
from novapolicy.providers.lerobot import load_execution_settings

settings = load_execution_settings("./pretrained_model")
# settings.chunk_size == 11
# settings.n_action_steps == 8
# settings.input_features["observation.state"].shape == (7,)
```

### Checking the observation against the checkpoint's statistics

Feature shapes prove the schema has the right width. They do not prove it carries the right
numbers: a permuted joint order or a unit mismatch passes the shape check.

When `policy_preprocessor.json` and its safetensors are readable (next to `config.json`, on the
Hub or locally), the client holds the first live observation against the checkpoint's training
statistics:

- **More than five times larger** than anything in training raises before the robot moves. That is
  what a unit mismatch looks like.
- **More than one training span outside the demonstrated range** warns. This is normal at a pose
  the demonstrations never visited, such as a park pose before homing.

A checkpoint shipped as `config.json` alone skips the check. The check catches values that are too
large for the training range, such as degrees where radians were expected. It cannot catch the
reverse.

If the dataset used other units than NOVA reports, declare the conversion; see
[Units](../../docs/schema.md#units).

### `actions_per_chunk`

Number of action steps requested from the server. It defaults to the checkpoint's chunk length, so
the full prediction stays available for logging and visualization, while `n_action_steps` limits
what is executed. Pass it only to request a shorter chunk. A value above the chunk length raises.

### Sequential execution

With `SequentialExecution`, the executor runs the checkpoint's action horizon and waits until the
robot stands still before the next inference:

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

The policy then never observes a robot in motion, and the less certain tail of each chunk beyond
the execution horizon does not run. If the first target of a chunk is far from the robot, the
executor adds a bridge to it. See [executor.md](../../docs/executor.md).

### Continuous execution with `QueuedPolicyClient`

For continuous execution, wrap the client in `QueuedPolicyClient`, as for any policy that answers
a chunk per observation. It needs `ContinuousExecution`; `PolicyExecutor` refuses it with
`SequentialExecution`. Configure `ContinuousExecution(rate_hz=control_freq)` and
`n_action_steps=0`.

`aggregation` says how a new prediction for a step that is already queued is merged with it:

| Mode | Merge |
| --- | --- |
| `WEIGHTED_AVERAGE` | `0.3 * old + 0.7 * new` (LeRobot's default, and the queue's) |
| `AVERAGE` | arithmetic mean of every prediction received for the timestep |

The executor's default pipeline smooths the merged plan (`Smoothing`). For tasks that need sharp
contact transitions, pass the defaults without it:

```python
from novapolicy import ContinuousExecution, Smoothing, default_pipeline

execution = ContinuousExecution(rate_hz=FPS)
pipeline = [
    m for m in default_pipeline(execution=execution, policy=policy) if not isinstance(m, Smoothing)
]
```

[pipeline.md](../../docs/pipeline.md) describes the modules.

## Observation and action mapping

`LeRobotPolicyClient` maps the NOVA `PolicySchema` into LeRobot feature metadata. For example:

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

`observation.state` is filled in a fixed order, whatever order the schema declares: joint
positions, joint velocities, TCP poses, object poses (`Observation.pose`), IOs, then the keys of
`Observation.computed` (one value each). The checkpoint matches the values by position, so record
the dataset in the same order.

The image shape comes from the first camera frame.

Returned LeRobot actions use a fixed flat layout: joint targets first, then TCP targets, then IO
actions, then the keys of `Action.computed(..., keys=[...])` (one value each). Joint targets are
split according to the schema's joint action motion groups. Each TCP target contributes six values,
by default in NOVA's format: `[x, y, z, rx, ry, rz]` in millimetres and rotation-vector radians.
With `orientation_type=OrientationType.QUATERNION` it contributes seven, and the entry's operators
convert the units. IO actions are written once per returned chunk, from the first action step.

A motion group is controlled through joints or TCP, not both. To observe joints while controlling
the same robot in Cartesian space, turn off the joint action:

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

### Camera resolution

Configure the stream in the Camera App at the resolution and aspect ratio the checkpoint was
trained on, where the hardware allows it. novapolicy never overrides your choice. It only warns
when the frames disagree with the checkpoint.

When the client can read the checkpoint, it compares the first frame with the declared `VISUAL`
shape:

| Difference | What it means |
| --- | --- |
| **Resolution**, same aspect ratio | Frames rescale cleanly. You pay network and decode cost for pixels that are scaled down again, and lose some detail. Fix it when the camera supports it. |
| **Aspect ratio** | Rescaling cannot fix this. `resize=` alone stretches a 16:9 stream into a 4:3 target; `resize_mode="crop"` centre-crops to the target's aspect ratio first, which cuts the image's edges instead. pi0, pi0_fast, pi05, smolvla and xvla resize with padding internally and tolerate it. **ACT does not resize** and sees the distorted image. |

Neither stops the run.

```python
# Preferred: the camera stream is already at the checkpoint's resolution.
WebRTCCameras(api_url=CAMERA_API)

# Rescale on read when the hardware cannot deliver the checkpoint's resolution.
WebRTCCameras(api_url=CAMERA_API, resize=(320, 240))

# Centre-crop to the checkpoint's aspect ratio, then rescale.
WebRTCCameras(api_url=CAMERA_API, resize=(320, 240), resize_mode="crop")
```

`resize=` scales frames on read, after they have crossed the network. It fixes the shape the policy
sees, not the bandwidth. Match the aspect ratio at the stream even when the resolution cannot
match exactly.

## Troubleshooting

### The server cannot load the checkpoint

Check that `pretrained_name_or_path` exists on the LeRobot server machine, not just on the NOVA
client machine.

### `LeRobotPolicyClient cannot read the checkpoint at ...`

The client has no readable copy of the checkpoint. Pass `config_path` to a local copy, or pass
`policy_type` and `actions_per_chunk` explicitly.

### `PolicySchema does not match the LeRobot checkpoint`

The schema does not produce what the checkpoint declares. The message names the keys or shapes
that disagree. Common causes: an observation the checkpoint expects and the schema omits, an image
key that does not match the dataset's name, or a state width that is off.

The flat action vector must contain the total DOF of all joint action motion groups, six values for
every TCP action target (seven with a quaternion), one value for every IO action and one for every
`Action.computed` key. A single 6-axis arm with one gripper IO returns seven values per step. A
TCP-controlled arm with one gripper IO also returns seven: six Cartesian values, then the IO value.

This check needs a client-readable checkpoint. Without one, the client warns that it was skipped.

### `LeRobot server returned no actions`

The server answers a failed inference with the same empty message as a missing observation, and
writes the real error only to its own log. Look there first. A `KeyError: 'task'` means the
checkpoint needs a language instruction: pass `task=`. The client normally catches that at
`connect()`, but not when it has no readable copy of the checkpoint.

### `... is conditioned on a language instruction. Pass task='...'`

Pass `task=` with an instruction phrased like those in the training data.

### `policy_type=... contradicts the checkpoint` or `... exceeds the checkpoint's action chunk`

Remove the override, or pass a value the checkpoint allows.

### `Observations are implausibly far outside what this checkpoint was trained on`

The live observation does not resemble the training data, most often because of a unit mismatch.
Compare the values in the message with the training ranges it prints. If the dataset used other
units, declare the conversion with `ops=` on the observation; see
[Units](../../docs/schema.md#units).

Warnings about being *outside the training distribution*, as opposed to this error, are expected
before homing: the robot is somewhere the demonstrations never visited.

### The robot hovers above the task and never grasps

Check the TCP. It has to be the TCP the demonstrations were recorded in.

### Missing image errors

`LeRobotPolicyClient` needs the first image frame to declare LeRobot feature metadata, so the camera
has to be connected and streaming before the first inference. A frame older than the executor's
`camera_max_age_s` (1 s by default) counts as stale, and the run stops. See
[executor.md](../../docs/executor.md) for stale inputs.
